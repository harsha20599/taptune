// Book Piano · touch sensing + MIDI (USB / Bluetooth) + ESP-NOW
// Runs on ESP32-S3, ESP32-S2 and the classic ESP32 — see compat.h.
//
// Two boards read the books (one is enough for up to 14 / 9 books):
//   PARENT (default build): plugged into the phone/laptop or just powered.
//     Reads its touch pins, receives the child's over ESP-NOW, and plays
//     everything as "Book Piano" over Bluetooth MIDI and/or USB MIDI. It owns
//     the configuration (config.ino).
//   CHILD (build with -DBOOK_PIANO_CHILD): only needs power. Reads its touch
//     pins and sends touches + telemetry to the parent.
// They pair by themselves; the child finds the parent's radio channel.
//
// Model (config.ino): a SENSOR is a board + slot (14 slots per board, 28 in
// total; slot i = the board's i-th touch pin, PINS[i]). A BOOK is a position
// on the shelf; each book points at one sensor and has a note. Each sensor has
// its own touch threshold, set by TapTune's setup wizard.
//
// Build: see README.md (Arduino IDE or arduino-cli). ESP32-S3 example:
//   FQBN esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,USBMode=default,CDCOnBoot=cdc
//
// MIDI from the web page: Control Change 119 = re-baseline (both boards),
// 115 = update mode; the board answers CC 116 (1 = re-baselined, 4 = update
// mode). Configuration and calibration use SysEx (config.ino).
//
// Serial commands (native USB or the COM/UART port):
//   r  re-baseline every pin (hands off!)      v  toggle the visualiser stream
//   s  status                                  u  update mode (both boards)
//   C<1-13>  parent: radio channel, saved, reboots
//   c<number>  touch charge cycles (speed vs. signal), saved, reboots
//   w<number>  touch power-on wait in µs, saved, reboots
//   W  save Wi-Fi for update mode (tools\set_wifi.py)

#include "compat.h"
#include <Preferences.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#ifdef BOOK_PIANO_CHILD
const bool IS_CHILD = true;
#else
const bool IS_CHILD = false;
#endif

const char *FW_VERSION = "3.0";
const uint8_t FW_MAJOR = 3, FW_MINOR = 0;
const uint8_t MIDI_CHANNEL = 1;

// ---- touch pins: PINS[] (GPIO numbers) comes from compat.h ----
const uint8_t NUM_PINS = sizeof(PINS);       // real touch pins on this board (14 on S2/S3, 9 on ESP32)
const uint8_t SLOTS = 14;                    // sensor ids per board, the same on every chip
const uint8_t MAX_SENSORS = 2 * SLOTS;       // main board 0–13, second board 14–27
const uint16_t ALL_PINS = (1 << NUM_PINS) - 1;
static_assert(NUM_PINS <= SLOTS, "more touch pins than slots");

// ---- touch tuning ----
// touch threshold: % above resting (bp = 0.01 %). The classic ESP32's readings
// are coarse (a few dozen counts), so it starts higher.
const uint16_t DEFAULT_THR_BP = TOUCH_FALLS ? 1500 : 300;
const float    NOISE_FLOOR_MULT = 4.0f;  // a threshold is never closer than 4x the idle noise
const float    RELEASE_FRAC   = 0.4f;    // release at 40 % of the touch threshold (hysteresis)
const uint32_t ON_CONFIRM_US  = 1000;    // must stay above threshold this long (rejects spikes)
const uint32_t OFF_CONFIRM_US = 25000;   // must stay below release this long (a noisy dip mid-press won't cut the note)
const uint8_t  MEDIAN_N       = 5;       // spike filter: each pin uses the middle of its last 5 readings
const uint8_t  MIN_SCANNED    = 6;       // pins with books + spare "reference" pins, for ground-noise rejection
const uint32_t RETRIGGER_US   = 40000;   // after a release, ignore new touches this long (no double notes)
const float    DRIFT_TAU_MS   = 1500.0f; // baseline follows slow wander while idle...
const float    DROP_TAU_MS    = 500.0f;  // ...faster when the reading falls (a finger only raises it)
const uint32_t DRIFT_HOLD_US  = 300000;  // baseline frozen this long after a release
const uint32_t STUCK_MS       = 8000;    // a pin "held" longer than this is re-baselined
const uint32_t FROZEN_MS      = 1500;    // a "touch" that doesn't wobble at all for this long isn't a finger:
                                         // the book/foil/wire shifted. Release and take it as the new rest level.
const float    CAL_MAX_NOISE  = 0.02f;   // calibration noisier than 2 % of baseline = someone moved; retry
const uint32_t STREAM_EVERY_MS = 10;     // visualiser stream rate (100 Hz)
const uint32_t DEFAULT_CHARGE = 500;     // touch charge cycles per reading (chip default 500)
const uint32_t DEFAULT_WAIT_US = 256;    // touch power-on wait before each pad (core default 256)

// ---- radio ----
const uint8_t  DEFAULT_WIFI_CH = 1;      // parent's channel until set with C<n>; child searches for it
const uint32_t HOP_EVERY_MS   = 120;     // unpaired child tries the next channel this often
const uint8_t  GROUP          = 0x42;    // ignore other Book Pianos nearby
const uint32_t STATE_EVERY_MS = 20;      // child resends all pin states this often (no lost note-offs)
const uint32_t CHILD_TIMEOUT_MS = 150;   // parent releases child notes if it hears nothing this long
const uint32_t PING_EVERY_MS  = 500;     // parent measures the link round trip
const uint32_t PARENT_LOST_MS = 3000;    // child goes back to pairing if parent is silent this long

#if HAS_USB_MIDI
USBMIDI MIDI("Book Piano");
#else
NoUsbMidi MIDI("Book Piano");  // no native USB on this chip/build: Bluetooth only
#endif
Preferences prefs;

struct Pin {
  float baseline;
  float noise;        // average |reading - baseline| at calibration
  float onDelta, offDelta;
  uint16_t thrBp;     // configured threshold (from the parent's config)
  bool touched;
  uint32_t aboveSince, belowSince, touchedAt, releasedAt;  // micros / micros / millis / micros
  uint32_t last;      // last reading
  float peak;         // highest delta in the current touch
  uint16_t peakBp;    // highest rise (bp) since last reported (telemetry for calibration)
  uint32_t hist[MEDIAN_N];  // last raw readings, for the spike filter
  uint8_t hn;
  uint32_t lastRaw;   // last unfiltered reading (to spot new readings)
  float flatRef;      // frozen-touch detector: level the signal has stayed near...
  uint32_t flatSince; // ...since this time (ms)
};

// Only pins with books are scanned (plus a few spare ones as ground-noise
// references): fewer pins = each read more often. Fixed at boot (NVS "mask").
uint16_t scanMask = ALL_PINS;
inline bool scanned(uint8_t i) { return scanMask >> i & 1; }
uint16_t maskFor(uint16_t used) {  // books' pins, topped up with spare pins to MIN_SCANNED
  uint16_t m = used & ALL_PINS;
  uint8_t n = __builtin_popcount(m);
  for (uint8_t i = 0; i < NUM_PINS && n < MIN_SCANNED; i++) if (!(m >> i & 1)) { m |= 1 << i; n++; }
  return m ? m : ALL_PINS;
}
uint16_t childUsed = 0x0FFF;  // child: which of its slots have books (from the parent)
uint32_t restartAtMs = 0;     // a new scan mask needs a restart (touch pins are set up at boot)
void scheduleMaskRestart(uint16_t m) {
  if (m == scanMask) return;
  prefs.begin("bookpiano", false); prefs.putUShort("mask", m); prefs.end();
  restartAtMs = millis() + 600;
}
uint32_t median5(const uint32_t *h, uint8_t n) {
  uint32_t a[MEDIAN_N];
  memcpy(a, h, n * sizeof(uint32_t));
  for (uint8_t i = 1; i < n; i++) { uint32_t v = a[i]; int8_t j = i - 1; while (j >= 0 && a[j] > v) { a[j + 1] = a[j]; j--; } a[j + 1] = v; }
  return a[n / 2];
}
Pin pins[NUM_PINS];

float cmFrac = 0;       // current common (ground-noise) rise, fraction of baseline — see scanPins()
uint16_t cmPeakBp = 0;  // highest common rise since last reported (telemetry)
bool streaming = true;
bool busy = false;  // inside calibrate()
uint32_t chargeCycles = DEFAULT_CHARGE, powerWaitUs = DEFAULT_WAIT_US;
uint32_t scans = 0, scanHz = 0, lastHzAt = 0, lastStreamAt = 0;
uint32_t freshCount = 0, sensorHz = 0;  // new readings per second from pin 0 = real sensor rate

// in config.ino (parent) — declared here because this file comes first
void sensorTouch(uint8_t sensor, bool on);
void loadConfig();
uint16_t childThrHash();
void sendChildConfig();
bool sysexPacket(const midiEventPacket_t &p);
void configLoop();
void printConfig(Stream &out);
void setRadioChannel(long v, Stream &out);
void setBooksFromSerial(Stream &in);
void bleBegin();
void bleLoop();
void bleSendSettings(const uint8_t *d, size_t n);
void outNoteOn(uint8_t note, uint8_t vel);
void outNoteOff(uint8_t note);
void outCC(uint8_t cc, uint8_t val);
void handleSysex(const uint8_t *d, size_t n);
extern volatile bool bleConnected;
void dumpPins(Stream &out);
void setScanAll(bool on);
void waveSample(uint8_t s, float riseFrac, uint32_t nowUs);
extern bool updateMode;  // update_mode.ino
void enterUpdateMode();
void updateLoop();
bool bootIntoUpdateMode();

uint16_t bp(float delta, float base) {  // rise as basis points of the baseline, clamped
  if (base <= 0 || delta <= 0) return 0;
  float v = delta / base * 10000.0f;
  return v > 16383 ? 16383 : (uint16_t)v;
}

// ================= radio messages =================
enum : uint8_t { M_HELLO = 1, M_WELCOME, M_EVENT, M_STATE, M_CMD, M_ACK, M_PING, M_PONG, M_CFG };
struct __attribute__((packed)) Hdr { uint8_t magic, group, type, seq; };
struct __attribute__((packed)) MsgEvent { Hdr h; uint8_t pin, on; };
struct __attribute__((packed)) PinTele { uint32_t raw, base; uint16_t thrBp, noiseBp, peakBp; };
struct __attribute__((packed)) MsgState {  // n = the child's real pin count, gpio = their numbers
  Hdr h; uint16_t touched; uint16_t fails; uint16_t cfgHash; uint16_t cmBp; uint8_t n; uint8_t chip;
  uint8_t gpio[SLOTS]; PinTele p[SLOTS];
};
struct __attribute__((packed)) MsgCmd { Hdr h; char c; };
struct __attribute__((packed)) MsgAck { Hdr h; uint8_t code; };
struct __attribute__((packed)) MsgPing { Hdr h; uint32_t t; };
struct __attribute__((packed)) MsgCfg { Hdr h; uint16_t thrBp[SLOTS]; uint16_t used; };  // used: child slots with books
static_assert(sizeof(MsgState) <= ESP_NOW_MAX_DATA_LEN, "state message too big");

struct RxItem { uint8_t mac[6]; uint8_t len; uint8_t data[ESP_NOW_MAX_DATA_LEN]; };
QueueHandle_t rxQueue;
const uint8_t BROADCAST[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
uint8_t peerMac[6];
bool paired = false;
uint8_t wifiCh = DEFAULT_WIFI_CH;  // NVS "ch": the parent's choice, or where the child last found it
uint8_t txSeq = 0;
volatile uint32_t sendFails = 0;
uint32_t lastHeardMs = 0;

// parent's view of the child
struct Remote { bool touched; PinTele t; uint16_t peakBp; };  // peakBp: max since last telemetry frame
Remote remote[SLOTS];
bool childOnline = false;
uint8_t childPins = 0, childChip = 0;
uint8_t childGpio[SLOTS];
uint16_t childCfgHash = 0;
uint16_t childCmBp = 0;  // child's highest common (ground-noise) rise since last telemetry frame
uint32_t childFails = 0, rttUs = 0, lastPingMs = 0;

void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len < (int)sizeof(Hdr) || len > ESP_NOW_MAX_DATA_LEN) return;
  RxItem it;
  memcpy(it.mac, info->src_addr, 6);
  it.len = len;
  memcpy(it.data, data, len);
  xQueueSend(rxQueue, &it, 0);  // handled in loop(), never in the Wi-Fi task
}
void onSent(const esp_now_send_info_t *, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) sendFails++;
}

void addPeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return;
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, mac, 6);
  p.channel = 0;  // = whatever channel the radio is on (the child changes channel while searching)
  p.encrypt = false;
  esp_now_add_peer(&p);
}

template <typename T> void send(const uint8_t *mac, T &msg, uint8_t type) {
  msg.h = Hdr{ 'B', GROUP, type, txSeq++ };
  esp_now_send(mac, (const uint8_t *)&msg, sizeof(T));
}

void radioBegin() {
  rxQueue = xQueueCreate(24, sizeof(RxItem));
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  // Child: radio never naps (no added delay). Parent also runs Bluetooth, and
  // Wi-Fi + Bluetooth must share the radio, which requires "modem sleep"; with
  // no Wi-Fi network joined it doesn't actually sleep, so ESP-NOW stays fast.
  esp_wifi_set_ps(IS_CHILD || !HAS_BLE ? WIFI_PS_NONE : WIFI_PS_MIN_MODEM);
  esp_wifi_set_channel(wifiCh, WIFI_SECOND_CHAN_NONE);
  esp_now_init();
  esp_now_register_recv_cb(onRecv);
  esp_now_register_send_cb(onSent);
  addPeer(BROADCAST);
  Serial.printf("{\"ev\":\"radio\",\"role\":\"%s\",\"mac\":\"%s\",\"ch\":%u}\n", IS_CHILD ? "child" : "parent",
                WiFi.macAddress().c_str(), wifiCh);
}

// ================= pin thresholds (each board keeps its own 14 slots) =================
uint16_t thrSlots[SLOTS];  // as received/saved; slots past NUM_PINS are kept only for the checksum

void updateThresholds(Pin &p) {
  float d = max(p.thrBp / 10000.0f * p.baseline, NOISE_FLOOR_MULT * p.noise);
  p.onDelta = d;
  p.offDelta = d * RELEASE_FRAC;
}

uint16_t thrHash(const uint16_t *t) {  // tiny checksum so the parent can tell if the child is in sync
  uint16_t h = 0x1234;
  for (uint8_t i = 0; i < SLOTS; i++) h = (h << 5 | h >> 11) ^ t[i];
  return h;
}

uint16_t localThrHash() { return thrHash(thrSlots) ^ (IS_CHILD ? childUsed : 0); }

// Apply and save this board's thresholds (parent: from its config; child: from the parent).
void setPinThresholds(const uint16_t *t) {
  bool changed = false;
  for (uint8_t i = 0; i < SLOTS; i++) {
    uint16_t v = constrain(t[i], 20, 16000);
    changed |= thrSlots[i] != v;
    thrSlots[i] = v;
    if (i < NUM_PINS) { pins[i].thrBp = v; updateThresholds(pins[i]); }
  }
  if (!changed) return;  // spare the flash
  prefs.begin("bookpiano", false);
  prefs.putBytes("thr", thrSlots, sizeof(thrSlots));
  prefs.end();
}

void loadPinThresholds() {
  for (uint8_t i = 0; i < SLOTS; i++) thrSlots[i] = DEFAULT_THR_BP;
  prefs.begin("bookpiano", true);
  if (prefs.getBytesLength("thr") == sizeof(thrSlots)) prefs.getBytes("thr", thrSlots, sizeof(thrSlots));
  prefs.end();
  for (uint8_t i = 0; i < NUM_PINS; i++) pins[i].thrBp = thrSlots[i];
}

// ================= touch → note / message =================
void sendEvent(uint8_t i, bool on) {
  if (!paired) return;
  MsgEvent m; m.pin = i; m.on = on;
  send(peerMac, m, M_EVENT);
}

void localTouch(uint8_t i, bool on) {
  if (IS_CHILD) sendEvent(i, on);
  else sensorTouch(i, on);  // main board pin i = sensor i
}

void setRemote(uint8_t i, bool on) {
  if (remote[i].touched == on) return;
  remote[i].touched = on;
  sensorTouch(SLOTS + i, on);  // second board slot i = sensor 14+i
}

void releaseChild() {
  for (uint8_t i = 0; i < SLOTS; i++) setRemote(i, false);
}

// ================= child: send all states + telemetry =================
void sendState() {
  if (!paired) return;
  MsgState m = {};
  m.n = NUM_PINS;
  m.chip = CHIP_ID;
  memcpy(m.gpio, PINS, NUM_PINS);
  m.fails = sendFails;
  m.cfgHash = localThrHash();
  m.cmBp = cmPeakBp;
  cmPeakBp = 0;
  for (uint8_t i = 0; i < NUM_PINS; i++) {
    Pin &p = pins[i];
    if (p.touched) m.touched |= 1 << i;
    m.p[i] = PinTele{ p.last, (uint32_t)p.baseline, bp(p.onDelta, p.baseline), bp(p.noise, p.baseline), p.peakBp };
    p.peakBp = 0;
  }
  send(peerMac, m, M_STATE);
}

void calibrate(); void releaseLocal();

// ================= radio: handle received messages =================
void handleRadio() {
  RxItem it;
  while (xQueueReceive(rxQueue, &it, 0) == pdTRUE) {
    const Hdr *h = (const Hdr *)it.data;
    if (h->magic != 'B' || h->group != GROUP) continue;

    if (!IS_CHILD) {
      // parent: the (one) child is whoever talks to us
      if (!paired || memcmp(peerMac, it.mac, 6) != 0) {
        memcpy(peerMac, it.mac, 6); addPeer(peerMac); paired = true;
        Serial.printf("{\"ev\":\"child\",\"what\":\"paired\",\"mac\":\"%02x:%02x:%02x:%02x:%02x:%02x\"}\n",
                      it.mac[0], it.mac[1], it.mac[2], it.mac[3], it.mac[4], it.mac[5]);
      }
      lastHeardMs = millis();
      if (!childOnline) { childOnline = true; Serial.println("{\"ev\":\"child\",\"what\":\"online\"}"); }
      switch (h->type) {
        case M_HELLO: { MsgAck w; w.code = 0; send(peerMac, w, M_WELCOME); sendChildConfig(); break; }
        case M_EVENT: {
          const MsgEvent *e = (const MsgEvent *)it.data;
          if (e->pin < SLOTS) setRemote(e->pin, e->on);
          break;
        }
        case M_STATE: {  // catches anything an event message missed
          if (it.len != sizeof(MsgState)) {  // child on other firmware: don't misread it
            static uint32_t warnedMs = 0;
            if (millis() - warnedMs > 5000) { warnedMs = millis(); Serial.println("{\"ev\":\"child\",\"what\":\"firmware mismatch\"}"); }
            break;
          }
          const MsgState *s = (const MsgState *)it.data;
          childPins = min<uint8_t>(s->n, SLOTS);
          childChip = s->chip;
          memcpy(childGpio, s->gpio, SLOTS);
          childFails = s->fails;
          childCfgHash = s->cfgHash;
          childCmBp = max(childCmBp, s->cmBp);
          for (uint8_t i = 0; i < childPins; i++) {
            remote[i].t = s->p[i];
            remote[i].peakBp = max(remote[i].peakBp, s->p[i].peakBp);
            if (s->p[i].base) waveSample(SLOTS + i, ((float)s->p[i].raw - s->p[i].base) / s->p[i].base - s->cmBp / 10000.0f, micros());
            setRemote(i, s->touched & (1 << i));
          }
          break;
        }
        case M_PONG: rttUs = micros() - ((const MsgPing *)it.data)->t; break;
        case M_ACK: {
          const char *what[] = { "?", "re-baselined", "?", "?", "update-mode", "config" };
          uint8_t c = ((const MsgAck *)it.data)->code;
          Serial.printf("{\"ev\":\"child\",\"what\":\"%s\"}\n", what[c < 6 ? c : 0]);
          break;
        }
      }
    } else {
      // child
      if (h->type == M_WELCOME || paired) lastHeardMs = millis();
      switch (h->type) {
        case M_WELCOME:
          if (!paired || memcmp(peerMac, it.mac, 6) != 0) {
            memcpy(peerMac, it.mac, 6); addPeer(peerMac); paired = true;
            uint8_t ch; wifi_second_chan_t second;
            esp_wifi_get_channel(&ch, &second);
            if (ch != wifiCh) {  // found the parent on a new channel: start there next time
              wifiCh = ch;
              prefs.begin("bookpiano", false); prefs.putUChar("ch", ch); prefs.end();
            }
            Serial.printf("{\"ev\":\"paired\",\"ch\":%u}\n", ch);
          }
          break;
        case M_PING: { MsgPing p = *(const MsgPing *)it.data; send(peerMac, p, M_PONG); break; }
        case M_CFG: {
          const MsgCfg *c = (const MsgCfg *)it.data;
          uint16_t t[SLOTS];
          memcpy(t, c->thrBp, sizeof(t));
          setPinThresholds(t);  // only writes flash if they changed
          if (it.len >= sizeof(MsgCfg) && c->used != childUsed) {
            childUsed = c->used & ALL_PINS;
            prefs.begin("bookpiano", false); prefs.putUShort("used", childUsed); prefs.end();
            scheduleMaskRestart(maskFor(childUsed));
          }
          MsgAck a; a.code = 5; send(peerMac, a, M_ACK);
          Serial.println("{\"ev\":\"config\",\"from\":\"parent\"}");
          break;
        }
        case M_CMD: {
          if (busy) break;  // already calibrating (we're being called from inside it)
          char c = ((const MsgCmd *)it.data)->c;
          MsgAck a;
          if (c == 'u') { a.code = 4; send(peerMac, a, M_ACK); delay(20); enterUpdateMode(); break; }
          if (c == 'r') { calibrate(); a.code = 1; send(peerMac, a, M_ACK); }
          break;
        }
      }
    }
  }
}

// Radio upkeep. Also called from inside calibrate() so the link stays alive.
void background() {
  static uint32_t lastStateMs = 0, lastHelloMs = 0;
  uint32_t now = millis();
  handleRadio();
  if (IS_CHILD) {
    static uint8_t hopCh = 0, hops = 0;
    if (paired && now - lastHeardMs > PARENT_LOST_MS) { paired = false; hops = 0; Serial.println("{\"ev\":\"unpaired\"}"); }
    // Unpaired: say hello on the last known channel first, then try 1–13 in turn
    // until the parent answers (it may have moved channel).
    if (!paired && now - lastHelloMs >= HOP_EVERY_MS) {
      lastHelloMs = now;
      if (hops == 0) hopCh = wifiCh;
      else hopCh = hopCh % 13 + 1;
      hops = hops < 250 ? hops + 1 : 1;
      esp_wifi_set_channel(hopCh, WIFI_SECOND_CHAN_NONE);
      MsgAck m; m.code = 0;
      send(BROADCAST, m, M_HELLO);
    }
    if (paired) hops = 0;
    if (paired && now - lastStateMs >= STATE_EVERY_MS) { lastStateMs = now; sendState(); }
  } else {
    if (childOnline && now - lastHeardMs > CHILD_TIMEOUT_MS) {
      childOnline = false; releaseChild();
      Serial.println("{\"ev\":\"child\",\"what\":\"offline\"}");
    }
    if (paired && now - lastPingMs >= PING_EVERY_MS) { lastPingMs = now; MsgPing p; p.t = micros(); send(peerMac, p, M_PING); }
    // child's thresholds out of date (lost message, or it was reflashed)? resend, at most once a second
    static uint32_t lastCfgSyncMs = 0;
    if (childOnline && childCfgHash != childThrHash() && now - lastCfgSyncMs > 1000) {
      lastCfgSyncMs = now;
      sendChildConfig();
    }
  }
}

void tellChild(char c) {
  if (IS_CHILD || !paired) return;
  MsgCmd m; m.c = c;
  send(peerMac, m, M_CMD);
}

// ================= calibration (resting level + noise) =================
void releaseLocal() {
  for (uint8_t i = 0; i < NUM_PINS; i++) {
    Pin &p = pins[i];
    if (p.touched) localTouch(i, false);
    p.touched = false; p.aboveSince = p.belowSince = 0;
  }
}

// Average each pin while nobody touches it, and measure its idle noise. If a
// pin is unusually noisy (usually a hand still moving nearby) try again.
void calibrate() {
  busy = true;
  releaseLocal();
  uint32_t start = millis();
  while (millis() - start < 300) {  // first reads after boot are inflated
    for (uint8_t i = 0; i < NUM_PINS; i++) if (scanned(i)) readTouch(PINS[i]);
    background();
  }
  for (uint8_t attempt = 1; attempt <= 3; attempt++) {
    double sum[NUM_PINS] = {}, dev[NUM_PINS] = {};
    uint32_t n = 0;
    start = millis();
    while (millis() - start < 500) {
      for (uint8_t i = 0; i < NUM_PINS; i++) if (scanned(i)) sum[i] += readTouch(PINS[i]);
      n++;
      background();
      delayMicroseconds(200);
    }
    for (uint8_t i = 0; i < NUM_PINS; i++) pins[i].baseline = scanned(i) ? sum[i] / n : 0;
    n = 0;
    start = millis();
    uint32_t h[NUM_PINS][MEDIAN_N];
    uint8_t hn = 0;
    while (millis() - start < 300) {  // noise of the FILTERED signal, i.e. what touch decisions see
      for (uint8_t i = 0; i < NUM_PINS; i++) {
        if (!scanned(i)) continue;
        h[i][hn % MEDIAN_N] = readTouch(PINS[i]);
        if (hn >= MEDIAN_N) dev[i] += fabs((double)median5(h[i], MEDIAN_N) - pins[i].baseline);
      }
      if (++hn >= MEDIAN_N) n++;
      if (hn >= 2 * MEDIAN_N) hn = MEDIAN_N;
      background();
      delayMicroseconds(200);
    }
    bool calm = true;
    for (uint8_t i = 0; i < NUM_PINS; i++) {
      pins[i].noise = n ? dev[i] / n : 0;
      if (scanned(i) && pins[i].noise > CAL_MAX_NOISE * pins[i].baseline) calm = false;
    }
    if (calm) break;
    Serial.printf("{\"ev\":\"calretry\",\"attempt\":%u}\n", attempt);
  }
  for (uint8_t i = 0; i < NUM_PINS; i++) {
    Pin &p = pins[i];
    p.noise = max(p.noise, 1.0f);
    p.peakBp = 0;
    for (uint8_t j = 0; j < MEDIAN_N; j++) p.hist[j] = (uint32_t)p.baseline;  // filter starts at rest
    p.hn = MEDIAN_N;
    updateThresholds(p);
    if (!scanned(i)) continue;
    Serial.printf("{\"ev\":\"cal\",\"k\":%u,\"base\":%.0f,\"noiseBp\":%u,\"thrBp\":%u}\n", i, p.baseline,
                  bp(p.noise, p.baseline), bp(p.onDelta, p.baseline));
  }
  busy = false;
}

// ================= setup / loop =================
void setup() {
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);  // printing must never stall the scan loop: drop data instead of waiting
  Serial.begin(921600);
#else
  // Serial is a UART (USB-serial chip): standard speed so any serial monitor
  // reads it; the 100/s visualiser stream would be too much, so it starts off.
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);
  streaming = false;
#endif
#if HAS_SERIAL0
  Serial0.begin(115200);  // COM/UART port: commands + update-mode status
#endif
  MIDI.begin();
#if HAS_USB_MIDI
  USB.begin();
#endif

  prefs.begin("bookpiano", false);
  chargeCycles = prefs.getUInt("charge", DEFAULT_CHARGE);
  wifiCh = prefs.getUChar("ch", DEFAULT_WIFI_CH);
  powerWaitUs = prefs.getUInt("wait", DEFAULT_WAIT_US);
  char name[8];
  for (uint8_t i = 0; i < 12; i++) {  // drop settings from older firmware
    snprintf(name, sizeof(name), "peak%u", i); prefs.remove(name);
    snprintf(name, sizeof(name), "lrn%u", i); prefs.remove(name);
  }
  prefs.end();

  for (uint8_t i = 0; i < NUM_PINS; i++) pins[i] = Pin{};
  loadPinThresholds();
  prefs.begin("bookpiano", true);
  childUsed = prefs.getUShort("used", 0x0FFF);
  prefs.end();
  if (IS_CHILD) scanMask = maskFor(childUsed);
  else loadConfig();  // also applies the main board's thresholds and sets scanMask

  // Speed vs. signal, both must be set before the first touchRead():
  // fewer charge cycles = quicker reading, smaller signal;
  // shorter power-on wait = less idle time before each pad is measured.
  // (S2/S3 only; the classic ESP32 keeps the core's defaults.)
#if SOC_TOUCH_SENSOR_VERSION == 2
  touchSetTiming(32, powerWaitUs);
  touchSetConfig(chargeCycles, TOUCH_VOLT_LIM_L_0V5, TOUCH_VOLT_LIM_H_2V2);
#endif

  radioBegin();
  if (!IS_CHILD) bleBegin();
  Serial.printf("{\"ev\":\"boot\",\"fw\":\"%s\",\"chip\":%u,\"pins\":%u,\"usbMidi\":%d,\"ble\":%d}\n", FW_VERSION,
                CHIP_ID, NUM_PINS, HAS_USB_MIDI, HAS_BLE && !IS_CHILD);
  calibrate();
  if (bootIntoUpdateMode()) enterUpdateMode();
}

// Common-mode rejection. A finger on a book raises ONE pin; noise coming in
// through the shared ground (e.g. touching the phone the board is plugged
// into) raises ALL pins together. Each scan we take the median rise across the
// board's 14 pins and subtract it before deciding touches, so a book only
// counts by how far it rises above the others. (It would take 8 books touched
// at once on one board to fool this.)

void scanPins(uint32_t nowUs, uint32_t nowMs) {
  uint32_t raws[NUM_PINS];
  bool freshRaw[NUM_PINS] = {};
  float rise[NUM_PINS];
  uint8_t m = 0;
  for (uint8_t i = 0; i < NUM_PINS; i++) {
    if (!scanned(i)) continue;
    Pin &p = pins[i];
    uint32_t r = readTouch(PINS[i]);
    // A new reading goes into the spike filter. S2/S3 measure in the background,
    // so an unchanged value = no new measurement yet; the classic ESP32 measures
    // on every call (and its coarse values often repeat).
    if (TOUCH_FALLS || r != p.lastRaw) {
      p.lastRaw = r;
      freshRaw[i] = true;
      p.hist[p.hn++ % MEDIAN_N] = r;
      if (p.hn >= 2 * MEDIAN_N) p.hn = MEDIAN_N;
    }
    raws[i] = p.hn >= MEDIAN_N ? median5(p.hist, MEDIAN_N) : r;
    rise[m++] = p.baseline > 0 ? (raws[i] - p.baseline) / p.baseline : 0;
  }
  for (uint8_t i = 1; i < m; i++) {  // insertion sort for the median
    float v = rise[i]; int8_t j = i - 1;
    while (j >= 0 && rise[j] > v) { rise[j + 1] = rise[j]; j--; }
    rise[j + 1] = v;
  }
  // Ground-noise estimate needs a few pins to compare; with fewer, skip it.
  cmFrac = m >= 4 ? max(0.0f, (rise[(m - 1) / 2] + rise[m / 2]) / 2) : 0;
  uint16_t cmBpNow = cmFrac * 10000 > 16383 ? 16383 : (uint16_t)(cmFrac * 10000);
  if (cmBpNow > cmPeakBp) cmPeakBp = cmBpNow;

  for (uint8_t i = 0; i < NUM_PINS; i++) {
    if (!scanned(i)) continue;
    Pin &p = pins[i];
    uint32_t raw = raws[i];
    float rawDelta = (float)raw - p.baseline;
    float delta = rawDelta - cmFrac * p.baseline;  // what the finger added, ground noise removed
    bool fresh = freshRaw[i];
    p.last = raw;
    if (fresh && i == 0) freshCount++;
    if (fresh) { uint16_t b = bp(delta, p.baseline); if (b > p.peakBp) p.peakBp = b; }
    if (fresh && p.baseline > 0) waveSample(i, delta / p.baseline, nowUs);

    if (!p.touched) {
      bool rearmed = !p.releasedAt || nowUs - p.releasedAt >= RETRIGGER_US;
      if (delta > p.onDelta && rearmed) {
        if (!p.aboveSince) p.aboveSince = nowUs | 1;
        if (nowUs - p.aboveSince >= ON_CONFIRM_US) {
          localTouch(i, true);
          p.touched = true; p.touchedAt = nowMs; p.belowSince = 0; p.peak = delta; p.flatSince = 0;
          Serial.printf("{\"ev\":\"on\",\"k\":%u}\n", i);
        }
      } else {
        p.aboveSince = 0;
        // Idle: follow slow wander. Not right after a release (the finger is
        // still leaving) and not while a hand hovers near the threshold.
        static uint32_t lastDriftUs[NUM_PINS];
        bool settled = !p.releasedAt || nowUs - p.releasedAt >= DRIFT_HOLD_US;
        if (fresh && settled && rawDelta < p.offDelta && cmFrac * p.baseline < p.offDelta) {
          float dt = min((nowUs - lastDriftUs[i]) / 1000.0f, 50.0f);
          float a = min(dt / (rawDelta < 0 ? DROP_TAU_MS : DRIFT_TAU_MS), 1.0f);
          p.baseline += (raw - p.baseline) * a;
          updateThresholds(p);
        }
        if (fresh) lastDriftUs[i] = nowUs;  // dt = time between sensor readings, not loop passes
      }
    } else {
      p.peak = max(p.peak, delta);
      // A finger always wobbles a little; a level that stays dead flat is a
      // shifted book/foil/wire, not a touch.
      float band = max(3 * p.noise, 0.15f * p.onDelta);
      if (fresh && (!p.flatSince || fabsf(delta - p.flatRef) > band)) { p.flatSince = nowMs; p.flatRef = delta; }
      bool frozen = p.flatSince && nowMs - p.flatSince > FROZEN_MS;
      bool stuck = nowMs - p.touchedAt > STUCK_MS || frozen;
      if (delta < p.offDelta || stuck) {
        if (!p.belowSince) p.belowSince = nowUs | 1;
        if (stuck || nowUs - p.belowSince >= OFF_CONFIRM_US) {
          localTouch(i, false);
          p.touched = false; p.aboveSince = 0; p.releasedAt = nowUs | 1;
          if (stuck) p.baseline = raw;  // environment changed under the pin
          Serial.printf("{\"ev\":\"off\",\"k\":%u,\"peakBp\":%u,\"ms\":%lu%s}\n", i, bp(p.peak, p.baseline),
                        (unsigned long)(nowMs - p.touchedAt), frozen ? ",\"frozen\":true" : stuck ? ",\"stuck\":true" : "");
          p.flatSince = 0;
        }
      } else {
        p.belowSince = 0;
      }
    }
  }
}

void stream(uint32_t nowMs) {
  Serial.printf("{\"t\":%lu,\"role\":\"%s\",\"hz\":%lu,\"sr\":%lu,\"chg\":%lu,\"wait\":%lu", (unsigned long)nowMs,
                IS_CHILD ? "child" : "parent", (unsigned long)scanHz, (unsigned long)sensorHz,
                (unsigned long)chargeCycles, (unsigned long)powerWaitUs);
  if (IS_CHILD) Serial.printf(",\"paired\":%d,\"fails\":%lu", paired, (unsigned long)sendFails);
  else Serial.printf(",\"child\":{\"on\":%d,\"rtt\":%lu,\"fails\":%lu}", childOnline, (unsigned long)rttUs,
                     (unsigned long)childFails);
  Serial.print(",\"k\":[");  // per sensor: [raw, baseline, touch level, release level, touched]
  for (uint8_t i = 0; i < NUM_PINS; i++) {
    const Pin &p = pins[i];
    Serial.printf("%s[%lu,%.0f,%.0f,%.0f,%d]", i ? "," : "", (unsigned long)p.last, p.baseline,
                  p.baseline + p.onDelta, p.baseline + p.offDelta, p.touched);
  }
  if (!IS_CHILD && childOnline) {
    for (uint8_t i = 0; i < childPins; i++) {
      const Remote &r = remote[i];
      float on = r.t.base * (r.t.thrBp / 10000.0f);
      Serial.printf(",[%lu,%lu,%.0f,%.0f,%d]", (unsigned long)r.t.raw, (unsigned long)r.t.base, r.t.base + on,
                    r.t.base + on * RELEASE_FRAC, r.touched);
    }
  }
  Serial.print("]}\n");
}

// Radio channel: the parent decides, saves it and restarts on it; the child
// finds it by itself (it searches every channel when it loses the parent).
void setRadioChannel(long v, Stream &out) {
  if (IS_CHILD || v < 1 || v > 13) return;
  prefs.begin("bookpiano", false); prefs.putUChar("ch", v); prefs.end();
  out.printf("{\"ev\":\"channel\",\"value\":%ld,\"restarting\":true}\n", v);
  out.flush(); delay(100);
  ESP.restart();
}

// `in` is the port the command came from: Serial (native USB) or Serial0
// (the COM/UART port), so either cable works.
void command(char c, Stream &in = Serial) {
  switch (c) {
    case 'W': {  // save Wi-Fi for update mode: "W\n<name>\n<password>\n" (sent by tools\set_wifi.py)
      in.setTimeout(3000);
      in.readStringUntil('\n');  // rest of the "W" line
      String s = in.readStringUntil('\n'), p = in.readStringUntil('\n');
      s.trim(); p.trim();
      if (!s.length()) { in.println("{\"ev\":\"wifi\",\"saved\":false}"); break; }
      prefs.begin("bookpiano", false); prefs.putString("wifi_s", s); prefs.putString("wifi_p", p); prefs.end();
      in.printf("{\"ev\":\"wifi\",\"saved\":true,\"name\":\"%s\"}\n", s.c_str());  // never echo the password
      break;
    }
    case 'r': tellChild('r'); calibrate(); break;
    case 'v': streaming = !streaming; break;
    case 'K': if (!IS_CHILD) setBooksFromSerial(in); break;
    case 'd': dumpPins(in); break;
    case 's': {  // link + scan status, on whichever port asked
      uint8_t touched = 0;
      for (uint8_t i = 0; i < NUM_PINS; i++) touched += pins[i].touched;
      in.printf("{\"ev\":\"status\",\"fw\":\"%s\",\"role\":\"%s\",\"paired\":%d,\"peer\":\"%02x:%02x:%02x:%02x:%02x:%02x\","
                "\"sinceHeardMs\":%lu,\"sendFails\":%lu,\"sensorHz\":%lu,\"touched\":%u,\"ch\":%u,\"thrHash\":%u",
                FW_VERSION, IS_CHILD ? "child" : "parent", paired, peerMac[0], peerMac[1], peerMac[2], peerMac[3],
                peerMac[4], peerMac[5], (unsigned long)(millis() - lastHeardMs), (unsigned long)sendFails,
                (unsigned long)sensorHz, touched, wifiCh, localThrHash());
      if (!IS_CHILD) {
        in.printf(",\"childOnline\":%d,\"rttUs\":%lu,\"childFails\":%lu,\"childInSync\":%d", childOnline,
                  (unsigned long)rttUs, (unsigned long)childFails, childCfgHash == childThrHash());
        printConfig(in);
      }
      in.print("}\n");
      break;
    }
    case 'u':
      if (!IS_CHILD && paired) {  // tell the child a few times: it must hear it before we leave the channel
        for (int n = 0; n < 3; n++) { tellChild('u'); delay(20); }
      }
      enterUpdateMode();
      break;
    case 'C': setRadioChannel(in.parseInt(), in); break;
    case 'c':    // charge cycles
    case 'w': {  // power-on wait (µs)
      long v = in.parseInt();
      bool ok = c == 'c' ? (v >= 20 && v <= 2000) : (v >= 1 && v <= 1000);
      if (ok) {
        prefs.begin("bookpiano", false); prefs.putUInt(c == 'c' ? "charge" : "wait", v); prefs.end();
        in.printf("{\"ev\":\"%s\",\"value\":%ld,\"restarting\":true}\n", c == 'c' ? "charge" : "wait", v);
        in.flush(); delay(50);
        ESP.restart();
      }
      break;
    }
  }
}

void loop() {
  if (updateMode) {  // no playing while waiting for new firmware
    updateLoop();
    return;
  }
  if (restartAtMs && (int32_t)(millis() - restartAtMs) > 0) {  // books moved to other pins
    releaseLocal();
    ESP.restart();
  }
  uint32_t nowUs = micros(), nowMs = millis();

  scanPins(nowUs, nowMs);
  background();

  scans++;
  if (nowMs - lastHzAt >= 1000) {
    scanHz = scans; scans = 0;
    sensorHz = freshCount; freshCount = 0;
    lastHzAt = nowMs;
  }

  if (streaming && nowMs - lastStreamAt >= STREAM_EVERY_MS) { lastStreamAt = nowMs; stream(nowMs); }

  while (Serial.available()) command(Serial.read(), Serial);
#if HAS_SERIAL0
  while (Serial0.available()) command(Serial0.read(), Serial0);
#endif

  static uint32_t lastMidiPollMs = 0;
  if (!IS_CHILD && nowMs != lastMidiPollMs) {  // once per ms is plenty for commands
    lastMidiPollMs = nowMs;
    midiEventPacket_t p;
    while (MIDI.readPacket(&p)) {
      if (sysexPacket(p)) continue;                              // configuration + calibration
      if ((p.byte1 & 0xF0) != 0xB0 || p.byte3 == 0) continue;  // Control Change, value > 0
      switch (p.byte2) {
        case 119: command('r'); outCC(116, 1); break;
        case 115: outCC(116, 4); command('u'); break;
      }
    }
    configLoop();  // telemetry frames for the setup wizard
    bleLoop();     // Bluetooth notes/settings from the phone
  }
}
