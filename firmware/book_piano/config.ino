// Book Piano · configuration (parent only)
//
// SENSOR = board + slot: 0–13 main board, 14–27 second board. Slot i is the
//          board's i-th touch pin (PINS[] in compat.h; GPIO i+1 on S2/S3).
// BOOK   = a position on the shelf, 1..numBooks left to right. Each book points
//          at one sensor and has a note. Transpose shifts every note.
// Each sensor has a touch threshold in basis points (bp = 0.01 % of its resting
// level). The parent stores everything ("cfg2") and keeps the child's 14
// thresholds in sync (it compares a checksum the child reports and resends).
//
// TapTune talks to it over MIDI System Exclusive, F0 7D 42 <cmd> … F7
// (0x7D = non-commercial ID, 0x42 = Book Piano). 14-bit values go as two 7-bit
// bytes, low first.
//
//   host → board  10                 get config          → 11
//                 12 <config>        set + save config   → 11
//                 13                 reset to defaults   → 11
//                 20 <0|1>           telemetry off/on (auto-off 10 s after the last 20 01)
//                 22                 re-baseline both boards (hands off) → 23 when done
//                 24                 board info          → 25
//                 26 <1-13>          radio channel: saved, board restarts on it
//                 2A <0|1>           listen on ALL pins (for finding books) on/off;
//                                    boards restart; saving a config (12) turns it off
//   board → host  11 <config>        ver, books, transpose+64, velocity,
//                                    28 × book sensor (127 = none), 28 × book note,
//                                    28 × sensor threshold (14-bit bp)
//                 21 n <n × 9 bytes> telemetry, 20 per second, per sensor:
//                                    flags (1 online, 2 touched, 4 used by a book),
//                                    rise now (14-bit, +8192 offset), peak rise since
//                                    last frame (ground noise removed), idle noise,
//                                    threshold in use (all bp); then 2 × 14-bit: main
//                                    and second board ground-noise (common-mode) peak
//                 31 s               sensor s was just touched (telemetry on)
//                 25 …               fw major, fw minor, second board online,
//                                    in sync, radio channel, main board's pin count,
//                                    BLE interval (14-bit), then (fw 3+): main chip,
//                                    second chip, 14 main GPIOs, 14 second GPIOs
//                                    (0 = no such pin; chip 1 ESP32, 2 S2, 3 S3)

const uint8_t MAX_BOOKS = MAX_SENSORS;
const uint8_t NO_SENSOR = 127;
const uint8_t CFG_VERSION = 2;
const uint8_t SYX_ID = 0x7D, SYX_DEV = 0x42;
const uint32_t TELEMETRY_EVERY_MS = 50;
const uint32_t TELEMETRY_KEEPALIVE_MS = 10000;

struct Config {
  uint8_t version, numBooks;
  int8_t transpose;
  uint8_t velocity;
  uint8_t bookSensor[MAX_BOOKS];
  uint8_t bookNote[MAX_BOOKS];
  uint16_t thrBp[MAX_SENSORS];
};
Config cfg;
uint8_t sensorBook[MAX_SENSORS];    // reverse lookup, 255 = not used by any book
uint8_t soundingNote[MAX_SENSORS];  // note playing for each sensor, 255 = none
bool telemetryOn = false;
uint32_t telemetryUntilMs = 0;

// Wave: one sensor's rise (ground noise removed) at up to 200 samples/s, for
// the live graph in TapTune. Main-board pins sample every 5 ms; second-board
// pins at its state rate (50/s).
uint8_t waveSensor = 255;   // 255 = off
uint32_t waveUntilMs = 0;
int16_t waveBuf[48];
uint8_t waveN = 0;
uint32_t lastWaveUs = 0;
void waveSample(uint8_t s, float riseFrac, uint32_t nowUs) {
  if (s != waveSensor || waveN >= 48) return;
  if (s < SLOTS && nowUs - lastWaveUs < 5000) return;
  lastWaveUs = nowUs;
  waveBuf[waveN++] = constrain((int)(riseFrac * 10000), -8191, 8191);
}

void defaultConfig() {
  cfg = Config{};
  cfg.version = CFG_VERSION;
  uint8_t mainBooks = min<uint8_t>(12, NUM_PINS);
  cfg.numBooks = mainBooks + 12;    // 12 per board (S2/S3: GPIO1–12), fewer on a 9-pin main board
  cfg.transpose = 0;
  cfg.velocity = 100;
  for (uint8_t k = 0; k < MAX_BOOKS; k++) {
    cfg.bookSensor[k] = k < mainBooks ? k : k < cfg.numBooks ? SLOTS + (k - mainBooks) : NO_SENSOR;
    cfg.bookNote[k] = 60 + k;         // chromatic from C4
  }
  for (uint8_t s = 0; s < MAX_SENSORS; s++) cfg.thrBp[s] = DEFAULT_THR_BP;
}

void rebuildIndex() {
  memset(sensorBook, 255, sizeof(sensorBook));
  for (uint8_t k = 0; k < cfg.numBooks; k++) {
    uint8_t s = cfg.bookSensor[k];
    if (s < MAX_SENSORS && sensorBook[s] == 255) sensorBook[s] = k;
  }
}

uint16_t usedMask(bool child) {  // which pins of a board have books
  uint16_t m = 0;
  for (uint8_t k = 0; k < cfg.numBooks; k++) {
    uint8_t s = cfg.bookSensor[k];
    if (s < MAX_SENSORS && (s >= SLOTS) == child) m |= 1 << (s % SLOTS);
  }
  return m;
}

// "Listen on all pins": while finding books every pin must be scanned, not
// just the ones that already have books. Saved, so it survives the restart.
bool scanAll = false;
// (The child keeps only the slots it really has: masked with its pin count.)
uint16_t childScanUsed() {
  uint16_t m = scanAll ? 0x3FFF : usedMask(true);
  return childPins ? m & ((1 << childPins) - 1) : m;
}
uint16_t localScanMask() { return scanAll ? ALL_PINS : maskFor(usedMask(false)); }

uint16_t childThrHash() { return thrHash(&cfg.thrBp[SLOTS]) ^ childScanUsed(); }

void sendChildConfig() {
  if (!paired) return;
  MsgCfg m;
  memcpy(m.thrBp, &cfg.thrBp[SLOTS], sizeof(m.thrBp));
  m.used = childScanUsed();
  send(peerMac, m, M_CFG);
}

// Notes held under the old map still get their own note-off (soundingNote), so
// applying a new config while playing never leaves a note stuck.
void applyConfig() {
  rebuildIndex();
  setPinThresholds(cfg.thrBp);  // main board's 14
  sendChildConfig();            // second board's 14 (+ which of its pins to scan)
  scheduleMaskRestart(localScanMask());  // books moved: rescan the right pins
}

void setScanAll(bool on) {
  if (scanAll == on) return;
  scanAll = on;
  prefs.begin("bookpiano", false); prefs.putBool("scanall", on); prefs.end();
}

void saveConfig() {
  prefs.begin("bookpiano", false);
  prefs.putBytes("cfg2", &cfg, sizeof(cfg));
  prefs.end();
}

void loadConfig() {
  defaultConfig();
  memset(soundingNote, 255, sizeof(soundingNote));
  prefs.begin("bookpiano", true);
  if (prefs.getBytesLength("cfg2") == sizeof(cfg)) {
    Config c;
    prefs.getBytes("cfg2", &c, sizeof(c));
    if (c.version == CFG_VERSION && c.numBooks <= MAX_BOOKS) cfg = c;
  } else if (prefs.getBytesLength("notes") == 24) {  // older firmware's note map: keep the notes
    prefs.getBytes("notes", cfg.bookNote, 24);
  }
  scanAll = prefs.getBool("scanall", false);
  prefs.end();
  scanMask = localScanMask();  // at boot, before any pin is set up
  applyConfig();
}

// ---- touch → note ----
void sendTouchEvent(uint8_t s);
void sensorTouch(uint8_t s, bool on) {
  if (on) {
    if (telemetryOn) sendTouchEvent(s);
    uint8_t k = sensorBook[s];
    if (k == 255) return;  // wired, but not one of the books
    uint8_t note = constrain((int)cfg.bookNote[k] + cfg.transpose, 0, 127);
    if (soundingNote[s] != 255) outNoteOff(soundingNote[s]);
    outNoteOn(note, cfg.velocity);
    soundingNote[s] = note;
    if (s >= SLOTS) Serial.printf("{\"ev\":\"on\",\"sensor\":%u,\"book\":%u,\"note\":%u}\n", s, k + 1, note);
  } else if (soundingNote[s] != 255) {
    outNoteOff(soundingNote[s]);
    soundingNote[s] = 255;
  }
}

// ---- SysEx out: split into USB-MIDI packets (3 bytes each) ----
void sendSysex(const uint8_t *d, size_t n) {
  if (n >= 2) bleSendSettings(d + 1, n - 2);  // Bluetooth gets the same message, without F0/F7
  size_t i = 0;
  while (n - i > 3) {
    midiEventPacket_t p = { 0x04, d[i], d[i + 1], d[i + 2] };  // CIN 4: SysEx starts/continues
    MIDI.writePacket(&p);
    i += 3;
  }
  size_t rest = n - i;  // 1–3 bytes, the last ones end with F7
  midiEventPacket_t p = { (uint8_t)(0x04 + rest), d[i], rest > 1 ? d[i + 1] : (uint8_t)0, rest > 2 ? d[i + 2] : (uint8_t)0 };
  MIDI.writePacket(&p);  // CIN 5/6/7: SysEx ends with 1/2/3 bytes
}

inline void put14(uint8_t *&w, uint16_t v) { *w++ = v & 0x7F; *w++ = (v >> 7) & 0x7F; }
inline uint16_t get14(const uint8_t *r) { return r[0] | (r[1] << 7); }

const size_t CFG_BYTES = 4 + MAX_BOOKS + MAX_BOOKS + 2 * MAX_SENSORS;  // payload after the command byte

void sendConfigSysex() {
  uint8_t msg[4 + CFG_BYTES + 1], *w = msg;
  *w++ = 0xF0; *w++ = SYX_ID; *w++ = SYX_DEV; *w++ = 0x11;
  *w++ = cfg.version; *w++ = cfg.numBooks; *w++ = cfg.transpose + 64; *w++ = cfg.velocity;
  for (uint8_t k = 0; k < MAX_BOOKS; k++) *w++ = cfg.bookSensor[k] & 0x7F;
  for (uint8_t k = 0; k < MAX_BOOKS; k++) *w++ = cfg.bookNote[k] & 0x7F;
  for (uint8_t s = 0; s < MAX_SENSORS; s++) put14(w, cfg.thrBp[s]);
  *w++ = 0xF7;
  sendSysex(msg, w - msg);
}

Config parsed;  // parseConfig() fills this (Arduino's generated prototypes can't name Config)
bool parseConfig(const uint8_t *r, size_t n) {  // r = payload after the command byte
  if (n < CFG_BYTES) return false;
  Config &c = parsed;
  c = Config{};
  c.version = CFG_VERSION;
  c.numBooks = r[1];
  c.transpose = (int)r[2] - 64;
  c.velocity = r[3];
  if (c.numBooks > MAX_BOOKS || c.velocity < 1 || c.transpose < -48 || c.transpose > 48) return false;
  r += 4;
  for (uint8_t k = 0; k < MAX_BOOKS; k++) {
    c.bookSensor[k] = r[k] < MAX_SENSORS ? r[k] : NO_SENSOR;
    c.bookNote[k] = r[MAX_BOOKS + k] & 0x7F;
  }
  r += 2 * MAX_BOOKS;
  for (uint8_t s = 0; s < MAX_SENSORS; s++) c.thrBp[s] = constrain(get14(r + 2 * s), 20, 16000);
  return true;
}

void sendTouchEvent(uint8_t s) {
  uint8_t msg[] = { 0xF0, SYX_ID, SYX_DEV, 0x31, s, 0xF7 };
  sendSysex(msg, sizeof(msg));
}

uint16_t bleIntervalUnits();
void sendInfo() {
  uint16_t itvl = bleIntervalUnits();  // Bluetooth time slot the phone granted, ×1.25 ms
  uint8_t msg[15 + 2 * SLOTS], *w = msg;
  *w++ = 0xF0; *w++ = SYX_ID; *w++ = SYX_DEV; *w++ = 0x25;
  *w++ = FW_MAJOR; *w++ = FW_MINOR; *w++ = childOnline; *w++ = childOnline && childCfgHash == childThrHash();
  *w++ = wifiCh; *w++ = NUM_PINS; put14(w, itvl);
  *w++ = CHIP_ID; *w++ = childOnline ? childChip : 0;
  for (uint8_t i = 0; i < SLOTS; i++) *w++ = i < NUM_PINS ? PINS[i] : 0;
  for (uint8_t i = 0; i < SLOTS; i++) *w++ = childOnline && i < childPins ? childGpio[i] & 0x7F : 0;
  *w++ = 0xF7;
  sendSysex(msg, w - msg);
}

void handleSysex(const uint8_t *d, size_t n) {
  if (n < 5 || d[1] != SYX_ID || d[2] != SYX_DEV) return;
  const uint8_t *r = d + 4;
  size_t rn = n - 5;  // payload without header and F7
  switch (d[3]) {
    case 0x10: sendConfigSysex(); break;
    case 0x12: {
      if (!parseConfig(r, rn)) { Serial.println("{\"ev\":\"config\",\"error\":\"bad\"}"); break; }
      cfg = parsed;
      setScanAll(false);  // books are set: back to listening only on them
      applyConfig();
      saveConfig();
      Serial.printf("{\"ev\":\"config\",\"saved\":true,\"books\":%u}\n", cfg.numBooks);
      sendConfigSysex();
      break;
    }
    case 0x13: defaultConfig(); applyConfig(); saveConfig(); sendConfigSysex(); break;
    case 0x20:
      telemetryOn = rn > 0 && r[0];
      telemetryUntilMs = millis() + TELEMETRY_KEEPALIVE_MS;
      break;
    case 0x22: {
      command('r');  // both boards, ~1.2 s
      uint8_t msg[] = { 0xF0, SYX_ID, SYX_DEV, 0x23, 0xF7 };
      sendSysex(msg, sizeof(msg));
      break;
    }
    case 0x24: sendInfo(); break;
    case 0x28:  // wave on/off for one sensor (auto-off 10 s after the last request)
      if (rn >= 2 && r[0] < MAX_SENSORS && r[1]) { if (waveSensor != r[0]) waveN = 0; waveSensor = r[0]; waveUntilMs = millis() + TELEMETRY_KEEPALIVE_MS; }
      else waveSensor = 255;
      break;
    case 0x2A:
      setScanAll(rn > 0 && r[0]);
      applyConfig();  // restarts both boards if what they scan changed
      break;
    case 0x26:  // radio channel: saved, then restart on it; the second board finds it by itself
      if (rn > 0) setRadioChannel(r[0], Serial);
      break;
  }
}

// ---- SysEx in: reassemble from USB-MIDI packets. Returns true if the packet was SysEx. ----
bool sysexPacket(const midiEventPacket_t &p) {
  static uint8_t buf[192];
  static size_t len = 0;
  uint8_t cin = p.header & 0x0F, bytes[3] = { p.byte1, p.byte2, p.byte3 };
  uint8_t n;
  if (cin == 0x04) n = 3;
  else if (cin >= 0x05 && cin <= 0x07) n = cin - 0x04;
  else return false;
  for (uint8_t i = 0; i < n; i++) {
    if (bytes[i] == 0xF0) len = 0;
    if (len < sizeof(buf)) buf[len++] = bytes[i];
  }
  if (cin != 0x04) {  // message complete
    if (len && buf[0] == 0xF0) handleSysex(buf, len);
    len = 0;
  }
  return true;
}

// ---- telemetry for the setup wizard ----
uint16_t riseCode(float raw, float base) {  // signed rise in bp, offset by 8192
  if (base <= 0) return 8192;
  float v = (raw - base) / base * 10000.0f;
  return (uint16_t)(constrain((int)v, -8191, 8191) + 8192);
}

void sendTelemetry() {
  uint8_t msg[6 + 9 * MAX_SENSORS + 4], *w = msg;
  *w++ = 0xF0; *w++ = SYX_ID; *w++ = SYX_DEV; *w++ = 0x21; *w++ = MAX_SENSORS;
  for (uint8_t s = 0; s < MAX_SENSORS; s++) {
    bool used = sensorBook[s] != 255;
    if (s < SLOTS) {
      bool real = s < NUM_PINS;
      Pin &p = pins[real ? s : 0];
      if (!real) { *w++ = used ? 4 : 0; put14(w, 8192); put14(w, 0); put14(w, 0); put14(w, 0); continue; }
      *w++ = (scanned(s) ? 1 : 0) | (p.touched ? 2 : 0) | (used ? 4 : 0);
      put14(w, riseCode(p.last, p.baseline));
      put14(w, p.peakBp);
      put14(w, bp(p.noise, p.baseline));
      put14(w, bp(p.onDelta, p.baseline));
      p.peakBp = 0;
    } else {
      Remote &r = remote[s - SLOTS];
      bool online = childOnline && s - SLOTS < childPins;
      *w++ = (online ? 1 : 0) | (r.touched ? 2 : 0) | (used ? 4 : 0);
      put14(w, online ? riseCode(r.t.raw, r.t.base) : 8192);
      put14(w, online ? r.peakBp : 0);
      put14(w, online ? r.t.noiseBp : 0);
      put14(w, online ? r.t.thrBp : 0);
      r.peakBp = 0;
    }
  }
  put14(w, cmPeakBp);                     // ground noise seen by each board since the last frame
  put14(w, childOnline ? childCmBp : 0);
  cmPeakBp = 0; childCmBp = 0;
  *w++ = 0xF7;
  sendSysex(msg, w - msg);
}

void sendWave() {
  uint8_t msg[9 + 2 * 48], *w = msg;
  uint8_t s = waveSensor;
  if (s < SLOTS && s >= NUM_PINS) { waveN = 0; return; }  // no such pin on this board
  bool touched = s < SLOTS ? pins[s].touched : remote[s - SLOTS].touched;
  uint16_t thr = s < SLOTS ? bp(pins[s].onDelta, pins[s].baseline) : remote[s - SLOTS].t.thrBp;
  *w++ = 0xF0; *w++ = SYX_ID; *w++ = SYX_DEV; *w++ = 0x29;
  *w++ = s; *w++ = touched; put14(w, thr); *w++ = waveN;
  for (uint8_t i = 0; i < waveN; i++) put14(w, waveBuf[i] + 8192);
  *w++ = 0xF7;
  waveN = 0;
  sendSysex(msg, w - msg);
}

void configLoop() {
  static uint32_t lastFrameMs = 0, lastWaveMs = 0;
  uint32_t now = millis();
  if (waveSensor != 255 && (int32_t)(now - waveUntilMs) > 0) waveSensor = 255;
  if (waveSensor != 255 && waveN && (now - lastWaveMs >= 50 || waveN >= 40)) { lastWaveMs = now; sendWave(); }
  if (telemetryOn && (int32_t)(now - telemetryUntilMs) > 0) telemetryOn = false;  // page went away
  // over Bluetooth, half the rate so live data never crowds out notes
  if (telemetryOn && now - lastFrameMs >= (bleConnected ? 2 * TELEMETRY_EVERY_MS : TELEMETRY_EVERY_MS)) { lastFrameMs = now; sendTelemetry(); }
}

// Serial "K": set the books from a computer, e.g. "K 2 0:60 1:62" = 2 books,
// book 1 on sensor 0 (main board's first pin) playing note 60, book 2 on sensor 1 playing 62.
// Thresholds, transpose and velocity stay as they are.
void setBooksFromSerial(Stream &in) {
  in.setTimeout(1000);
  String line = in.readStringUntil('\n');
  line.trim();
  int n = line.toInt();
  if (n < 0 || n > MAX_BOOKS) { in.println("{\"ev\":\"books\",\"error\":\"count\"}"); return; }
  Config c = cfg;
  c.numBooks = n;
  for (uint8_t k = 0; k < MAX_BOOKS; k++) c.bookSensor[k] = NO_SENSOR;
  int pos = line.indexOf(' ');
  for (int k = 0; k < n; k++) {
    if (pos < 0) { in.println("{\"ev\":\"books\",\"error\":\"missing book\"}"); return; }
    int colon = line.indexOf(':', pos);
    int next = line.indexOf(' ', pos + 1);
    if (colon < 0) { in.println("{\"ev\":\"books\",\"error\":\"format\"}"); return; }
    int s = line.substring(pos + 1, colon).toInt();
    int note = line.substring(colon + 1, next < 0 ? line.length() : next).toInt();
    if (s < 0 || s >= MAX_SENSORS || note < 0 || note > 127) { in.println("{\"ev\":\"books\",\"error\":\"value\"}"); return; }
    c.bookSensor[k] = s;
    c.bookNote[k] = note;
    pos = next;
  }
  cfg = c;
  applyConfig();
  saveConfig();
  in.printf("{\"ev\":\"books\",\"saved\":%u}\n", cfg.numBooks);
}

// Serial "d": one line per pin of this board, plus the second board's pins on the parent.
void dumpPins(Stream &out) {
  out.printf("{\"ev\":\"pins\",\"cmBp\":%u,\"main\":[", (uint16_t)(cmFrac * 10000));
  for (uint8_t i = 0; i < NUM_PINS; i++) {
    const Pin &p = pins[i];
    out.printf("%s{\"pin\":%u,\"raw\":%lu,\"base\":%.0f,\"riseBp\":%d,\"noiseBp\":%u,\"thrBp\":%u,\"touch\":%d,\"book\":%d}",
               i ? "," : "", PINS[i], (unsigned long)p.last, p.baseline,
               p.baseline > 0 ? (int)(((float)p.last - p.baseline) / p.baseline * 10000) : 0,
               bp(p.noise, p.baseline), bp(p.onDelta, p.baseline), p.touched,
               IS_CHILD ? -1 : (sensorBook[i] == 255 ? 0 : sensorBook[i] + 1));
  }
  out.print("]}\n");
}

void printConfig(Stream &out) {
  out.printf(",\"books\":%u,\"transpose\":%d,\"velocity\":%u,\"map\":[", cfg.numBooks, cfg.transpose, cfg.velocity);
  for (uint8_t k = 0; k < cfg.numBooks; k++) out.printf("%s[%u,%u]", k ? "," : "", cfg.bookSensor[k], cfg.bookNote[k]);
  out.print("],\"thrBp\":[");
  for (uint8_t s = 0; s < MAX_SENSORS; s++) out.printf("%s%u", s ? "," : "", cfg.thrBp[s]);
  out.print("]");
}
