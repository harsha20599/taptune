// Book Piano · Bluetooth link (parent only)
//
// The parent is a standard Bluetooth LE MIDI device called "Book Piano", so
// TapTune connects to it straight from Chrome (Web Bluetooth) with no cable —
// the phone isn't electrically tied to the books — and Macs / music apps can
// use it as a Bluetooth MIDI keyboard too. USB MIDI keeps working alongside.
//
//   BLE-MIDI service 03b80e5a-…  characteristic 7772e5db-…: notes out, Control
//     Change in (119 re-baseline, 115 update mode). One message per packet,
//     sent the moment a book is touched.
//   Settings service b0c40001-…: the same SysEx protocol as USB (config.ino),
//     without the F0/F7 wrapper, split into chunks:
//       chunk = [flags, bytes…], flags 1 = first chunk, 2 = last chunk.
//     CFG_TX (notify) board → page, CFG_RX (write) page → board.
//
// Latency: after connecting we ask for a 7.5 ms connection interval (the
// shortest Bluetooth allows); phones grant 7.5–15 ms.
//
// Chips without Bluetooth LE (ESP32-S2) get the stubs at the end: USB only.
// The core uses NimBLE on S3 and Bluedroid on the classic ESP32; the few
// calls that differ are split below.

#if HAS_BLE
#include <BLEDevice.h>
#include <BLEServer.h>

#define BLE_MIDI_SVC "03b80e5a-ede8-4b33-a751-6ce34ec4c700"
#define BLE_MIDI_CHR "7772e5db-3868-4112-a1a9-f2669d106bf3"
#define BLE_CFG_SVC  "b0c40001-7d42-4b50-9a1e-426f6f6b5069"
#define BLE_CFG_TX   "b0c40002-7d42-4b50-9a1e-426f6f6b5069"
#define BLE_CFG_RX   "b0c40003-7d42-4b50-9a1e-426f6f6b5069"

BLEServer *bleServer = nullptr;
BLECharacteristic *bleMidiChr = nullptr, *bleCfgTx = nullptr, *bleCfgRx = nullptr;
volatile bool bleConnected = false;
volatile uint16_t bleMtu = 23;
volatile uint16_t bleConnHandle = 0xFFFF;
volatile uint32_t bleConnectedAtMs = 0;
volatile uint16_t bleItvl = 0;  // Bluedroid: last interval the phone set (NimBLE asks the stack)
#if CONFIG_BLUEDROID_ENABLED
esp_bd_addr_t blePeer;
#endif

// Writes arrive in the Bluetooth task; they're queued and handled in loop().
struct BleRx { uint8_t kind; uint8_t len; uint8_t data[244]; };  // kind 0 = MIDI, 1 = settings chunk
QueueHandle_t bleRxQueue;

void bleAskFastInterval() {
#if CONFIG_NIMBLE_ENABLED
  bleServer->updateConnParams(bleConnHandle, 6, 12, 0, 300);  // 7.5–15 ms interval, 3 s timeout
#else
  bleServer->updateConnParams(blePeer, 6, 12, 0, 300);
#endif
}

class BookServerCallbacks : public BLEServerCallbacks {
#if CONFIG_NIMBLE_ENABLED
  void onConnect(BLEServer *s, ble_gap_conn_desc *desc) override {
    bleConnected = true;
    bleConnHandle = desc->conn_handle;
    bleConnectedAtMs = millis();
    bleAskFastInterval();
    BLEDevice::startAdvertising();  // stay findable (only one phone is expected, but harmless)
  }
  void onDisconnect(BLEServer *s, ble_gap_conn_desc *desc) override {
    bleConnected = s->getConnectedCount() > 1;  // this callback runs before the count drops
    bleMtu = 23;
    BLEDevice::startAdvertising();
  }
  void onMtuChanged(BLEServer *s, ble_gap_conn_desc *desc, uint16_t mtu) override { bleMtu = mtu; }
#else
  void onConnect(BLEServer *s, esp_ble_gatts_cb_param_t *param) override {
    bleConnected = true;
    bleConnHandle = param->connect.conn_id;
    memcpy(blePeer, param->connect.remote_bda, sizeof(blePeer));
    bleConnectedAtMs = millis();
    bleItvl = 0;
    bleAskFastInterval();
    BLEDevice::startAdvertising();
  }
  void onDisconnect(BLEServer *s, esp_ble_gatts_cb_param_t *param) override {
    bleConnected = s->getConnectedCount() > 1;  // this callback runs before the count drops
    bleMtu = 23;
    BLEDevice::startAdvertising();
  }
  void onMtuChanged(BLEServer *s, esp_ble_gatts_cb_param_t *param) override { bleMtu = param->mtu.mtu; }
  void onConnParamsUpdate(esp_bd_addr_t, uint16_t interval, uint16_t, uint16_t, esp_bt_status_t status) override {
    if (status == ESP_BT_STATUS_SUCCESS) bleItvl = interval;
  }
#endif
};

class BookWriteCallbacks : public BLECharacteristicCallbacks {
  uint8_t kind;
 public:
  explicit BookWriteCallbacks(uint8_t k) : kind(k) {}
#if CONFIG_NIMBLE_ENABLED
  void onWrite(BLECharacteristic *c, ble_gap_conn_desc *) override { queue(c); }
#else
  void onWrite(BLECharacteristic *c, esp_ble_gatts_cb_param_t *) override { queue(c); }
#endif
  void queue(BLECharacteristic *c) {
    String v = c->getValue();
    BleRx r;
    r.kind = kind;
    r.len = min((size_t)v.length(), sizeof(r.data));
    memcpy(r.data, v.c_str(), r.len);
    xQueueSend(bleRxQueue, &r, 0);
  }
};

void bleBegin() {
  bleRxQueue = xQueueCreate(32, sizeof(BleRx));
  BLEDevice::init("Book Piano");
  BLEDevice::setMTU(247);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new BookServerCallbacks());

  BLEService *midi = bleServer->createService(BLE_MIDI_SVC);
  bleMidiChr = midi->createCharacteristic(BLE_MIDI_CHR, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE_NR |
                                                        BLECharacteristic::PROPERTY_NOTIFY);
  bleMidiChr->setCallbacks(new BookWriteCallbacks(0));
  midi->start();

  BLEService *cfgSvc = bleServer->createService(BLE_CFG_SVC);
  bleCfgTx = cfgSvc->createCharacteristic(BLE_CFG_TX, BLECharacteristic::PROPERTY_NOTIFY);
  bleCfgRx = cfgSvc->createCharacteristic(BLE_CFG_RX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  bleCfgRx->setCallbacks(new BookWriteCallbacks(1));
  cfgSvc->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_MIDI_SVC);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("{\"ev\":\"ble\",\"state\":\"advertising\",\"name\":\"Book Piano\"}");
}

// ---- out: MIDI (BLE-MIDI packet: header, timestamp, status, data) ----
void bleMidi(uint8_t status, uint8_t d1, uint8_t d2) {
  if (!bleConnected) return;
  uint16_t ts = millis() & 0x1FFF;
  uint8_t pkt[5] = { (uint8_t)(0x80 | (ts >> 7)), (uint8_t)(0x80 | (ts & 0x7F)), status, d1, d2 };
  bleMidiChr->setValue(pkt, 5);
  bleMidiChr->notify();
}

// Every note / CC goes out on USB and Bluetooth at once.
void outNoteOn(uint8_t note, uint8_t vel) { bleMidi(0x90 | (MIDI_CHANNEL - 1), note, vel); MIDI.noteOn(note, vel, MIDI_CHANNEL); }
void outNoteOff(uint8_t note) { bleMidi(0x80 | (MIDI_CHANNEL - 1), note, 0); MIDI.noteOff(note, 0, MIDI_CHANNEL); }
void outCC(uint8_t cc, uint8_t val) { bleMidi(0xB0 | (MIDI_CHANNEL - 1), cc, val); MIDI.controlChange(cc, val, MIDI_CHANNEL); }

// ---- out: settings (SysEx body without F0/F7), in chunks that fit the link ----
void bleSendSettings(const uint8_t *d, size_t n) {
  if (!bleConnected) return;
  size_t room = constrain((int)bleMtu - 3, 20, 244) - 1;
  uint8_t chunk[245];
  for (size_t i = 0; i < n || i == 0; i += room) {
    size_t len = min(room, n - i);
    chunk[0] = (i == 0 ? 1 : 0) | (i + len >= n ? 2 : 0);
    memcpy(chunk + 1, d + i, len);
    bleCfgTx->setValue(chunk, len + 1);
    bleCfgTx->notify();
    if (n == 0) break;
  }
}

// The phone decides the connection interval (the time slot notes wait for).
// Units of 1.25 ms; 0 = not connected.
uint16_t bleIntervalUnits() {
  if (!bleConnected || bleConnHandle == 0xFFFF) return 0;
#if CONFIG_NIMBLE_ENABLED
  struct ble_gap_conn_desc d;
  return ble_gap_conn_find(bleConnHandle, &d) == 0 ? d.conn_itvl : 0;
#else
  return bleItvl;
#endif
}

// ---- in: handled in loop() ----
void bleLoop() {
  // Phones often answer the first request with a slow interval; ask again a
  // couple of times (at 3 s and 8 s after connecting) if it's over 15 ms.
  static uint8_t asked = 0;
  static uint16_t lastHandle = 0xFFFF;
  if (bleConnHandle != lastHandle) { lastHandle = bleConnHandle; asked = 0; }
  if (bleConnected && asked < 2 && millis() - bleConnectedAtMs > (asked ? 8000u : 3000u)) {
    asked++;
    uint16_t itvl = bleIntervalUnits();
    if (itvl == 0 || itvl > 12) bleAskFastInterval();
  }

  static uint8_t buf[200];
  static size_t len = 0;
  BleRx r;
  while (xQueueReceive(bleRxQueue, &r, 0) == pdTRUE) {
    if (r.kind == 1) {                 // settings chunk
      if (!r.len) continue;
      uint8_t flags = r.data[0];
      if (flags & 1) { len = 0; buf[len++] = 0xF0; }
      for (uint8_t i = 1; i < r.len && len < sizeof(buf) - 1; i++) buf[len++] = r.data[i];
      if (flags & 2) { buf[len++] = 0xF7; handleSysex(buf, len); len = 0; }
    } else {                           // BLE-MIDI: [header, (timestamp, status, data…)…]
      uint8_t status = 0;
      for (uint8_t i = 1; i < r.len;) {
        if (r.data[i] & 0x80) {        // timestamp byte, maybe followed by a status byte
          i++;
          if (i < r.len && (r.data[i] & 0x80)) status = r.data[i++];
        }
        if ((status & 0xF0) == 0xB0 && i + 1 < r.len) {
          uint8_t cc = r.data[i], val = r.data[i + 1];
          i += 2;
          if (val == 0) continue;
          if (cc == 119) { command('r'); outCC(116, 1); }
          else if (cc == 115) { outCC(116, 4); command('u'); }
        } else {
          i++;                         // notes or anything else from the phone: ignore
        }
      }
    }
  }
}

#else  // no Bluetooth LE on this chip: notes and settings go over USB only
volatile bool bleConnected = false;
void bleBegin() {}
void bleLoop() {}
void bleSendSettings(const uint8_t *, size_t) {}
uint16_t bleIntervalUnits() { return 0; }
void outNoteOn(uint8_t note, uint8_t vel) { MIDI.noteOn(note, vel, MIDI_CHANNEL); }
void outNoteOff(uint8_t note) { MIDI.noteOff(note, 0, MIDI_CHANNEL); }
void outCC(uint8_t cc, uint8_t val) { MIDI.controlChange(cc, val, MIDI_CHANNEL); }
#endif
