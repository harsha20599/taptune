// Book Piano · chip compatibility
//
// Works on ESP32 chips that have capacitive touch pins:
//   ESP32-S3 (e.g. YD-ESP32-S3 / DevKitC-1 N16R8, S3 SuperMini)  14 touch pins, USB MIDI + Bluetooth
//   ESP32-S2                                                      14 touch pins, USB MIDI (no Bluetooth)
//   ESP32 (classic, e.g. 30/38-pin DevKit "ESP-WROOM-32")         9 touch pins, Bluetooth (no USB MIDI)
// Not possible: ESP32-C3 / C6 / H2 (no touch sensor).
//
// Every board talks in 14 "slots" (sensor ids: main 0–13, second 14–27), so
// TapTune and the two boards can mix chips; a board just has fewer real pins
// if its chip has fewer touch pins, and reports their GPIO numbers.
#pragma once
#include <Arduino.h>
#include "soc/soc_caps.h"

#if !SOC_TOUCH_SENSOR_SUPPORTED
#error "Book Piano needs a chip with capacitive touch pins: ESP32, ESP32-S2 or ESP32-S3 (not C3/C6/H2)."
#endif

// ---- touch pins (GPIO numbers) for this chip ----
#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
// GPIO1–14 are all touch pins (T1–T14). Books usually use 1–12; 13–14 are spares.
static const uint8_t PINS[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14 };
#define CHIP_ID 3  // reported to TapTune: 1 = ESP32, 2 = S2, 3 = S3
#if CONFIG_IDF_TARGET_ESP32S2
#undef CHIP_ID
#define CHIP_ID 2
#endif
#elif CONFIG_IDF_TARGET_ESP32
// Touch pins broken out on common 30/38-pin DevKit boards, safest first:
// T0 GPIO4, T4 13, T6 14, T7 27, T9 32, T8 33, then T3 GPIO15 (boot-strap pin,
// fine with a floating foil), T2 GPIO2 (on-board LED + boot strap) and last T5
// GPIO12 (sets flash voltage at boot — avoid unless you need a 9th book).
static const uint8_t PINS[] = { 4, 13, 14, 27, 32, 33, 15, 2, 12 };
#define CHIP_ID 1
#else
#error "Unsupported chip for Book Piano."
#endif

// ---- touch reading: always "rises when touched" ----
// S2/S3 (touch v2) count up with capacitance; the classic ESP32 (touch v1)
// counts DOWN. We invert v1 readings so everything above (thresholds as % of
// resting, TapTune's rise %) works the same on every chip.
#if SOC_TOUCH_SENSOR_VERSION == 1
#define TOUCH_FALLS 1
inline uint32_t readTouch(uint8_t pin) {
  uint32_t r = touchRead(pin);
  return r ? 100000000UL / r : 0;  // fewer counts (finger) → bigger value; ~same % change
}
#else
#define TOUCH_FALLS 0
inline uint32_t readTouch(uint8_t pin) { return touchRead(pin); }
#endif

// ---- USB MIDI (chips with native USB, built with USB Mode = USB-OTG/TinyUSB) ----
#if SOC_USB_OTG_SUPPORTED && !ARDUINO_USB_MODE
#define HAS_USB_MIDI 1
#include "USB.h"
#include "USBMIDI.h"
#else
#define HAS_USB_MIDI 0
typedef struct { uint8_t header, byte1, byte2, byte3; } midiEventPacket_t;
struct NoUsbMidi {  // same calls as USBMIDI, doing nothing
  explicit NoUsbMidi(const char *) {}
  void begin() {}
  void noteOn(uint8_t, uint8_t, uint8_t) {}
  void noteOff(uint8_t, uint8_t, uint8_t) {}
  void controlChange(uint8_t, uint8_t, uint8_t) {}
  bool readPacket(midiEventPacket_t *) { return false; }
  bool writePacket(midiEventPacket_t *) { return false; }
};
#endif

// ---- Bluetooth LE MIDI (not on ESP32-S2) ----
#if SOC_BLE_SUPPORTED
#define HAS_BLE 1
#else
#define HAS_BLE 0
#endif

// ---- a second serial port for commands (the COM/UART port), when Serial is USB ----
#if ARDUINO_USB_CDC_ON_BOOT
#define HAS_SERIAL0 1
#else
#define HAS_SERIAL0 0
#endif
