# Book Piano firmware

Firmware for the **Book Piano**: copper/foil strips hidden in book spines,
read by an ESP32's capacitive touch pins and played through
[TapTune](https://harsha20599.github.io/taptune/) (or any MIDI app).

By [@yem.cube](https://www.instagram.com/yem.cube/). Free for personal use and
tweaking; sharing it publicly needs a visible @yem.cube credit. See
[LICENSE](../LICENSE).

## Which boards work

| Chip | Example boards | Touch pins (books per board) | Connects by | Status |
|---|---|---|---|---|
| **ESP32-S3** | YD-ESP32-S3 / DevKitC-1 (N16R8, N8R8…), S3 SuperMini | 14 (SuperMini: 13 broken out) | Bluetooth MIDI + USB MIDI | ✅ tested on hardware |
| **ESP32** (classic) | 30/38-pin "ESP32 DevKit V1", ESP-WROOM-32 boards | 9 | Bluetooth MIDI | ⚠️ builds, not yet tested on hardware |
| **ESP32-S2** | S2 Mini, Saola | 14 | USB MIDI only (no Bluetooth) | ⚠️ builds, not yet tested on hardware |
| ESP32-C3 / C6 / H2 | – | – | – | ❌ these chips have no touch pins |

One board is enough. A **second board** (any supported chip) adds up to 14
more books; it talks to the main board wirelessly (ESP-NOW) and only needs
power. Boards can be mixed, e.g. an S3 main board and a classic ESP32 second.

## Pins

Each book's strip goes to one touch pin, with one plain wire. Nothing else.

**ESP32-S3 / S2:** GPIO 1–14 (books 1–12 default to GPIO 1–12; 13–14 are spare).

**ESP32 (classic):** in this order — GPIO 4, 13, 14, 27, 32, 33, 15, 2, 12.
GPIO 15, 2 and 12 are boot-strap pins: they work, but are last on purpose.
GPIO 2 also drives the blue LED on many boards (it may glow faintly).
Use GPIO 12 only if you need the 9th book.

TapTune shows each pin by its real GPIO number.

### Do not connect

- No **3V3, 5V or GND** to any strip. Strips connect to a touch pin only.
- Strips must not touch each other, a metal shelf, or other wires.
- Between two boards, never connect 3V3 to 3V3. Power each board from USB,
  or both from one good charger.
- Use a **good USB charger**. A cheap/noisy charger was the #1 cause of
  random notes while building this.

### Wires and strips

- Keep each wire as **short** as you can (under ~50 cm), and don't bundle
  book wires tightly together: wires next to each other "feel" each other.
- A strip of foil or copper tape about 1–2 cm wide along the spine works
  well. Bigger = stronger touch signal, but also more noise.
- Hide the strip under the spine label or cover; touch works through paper.

## Flash a prebuilt firmware (easiest)

Prebuilt files are in [`bin/`](bin/). Each is a single image written at
address **0x0**.

| File | For |
|---|---|
| `book-piano-parent-esp32s3-4mb.bin` | main board, any ESP32-S3 (4/8/16 MB flash; SuperMini too) |
| `book-piano-parent-esp32s3-16mb.bin` | main board, S3 with 16 MB flash (bigger file system; optional) |
| `book-piano-parent-esp32.bin` | main board, classic ESP32 |
| `book-piano-parent-esp32s2.bin` | main board, ESP32-S2 |
| `book-piano-child-*.bin` | the same, for the second board |

With [esptool](https://docs.espressif.com/projects/esptool/):

```
esptool --chip auto write-flash 0x0 book-piano-parent-esp32s3-4mb.bin
```

(older esptool versions spell it `write_flash`). If the board isn't found, hold
**BOOT**, tap **RESET**, release BOOT, and try again.

Web flashers: `bin/manifest-main.json` and `bin/manifest-second.json` are
[ESP Web Tools](https://esphome.github.io/esp-web-tools/) manifests (they pick
the right file for the chip).

## Build it yourself

1. Arduino IDE 2 (or arduino-cli) with **esp32 by Espressif**, version 3.3.x
   (tested with 3.3.12). No other libraries needed.
2. Open `book_piano/book_piano.ino`.
3. Board settings:

| | ESP32-S3 | ESP32 (classic) | ESP32-S2 |
|---|---|---|---|
| Board | ESP32S3 Dev Module | ESP32 Dev Module | ESP32S2 Dev Module |
| USB Mode | **USB-OTG (TinyUSB)** | – | – |
| USB CDC On Boot | **Enabled** | – | **Enabled** |
| Flash Size | your board's (4/8/16 MB) | 4 MB | 4 MB |
| Partition Scheme | **Minimal SPIFFS (1.9MB APP with OTA)** | **Minimal SPIFFS** | **Minimal SPIFFS** |

   With USB Mode left on "Hardware CDC and JTAG", an S3 still works over
   Bluetooth but has no USB MIDI.

   arduino-cli FQBNs used for the prebuilt files:
   - `esp32:esp32:esp32s3:FlashSize=4M,PartitionScheme=min_spiffs,USBMode=default,CDCOnBoot=cdc`
   - `esp32:esp32:esp32:PartitionScheme=min_spiffs`
   - `esp32:esp32:esp32s2:PartitionScheme=min_spiffs,CDCOnBoot=cdc`
4. **Second board:** add `#define BOOK_PIANO_CHILD` as the first line of
   `book_piano.ino` (or with arduino-cli:
   `--build-property compiler.cpp.extra_flags=-DBOOK_PIANO_CHILD`).
5. **Wireless-update password:** copy `ota_secret.example.h` to
   `ota_secret.h` and set your own. Without it the build uses the public
   default `bookpiano` (and prints a warning). `ota_secret.h` is git-ignored.

## First start

1. Flash the main board (and the second, if you have one). Wire the strips.
2. Power the board(s) with nobody touching the books — they measure the
   resting level at start.
3. Open [TapTune](https://harsha20599.github.io/taptune/) in **Chrome** or
   **Edge** (Android, Windows, Mac, Linux).
4. Connect: **Bluetooth** (device name **"Book Piano"**) or a USB cable
   (USB MIDI device "Book Piano", S3/S2 only).
5. Open **Setup** → find each book (tap it), and let the wizard set its touch
   sensitivity. The setup is saved on the main board.

The second board pairs with the main board by itself (it searches the radio
channels); TapTune shows it as "Second board: online".

**LED:** the firmware doesn't use an LED; the board's power LED just shows
it has power.

## Wireless updates (optional)

TapTune's **Update mode** button puts both boards into update mode for 10
minutes. A board with no saved Wi-Fi opens an open hotspot
**BookPiano-Setup-Parent** / **-Child**; join it and type your Wi-Fi name and
password on the page that opens. Then upload over the network from the
Arduino IDE (port `book-piano-parent` / `book-piano-child`) with your OTA
password. Normal playing never uses Wi-Fi.

## Serial commands (for tinkering)

On the USB serial port (S3/S2: 921600 baud; classic ESP32: 115200):
`s` status · `d` pin readings · `r` re-baseline · `v` live stream on/off ·
`u` update mode · `C<1-13>` radio channel · `K 2 0:60 1:62` set books from a
computer.

## How it works (short)

- Every board talks in 14 "slots"; slot *i* is its *i*-th touch pin. Sensors
  0–13 are the main board, 14–27 the second board. A **book** points at one
  sensor and has a note.
- Each pin: median-of-5 spike filter, slow baseline tracking, a per-pin
  threshold (% above resting), and ground-noise rejection (if all pins rise
  together, it's noise, not a finger).
- On the classic ESP32 readings fall when touched; the firmware flips them
  so every chip behaves the same.
- Settings and live sensor data use MIDI SysEx (`F0 7D 42 …`); see the top
  of `config.ino`.
