# TapTune

A low-latency web piano for the **Book Piano**: a row of ordinary-looking books
with hidden touch sensors, read by ESP32 boards that show up as a Bluetooth /
USB MIDI device called "Book Piano". Firmware, wiring and prebuilt files:
[firmware/](firmware/).

By [@yem.cube](https://www.instagram.com/yem.cube/).

**Open it:** https://harsha20599.github.io/taptune/

## Use it

1. Plug the Book Piano's main board into your phone or computer (Android: a
   USB-C cable; the phone powers the board).
2. Open the link above in **Chrome** or **Edge**. Safari can't read MIDI.
3. Tap **Start piano**, allow MIDI, and touch the books.

After the first visit the page and piano sounds are cached, so it also works
offline. On Android you can "Install app" from Chrome's menu.

## What's on the page

- Piano sound that starts the moment a MIDI note arrives: samples are decoded
  into memory up front, the silent start of each sample is trimmed, and notes
  start with no scheduling look-ahead.
- Audio-delay readout, so you can pick the fastest device.
- **Books** buttons: re-baseline the touch sensors, learn taps, update mode
  (wireless firmware update).
- **Books → notes**: touch a book to find its row, choose its note, and save
  the map to the board.

## Credits

Piano samples: **Salamander Grand Piano** by Alexander Holm, licensed
[CC BY 3.0](https://creativecommons.org/licenses/by/3.0/), via
[Tone.js audio](https://github.com/Tonejs/audio).

## License

Free for personal use and tweaking. **Sharing it publicly** (code, builds,
hosted copies, videos or posts that show it) **needs a visible credit to
[@yem.cube](https://www.instagram.com/yem.cube/)**. Commercial use needs
permission. Full terms: [LICENSE](LICENSE).
