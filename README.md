# HORIZON Video Link

Long-range live video over LoRa® radio, from BPYL d.o.o. A camera unit (the Air unit) encodes H.265 and sends it
over an LR2021 radio; a USB receiver (the Ground) brings it to a computer, where a browser page or a Windows app
shows it.

This is the software of **Horizon Dev**: firmware, transmitter programs and receivers.
Set up and flash the hardware from the browser at **https://horizon.bpyl.si/** (Chrome or Edge).

> Experimental, for research and education: read [DISCLAIMER.md](DISCLAIMER.md). Transmit only on frequencies and at
> powers you are allowed to use where you are.

## Hardware

| Part | What it is |
| --- | --- |
| Air unit, Horizon Dev Air | Luckfox Pico (RV1103) camera board with an RP2354 radio controller and an LR2021 module |
| Air unit, Raspberry Pi 5 | A Pi 5 with a camera and the Horizon HAT (RP2350 + LR2021) |
| Ground, Horizon Dev Ground Unit | USB receiver: RP2354 + Seeed Wio-LR2021 module, one SMA port per band (sub-GHz, 2.4 GHz) |
| Ground, Horizon HAT | The same HAT as the Pi 5 Air unit, with the receiver firmware: plugged into the computer by its own USB port, no Pi needed. Which role it has depends only on the firmware the setup site puts on it |

Radio: LoRa SF5, 1 MHz, sub-GHz (150–960 MHz) or 2.4 GHz with frequency hopping. Every packet carries a CRC keyed
with the pair's link ID (from the callsign), Reed-Solomon repair inside the packet, and block FEC across packets
(8 + 4 by default).

## What is where

| Folder | Contents |
| --- | --- |
| `radio_firmware/src` | RP235x radio firmware: Air radio controller, Pi HAT transmitter, Ground receivers (Ground Unit, HAT) |
| `transmitter` | `hz_vtx`, the transmitter program of both Air units: shared core, the Luckfox's camera and hardware H.265 encoder, the Pi's camera and x265 |
| `luckfox_image` | The Luckfox microSD image: build scripts, the transmitter service, self-check, pairing, updates |
| `format` | The over-the-air format: how it works (README), its reference code in Python and C |
| `pi_image` | The Raspberry Pi 5 image (Raspberry Pi OS with `hz_vtx`), its service, and the setup for your own Raspberry Pi OS |
| `desktop_receiver` | Horizon Receiver for Windows (Rust) |
| `decoder` | The H.265 decoder: `hz_decoder.dll` of the Windows app (unmodified FFmpeg, reproducible build) |
| `tools` | Build of the Windows app (`build_desktop_app.sh`) |
| `test`, `*/test` | The tests: each folder's own in its `test/`, the runner and the end-to-end ones in `test/` |

Builds need Linux or WSL: the Luckfox SDK for the Air's `hz_vtx` (`transmitter/build_luckfox.sh`), pico-sdk 2.x for
the firmware, MinGW-w64 for the decoder, and Rust on Windows for the app. The Pi's `hz_vtx` builds on the Pi itself
(`transmitter/build_pi.sh`, or `pi_image/setup_pi.sh` for the whole install). The scripts say what they need at the
top. Ready-made firmware, card images
and the Windows app are on the setup site.

## Tests

```sh
bash test/run.sh            # lists the plugged-in boards, asks what to test
bash test/run.sh air        # or pi-hat, ground-unit, hat-rx: the device's software, then the device over USB
bash test/run.sh pair       # factory test: an Air and a Ground over the radio
bash test/run.sh app        # or site, all: software only, nothing plugged in
```

A device run builds its firmware and runs the host tests of its software, then finds the board by asking each USB
serial port who it is and checks it (identity, radio, self-check). Linux, or Git Bash on Windows (the C and Python
tests and the firmware build then run in WSL). The top of `test/run.sh` lists what each choice runs.

CI (`.github/workflows/ci.yml`) runs `test/run.sh all` and the desktop app's tests on GitHub's runners for every
push and pull request, and keeps the four firmware builds as artifacts.

## License

Source-available under the [Prosperity Public License 3.0.0](LICENSE.md): free for personal, research and
noncommercial use, and to try commercially for 30 days. Other commercial use needs permission from BPYL d.o.o.
Third-party code keeps its own license ([THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)). The Horizon and BPYL
names and logos are trademarks.

Contributions are welcome on any hardware: see [CONTRIBUTING.md](CONTRIBUTING.md).

LoRa® is a registered trademark of Semtech Corporation.
