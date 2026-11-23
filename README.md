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
| Ground | Horizon Dev Ground: RP2350 + LR2021 USB receiver |

Radio: LoRa SF5, 1 MHz, sub-GHz (150–960 MHz) or 2.4 GHz with frequency hopping. Every packet carries a CRC keyed
with the pair's link ID (from the callsign), Reed-Solomon repair inside the packet, and block FEC across packets
(8 + 4 by default).

## What is where

| Folder | Contents |
| --- | --- |
| `lora_vlink/src` | RP2350 / RP2354 radio firmware: Air radio controller, Pi HAT transmitter, Ground receiver |
| `luckfox_vtx` | `hz_vtx`, the transmitter program on the Luckfox (camera, hardware H.265 encoder, packets) |
| `luckfox_image` | The Luckfox microSD image: build scripts, self-check, pairing, updates |
| `pi5_vtx`, `pi_image` | The Raspberry Pi 5 transmitter (x265) and its Raspberry Pi OS image; `pi5_vtx/swfec.py` and `blockfec.py` are the reference of the over-the-air format |
| `desktop_receiver` | Horizon Receiver for Windows (Rust) |
| `win_decoder` | Its H.265 decoder, `hz_decoder.dll`: unmodified FFmpeg, reproducible build |
| `tools` | Build of the Windows app (`build_desktop_app.sh`), factory test |

Builds need Linux or WSL: the Luckfox SDK for the camera side, pico-sdk 2.x for the firmware, MinGW-w64 for the
decoder, and Rust on Windows for the app. The scripts say what they need at the top. Ready-made firmware, card images
and the Windows app are on the setup site.

## License

Source-available under the [Prosperity Public License 3.0.0](LICENSE.md): free for personal, research and
noncommercial use, and to try commercially for 30 days. Other commercial use needs permission from BPYL d.o.o.
Third-party code keeps its own license ([THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)). The Horizon and BPYL
names and logos are trademarks.

Contributions are welcome on any hardware: see [CONTRIBUTING.md](CONTRIBUTING.md).

LoRa® is a registered trademark of Semtech Corporation.
