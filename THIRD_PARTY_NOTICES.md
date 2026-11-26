# Third-party software

The Horizon code is under the Prosperity Public License 3.0.0 (LICENSE.md). These parts are someone else's and keep their
own licenses. Anyone who distributes a build must pass these notices on with it.

| Component | Where | License | What a distributor must do |
| --- | --- | --- | --- |
| Semtech LR20xx radio driver | `radio_firmware/third_party/lr20xx_driver/` (in the radio firmware) | Clear BSD (its `LICENSE.txt`) | Keep the copyright notice and license text, in source and with firmware binaries. |
| Raspberry Pi Pico SDK | fetched by `radio_firmware/pico_sdk_import.cmake` (in the radio firmware) | BSD-3-Clause | Include its copyright notice and license with firmware binaries. |
| FFmpeg (HEVC decoder only) | `hz_decoder.dll` of the desktop app (FFmpeg 8.0, `decoder/native/build_native.sh`) and the setup site's browser decoder (FFmpeg 8.0 in WebAssembly, same `decoder.c`) | LGPL-2.1-or-later | Include the LGPL text, say which FFmpeg version was used, and offer the FFmpeg source and the build scripts, so users can rebuild the decoder with a changed FFmpeg. |
| Rust crates of the desktop app (Tauri and others) | `desktop_receiver/Cargo.lock` | mostly MIT and/or Apache-2.0 | Ship their notices with the app (to generate before a release, e.g. with `cargo about`). |
| Luckfox Pico SDK system | the Air microSD image (`luckfox_image/build_image.sh`): Linux kernel, U-Boot, BusyBox and the rest of the SDK's root file system, not in this repository | GPL-2.0 and others, each component's own | Offer the complete corresponding source of the SDK release the image was built from, with its license texts. |
| Rockchip libraries | linked by `transmitter/hz_vtx` on the Air, from the Luckfox SDK, not in this repository | Rockchip's terms | Passed on unchanged in the Air image, under the SDK's terms. |
| Linux USB serial gadget (`u_serial.ko`, `usb_f_acm.ko`) | in the Air image, built from the SDK kernel's own source by `luckfox_image/build_usb_serial.sh` (only the port's names changed) | GPL-2.0 | Offer the kernel source and that build script with the image. |
| OpenOCD | in the Air image (programs the radio controller over SWD), built separately, not in this repository | GPL-2.0-or-later | Offer the exact OpenOCD source it was built from, with its license. |
| x265 | linked by `hz_x265`, the Pi's encoder process (`transmitter/x265_encoder.c`), from the Raspberry Pi OS package; `hz_vtx` does not link it | GPL-2.0-or-later | Offer the x265 source of the package in the image. |
| libcamera | linked by the Pi's `hz_vtx` (`transmitter/camera_pi.cpp`), from the Raspberry Pi OS package | LGPL-2.1-or-later | Offer its source; it is a shared library the user can replace. |
| Raspberry Pi OS | the Pi microSD image (`pi_image/build_pi_image.sh`, release pinned in `pi_image/raspios.pin`), not in this repository | GPL and others (Debian packages) | Offer the source of the packages in the image (Raspberry Pi and Debian publish it for each release). |

Python packages used by the development tools (numpy, PyAV, ...) are installed by the user and not
redistributed.
