#!/bin/sh
# Builds everything a Horizon Dev Air card and its Ground receiver carry, from one source tree (Linux or WSL). The git
# hooks (tools/git-hooks/) run it on an exported commit or on the staged files; it can also be run by hand from a
# checkout.
#
#   sh luckfox_image/air_build.sh check      firmware (Air air_radio, Ground hat_rx and ground_unit,
#                                            Pi HAT hat_tx) and
#                                            hz_vtx: compile only
#   sh luckfox_image/air_build.sh release    the same, then the USB serial modules and the Air microSD image, into
#                                            $IMAGES/<HZ_GIT>/ with the Ground firmware (hat_rx.uf2/.elf, Ground Unit
#                                            ground_unit.uf2/.elf) and the
#                                            Raspberry Pi HAT firmware (hat_tx.uf2/.elf) beside it
#
# Environment:
#   BUILD          persistent build directory (incremental builds: firmware in $BUILD/fw, logs $BUILD/*.log)
#   HZ_GIT         the version stamped into the firmware, the image name and its manifest (e.g. a short commit)
#   LUCKFOX_SDK    Luckfox Pico SDK, built for the Pico SD card (luckfox_image/build_image.sh)
#   PICO_SDK_PATH  pico-sdk 2.x
#   OPENOCD        OpenOCD for the Luckfox (release only; build_image.sh)
#   TARGETLIB      librockit/librkaiq copied from a Luckfox (transmitter/build_luckfox.sh, install.sh --pull-libs)
#   IMAGES         release output root (release only)
#   INPUTS         fingerprint of the sources (release only): $IMAGES/inputs/<INPUTS> then names the release, so the
#                  pre-commit hook can tell whether the committed code has an image
#
# Prints EVENT lines and RESULT PASS|FAIL; build output goes to $BUILD/*.log (the tail is shown on failure).
set -eu
src=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
mode=${1:?usage: air_build.sh check|release}
build=${BUILD:?set BUILD}
ver=${HZ_GIT:?set HZ_GIT}
: "${LUCKFOX_SDK:?set LUCKFOX_SDK}" "${PICO_SDK_PATH:?set PICO_SDK_PATH}" "${TARGETLIB:?set TARGETLIB}"
export LUCKFOX_SDK PICO_SDK_PATH
fail() { echo "FATAL $*"; echo "RESULT FAIL"; exit 1; }
logfail() { echo "---- last lines of $1"; tail -25 "$1" | sed 's/^/  /'; fail "$2"; }
case "$mode" in check|release) ;; *) fail "unknown mode $mode (check|release)" ;; esac
mkdir -p "$build"
t0=$(date +%s)

# 1. radio firmware
echo "EVENT firmware git=$ver"
export HZ_FW_GIT=$ver  # for the configure and for CMake's automatic re-configure during the build
cmake -S "$src/radio_firmware" -B "$build/fw" -DCMAKE_BUILD_TYPE=Release >"$build/fw.log" 2>&1 ||
    logfail "$build/fw.log" "firmware: cmake configure"
cmake --build "$build/fw" -j"$(nproc)" --target air_radio hat_rx ground_unit hat_tx >>"$build/fw.log" 2>&1 ||
    logfail "$build/fw.log" "firmware: build"
for t in air_radio hat_rx ground_unit hat_tx; do
    strings "$build/fw/$t.elf" | grep -qx "$ver" || fail "firmware: $t.elf does not carry git=$ver"
done

# 2. hz_vtx, with its host tests
echo "EVENT hz_vtx"
mkdir -p "$src/transmitter/build/targetlib"
cp -p "$TARGETLIB"/* "$src/transmitter/build/targetlib/"
HZ_GIT=$ver sh "$src/transmitter/build_luckfox.sh" >"$build/hz_vtx.log" 2>&1 || logfail "$build/hz_vtx.log" "hz_vtx: build or host tests"

if [ "$mode" = check ]; then
    echo "EVENT built in $(( $(date +%s) - t0 )) s (compile check, no image)"
    echo "RESULT PASS"
    exit 0
fi

# 3. the USB serial gadget modules and the microSD image
: "${OPENOCD:?set OPENOCD}" "${IMAGES:?set IMAGES}" "${INPUTS:?set INPUTS}"
echo "EVENT usb_serial"
sh "$src/luckfox_image/build_usb_serial.sh" "$build/usb_serial" >"$build/usb_serial.log" 2>&1 ||
    logfail "$build/usb_serial.log" "USB serial modules"
out=$IMAGES/$ver
rm -rf "$out.tmp"
mkdir -p "$out.tmp"
echo "EVENT image"
HZ_GIT=$ver FW_ELF=$build/fw/air_radio.elf HZ_VTX=$src/transmitter/build/hz_vtx USB_SERIAL=$build/usb_serial \
    OPENOCD=$OPENOCD sh "$src/luckfox_image/build_image.sh" "$out.tmp" "$ver" >"$build/image.log" 2>&1 ||
    logfail "$build/image.log" "image"
cp "$build/fw/hat_rx.uf2" "$build/fw/hat_rx.elf" "$build/fw/ground_unit.uf2" "$build/fw/ground_unit.elf" \
   "$build/fw/hat_tx.uf2" "$build/fw/hat_tx.elf" "$out.tmp/"
echo "$INPUTS" >"$out.tmp/inputs"
rm -rf "$out"
mv "$out.tmp" "$out"
mkdir -p "$IMAGES/inputs"
echo "$ver" >"$IMAGES/inputs/$INPUTS"
ln -sfn "$ver" "$IMAGES/latest"
echo "EVENT release $out: $(sed -n 's/^{"file": "\([^"]*\)".*/\1/p' "$out/manifest.json"), hat_rx.uf2, ground_unit.uf2, hat_tx.uf2 ($(( $(date +%s) - t0 )) s)"
echo "RESULT PASS"
