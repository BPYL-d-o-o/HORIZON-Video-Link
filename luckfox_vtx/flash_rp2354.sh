#!/bin/sh
# Flashes a firmware build onto the Horizon Dev Air's radio controller (RP2354A) from this machine, with the Air on
# adb. The flashing itself is the board's own (luckfox_image/rp2354_update.sh flash: OpenOCD over SWD, transmitter
# stopped and started again); this only copies the file there.
#
#   sh luckfox_vtx/flash_rp2354.sh FIRMWARE.elf          (ADB_SERIAL=<serial> to pick one of several boards)
#
# For testing: the Air puts its image's own firmware back at the next power-up or update. To keep a build, install a
# whole update bundle instead (luckfox_vtx/install.sh). Prints RESULT PASS|FAIL.
set -eu
elf=${1:?usage: flash_rp2354.sh FIRMWARE.elf}
case "$(uname -s)" in  # Git Bash: keep the Air's paths from being rewritten into Windows paths
MINGW* | MSYS*) export MSYS_NO_PATHCONV=1; elf=$(cygpath -w "$elf") ;;
esac
A="adb ${ADB_SERIAL:+-s $ADB_SERIAL}"
$A get-state >/dev/null 2>&1 || { echo "FATAL no Air on adb"; echo "RESULT FAIL"; exit 1; }
$A shell "test -f /usr/share/horizon/rp2354_update.sh" ||
    { echo "FATAL this Luckfox has no Horizon Dev Air image (write the card from the setup site)"; echo "RESULT FAIL"; exit 1; }
want=$(md5sum <"$elf" | cut -d' ' -f1)
$A push "$elf" /userdata/rp2354.test.elf >/dev/null
have=$($A shell "md5sum </userdata/rp2354.test.elf" | cut -d' ' -f1 | tr -d '\r')
[ "$want" = "$have" ] || { echo "FATAL the copy on the Air differs"; echo "RESULT FAIL"; exit 1; }
out=$($A shell "sh /usr/share/horizon/rp2354_update.sh flash /userdata/rp2354.test.elf; rm -f /userdata/rp2354.test.elf" | tr -d '\r')
echo "$out"
echo "$out" | grep -q '^RESULT PASS'
