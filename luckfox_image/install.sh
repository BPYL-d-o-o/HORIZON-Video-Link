#!/bin/sh
# Installs a Horizon Dev Air build on an Air connected to this machine over adb: an update bundle
# (horizon-dev-air-VERSION.update.tar.xz, from luckfox_image/build_image.sh: a release in ~/horizon-image/releases/
# built by the git hooks, or the setup site's camera/ folder). The Air installs it exactly as the setup site's
# "Update the Air" does (luckfox_image/horizon_update.sh): settings and callsign kept, the radio controller flashed
# when its firmware changed, and the previous version restored if the transmitter does not start.
#
#   sh luckfox_image/install.sh BUNDLE.update.tar.xz     (ADB_SERIAL=<serial> to pick one of several boards)
#   sh luckfox_image/install.sh --pull-libs              copy librockit.so / librkaiq.so from the Luckfox into
#                                                      transmitter/build/targetlib/ (needed to build hz_vtx)
#
# Needs an Air running the Horizon Dev Air card image (setup site, step 1): the bundle has no OpenOCD, kernel modules
# or boot changes of its own. Prints the installer's EVENT lines and RESULT PASS|FAIL.
set -eu
here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
case "$(uname -s)" in  # Git Bash: keep the Air's paths from being rewritten into Windows paths
MINGW* | MSYS*) export MSYS_NO_PATHCONV=1; here=$(cygpath -m "$here") ;;
esac
A="adb ${ADB_SERIAL:+-s $ADB_SERIAL}"
$A get-state >/dev/null 2>&1 || { echo "FATAL no Luckfox on adb (adb devices)"; echo "RESULT FAIL"; exit 1; }

case "${1:-}" in
--pull-libs)
    mkdir -p "$here/build/targetlib"
    $A pull /oem/usr/lib/librockit.so "$here/build/targetlib/" >/dev/null
    $A pull /oem/usr/lib/librkaiq.so "$here/build/targetlib/" >/dev/null
    sha256sum "$here/build/targetlib/"*.so
    exit 0
    ;;
''|-*)
    echo "usage: $0 BUNDLE.update.tar.xz | --pull-libs" >&2; exit 2
    ;;
esac
bundle=$1
case "$(uname -s)" in MINGW* | MSYS*) bundle=$(cygpath -w "$bundle") ;; esac
test -f "$bundle" || { echo "FATAL no bundle $bundle"; echo "RESULT FAIL"; exit 1; }
$A shell "test -f /usr/share/horizon/VERSION" ||
    { echo "FATAL this Luckfox has no Horizon Dev Air card image: write the card first (setup site, step 1)"; echo "RESULT FAIL"; exit 1; }

# on the card, not in /tmp (RAM, of which the Air has little to spare)
want=$(md5sum <"$bundle" | cut -d' ' -f1)
$A push "$bundle" /userdata/horizon_update.tar.xz >/dev/null
have=$($A shell "md5sum </userdata/horizon_update.tar.xz" | cut -d' ' -f1 | tr -d '\r')
[ "$want" = "$have" ] || { echo "FATAL the copy on the Air differs"; echo "RESULT FAIL"; exit 1; }
# the bundle's own installer (it knows its own layout), detached from this adb session
out=$($A shell "rm -rf /userdata/horizon_update.inst; mkdir -p /userdata/horizon_update.inst
    xz -dc /userdata/horizon_update.tar.xz | tar -xf - -C /userdata/horizon_update.inst share/horizon_update.sh &&
    setsid sh /userdata/horizon_update.inst/share/horizon_update.sh /userdata/horizon_update.tar.xz </dev/null 2>&1
    rm -rf /userdata/horizon_update.inst /userdata/horizon_update.tar.xz" | tr -d '\r')
echo "$out"
echo "$out" | grep -q '^RESULT PASS'
