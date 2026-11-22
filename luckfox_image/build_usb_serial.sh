#!/bin/sh
# Builds the USB serial gadget function (CDC ACM: u_serial.ko + usb_f_acm.ko) for the Luckfox Pico kernel, so the
# Horizon Dev Air shows up as a USB serial port ("Horizon Dev Air") next to its network and adb functions; the setup
# site pairs it with the receiver over that port (luckfox_image/horizon_pair.sh). The stock kernel has configfs
# gadgets (CONFIG_USB_CONFIGFS=y) but not this function; nothing in the kernel itself changes.
#
#   LUCKFOX_SDK=~/luckfox-sdk sh luckfox_image/build_usb_serial.sh OUT_DIR      (Linux or WSL)
#
# Needs the SDK's kernel build (sysdrv/source/objs_kernel, from the SDK's ./build.sh). Checks: stock vermagic, and
# every symbol the modules use is exported by that kernel or by the two modules. Prints RESULT PASS|FAIL.
set -eu
here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
out=${1:?usage: build_usb_serial.sh OUT_DIR}
sdk=${LUCKFOX_SDK:?set LUCKFOX_SDK}
K=$sdk/sysdrv/source/kernel
O=$sdk/sysdrv/source/objs_kernel
PATH=$sdk/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export ARCH=arm CROSS_COMPILE=arm-rockchip830-linux-uclibcgnueabihf-
vermagic="5.10.160 mod_unload ARMv7 thumb2 p2v8 "
fail() { echo "FATAL $*"; echo "RESULT FAIL"; exit 1; }
test -f "$O/Module.symvers" || fail "no kernel build in $O (run the SDK's ./build.sh)"
grep -qx 'CONFIG_USB_CONFIGFS=y' "$O/.config" || fail "kernel without configfs USB gadgets"
grep -q '^CONFIG_USB_F_ACM=\|^CONFIG_USB_U_SERIAL=' "$O/.config" && fail "this kernel already has the serial gadget"

work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
cp "$here/usb_serial/Kbuild" "$work/"
cp "$K/drivers/usb/gadget/function/f_acm.c" "$K/drivers/usb/gadget/function/u_serial.c" \
   "$K/drivers/usb/gadget/function/u_serial.h" "$work/"
# the port's names (interface and function group), which the OS and the browser's port chooser show
sed -i -e 's|\[ACM_CTRL_IDX\].s = "CDC Abstract Control Model (ACM)",|[ACM_CTRL_IDX].s = "Horizon Dev Air rev-1A",|' \
       -e 's|\[ACM_DATA_IDX\].s = "CDC ACM Data",|[ACM_DATA_IDX].s = "Horizon Dev Air rev-1A data",|' \
       -e 's|\[ACM_IAD_IDX \].s = "CDC Serial",|[ACM_IAD_IDX ].s = "Horizon Dev Air rev-1A",|' "$work/f_acm.c"
[ "$(grep -c '"Horizon Dev Air rev-1A' "$work/f_acm.c")" = 3 ] || fail "f_acm.c: interface names not found"
make -C "$K" O="$O" M="$work" modules >"$work/build.log" 2>&1 || { tail -20 "$work/build.log"; fail "module build"; }
mkdir -p "$out"
awk '{print $2}' "$O/Module.symvers" | sort -u >"$work/exports"
for m in u_serial usb_f_acm; do
    ${CROSS_COMPILE}strip --strip-debug "$work/$m.ko"
    ${CROSS_COMPILE}nm "$work/$m.ko" | awk '$2 ~ /^[TDR]$/ {print $3}' >>"$work/exports"
done
sort -u -o "$work/exports" "$work/exports"
for m in u_serial usb_f_acm; do
    got=$(${CROSS_COMPILE}strings "$work/$m.ko" | sed -n 's/^vermagic=//p' | head -1)
    [ "$got" = "$vermagic" ] || fail "vermagic $m.ko: $got"
    missing=$(${CROSS_COMPILE}nm -u "$work/$m.ko" | awk '{print $2}' | sort -u | comm -23 - "$work/exports" | tr '\n' ' ')
    [ -z "$missing" ] || fail "unresolved in $m.ko: $missing"
    cp "$work/$m.ko" "$out/"
done
echo "EVENT modules $out/u_serial.ko $out/usb_f_acm.ko vermagic=ok unresolved=0"
echo "RESULT PASS"
