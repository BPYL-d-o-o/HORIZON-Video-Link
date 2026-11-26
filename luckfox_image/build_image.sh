#!/bin/sh
# Builds the Horizon Dev Air microSD image: the official Luckfox Pico SD-card Buildroot firmware plus the Horizon
# transmitter, written straight into the root filesystem image (debugfs: no root, no loop mounts). Linux or WSL.
#
#   LUCKFOX_SDK=~/luckfox-sdk  FW_ELF=.../air_radio.elf  OPENOCD=.../openocd-armhf.stage/opt/openocd \
#       sh luckfox_image/build_image.sh OUT_DIR [VERSION]
#
# LUCKFOX_SDK  the Luckfox Pico SDK after `./build.sh` with BoardConfig-SD_CARD-Buildroot-RV1103_Luckfox_Pico-IPC.mk
#              (output/image/ holds env/idblock/uboot/boot/oem/userdata/rootfs.img and sd_update.txt)
# FW_ELF       the RP2354A firmware (radio_firmware target air_radio)
# OPENOCD      an OpenOCD install for the Luckfox (armhf, static, with RP2350 support): bin/openocd, share/openocd/scripts
# HZ_VTX       the transmitter program (default transmitter/build/hz_vtx, from transmitter/build_luckfox.sh); the SPI link
#              test transmitter/build/spi_speed_test is taken from beside it
# USB_SERIAL   u_serial.ko + usb_f_acm.ko (luckfox_image/build_usb_serial.sh): the USB serial port the setup site
#              pairs the board over (horizon_pair.sh)
#
# Writes OUT_DIR/horizon-dev-air-VERSION.img.xz, its .sha256, the update bundle horizon-dev-air-VERSION.update.tar.xz
# (the same programs without the card: the setup site installs it over USB, horizon_update.sh) and manifest.json
# (for the setup site), and prints
# RESULT PASS|FAIL. The card layout is the SDK's (sd_update.txt): raw partitions at fixed offsets, no partition
# table; the root filesystem grows to its 6 GB partition on the first boot.
set -eu
here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repo=$(dirname "$here")
out=${1:?usage: build_image.sh OUT_DIR [VERSION]}
sdk=${LUCKFOX_SDK:?set LUCKFOX_SDK}
fw=${FW_ELF:?set FW_ELF}
ocd=${OPENOCD:?set OPENOCD}
vtx=${HZ_VTX:-$repo/transmitter/build/hz_vtx}
usbser=${USB_SERIAL:?set USB_SERIAL (luckfox_image/build_usb_serial.sh)}
git=${HZ_GIT:-$(git -C "$repo" rev-parse --short=8 HEAD 2>/dev/null || echo unknown)}  # HZ_GIT: set by air_build.sh
version=${2:-$git}
img=$sdk/output/image
PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
fail() { echo "FATAL $*"; echo "RESULT FAIL"; exit 1; }

for f in env idblock uboot boot oem userdata rootfs; do test -f "$img/$f.img" || fail "missing $img/$f.img (build the SDK first)"; done
grep -q 'rv1103g-luckfox-pico\.dts\|Luckfox_Pico-IPC' "$sdk/.BoardConfig.mk" || fail "the SDK is not set up for the Luckfox Pico SD card (.BoardConfig.mk)"
test -f "$fw" || fail "missing FW_ELF $fw"
test -x "$ocd/bin/openocd" || fail "missing $ocd/bin/openocd"
test -f "$ocd/share/openocd/scripts/target/rp2350.cfg" || fail "OpenOCD without RP2350 support"
test -f "$vtx" || fail "missing $vtx (sh transmitter/build_luckfox.sh)"
spitest=$(dirname "$vtx")/spi_speed_test
spiupd=$(dirname "$vtx")/rp2354_spi_update   # the RP2354A update over SPI (rp2354_update.sh)
fwbin=${fw%.elf}.bin                        # its flat image (pico_add_extra_outputs builds it beside the .elf)
test -f "$spitest" || fail "missing $spitest (sh transmitter/build_luckfox.sh)"
test -f "$usbser/u_serial.ko" && test -f "$usbser/usb_f_acm.ko" || fail "missing $usbser/u_serial.ko, usb_f_acm.ko"
command -v debugfs >/dev/null && command -v resize2fs >/dev/null && command -v e2fsck >/dev/null || fail "needs e2fsprogs (debugfs, resize2fs, e2fsck)"

work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
mkdir -p "$out"
name=horizon-dev-air-$version

echo "EVENT root filesystem"
cp "$img/rootfs.img" "$work/rootfs.img"
e2fsck -fy "$work/rootfs.img" >/dev/null 2>&1 || true
resize2fs "$work/rootfs.img" 256M >/dev/null 2>&1 || fail "resize2fs"

# The files, staged with their final modes.
st=$work/stage
mkdir -p "$st/openocd" "$st/modules"
cp "$vtx" "$st/hz_vtx"
cp "$repo/luckfox_image/service.sh" "$st/service.sh"
cp "$repo/luckfox_image/horizon_vtx.conf" "$st/horizon_vtx.conf"
cp "$here/rp2354_update.sh" "$st/rp2354_update.sh"
cp "$here/horizon_pair.sh" "$st/horizon_pair.sh"
cp "$here/horizon_check.sh" "$st/horizon_check.sh"
cp "$here/horizon_update.sh" "$st/horizon_update.sh"
cp "$spitest" "$st/spi_speed_test"
cp "$spiupd" "$st/rp2354_spi_update"
cp "$usbser/u_serial.ko" "$usbser/usb_f_acm.ko" "$st/modules/"
cp "$fw" "$st/rp2354.elf"
cp "$fwbin" "$st/rp2354.bin"
cp "$ocd/bin/openocd" "$st/openocd/openocd"
cp -r "$ocd/share/openocd/scripts" "$st/openocd/scripts"
cat >"$st/VERSION" <<EOF
image=$name
git=$git
hz_vtx=$(strings "$vtx" | grep -m1 -E '^[0-9a-f]{8}(-dirty)?$' || echo unknown)
rp2354=$(strings "$fw" | grep -m1 -E '^[0-9a-f]{8}(-dirty)?$' || echo unknown)
built=$(date -u +%Y-%m-%dT%H:%MZ)
freq_hz=$(sed -n 's/^HZ_FREQ_HZ:STRING=//p' "$(dirname "$fw")/CMakeCache.txt" 2>/dev/null | grep . || echo 868000000)
EOF

# The board's name: over USB (the stock gadget says manufacturer "rockchip", product "rk3xxx") and on the network.
debugfs -R "dump /etc/init.d/S50usbdevice $work/S50usbdevice" "$work/rootfs.img" >/dev/null 2>&1
sed -e 's|echo "rockchip"  > ${USB_STRINGS_DIR}/manufacturer|echo "BPYL" > ${USB_STRINGS_DIR}/manufacturer|' \
    -e 's|echo "rk3xxx"  > ${USB_STRINGS_DIR}/product|echo "Horizon Dev Air rev-1A" > ${USB_STRINGS_DIR}/product|' \
    -e 's|^ACM_EN=off$|ACM_EN=on|' \
    -e '/^\. \/etc\/profile$/a # Horizon Dev: the USB serial gadget (the pairing port), which the stock kernel does not have\
for m in u_serial usb_f_acm; do insmod /usr/share/horizon/modules/$m.ko 2>/dev/null; done' \
    "$work/S50usbdevice" >"$work/S50usbdevice.new"
grep -qx 'ACM_EN=on' "$work/S50usbdevice.new" && grep -q 'insmod /usr/share/horizon/modules/' "$work/S50usbdevice.new" ||
    fail "S50usbdevice: serial port switch not found"
grep -q 'echo "Horizon Dev Air rev-1A" > ${USB_STRINGS_DIR}/product' "$work/S50usbdevice.new" &&
    grep -q 'echo "BPYL" > ${USB_STRINGS_DIR}/manufacturer' "$work/S50usbdevice.new" || fail "S50usbdevice: USB names not found"
echo horizon-air >"$work/hostname"
printf '127.0.0.1\tlocalhost\n127.0.1.1\thorizon-air\n' >"$work/hosts"

# debugfs commands: directories, files, owner root, modes
cmds=$work/debugfs.cmds
{
    for f in /etc/init.d/S50usbdevice /etc/hostname /etc/hosts; do
        src=$work/$(basename "$f"); [ "$f" = /etc/init.d/S50usbdevice ] && src=$work/S50usbdevice.new
        echo "rm $f"; echo "write $src $f"; echo "sif $f uid 0"; echo "sif $f gid 0"
        [ "$f" = /etc/init.d/S50usbdevice ] && echo "sif $f mode 0100755" || echo "sif $f mode 0100644"
    done
    echo "mkdir /usr/share/horizon"
    (cd "$st" && find . -mindepth 1 -type d | sort) | while read -r d; do echo "mkdir /usr/share/horizon/${d#./}"; done
    (cd "$st" && find . -type f | sort) | while read -r f; do
        p=/usr/share/horizon/${f#./}
        echo "write $st/${f#./} $p"
        case "$f" in ./hz_vtx | ./service.sh | ./rp2354_update.sh | ./horizon_pair.sh | ./horizon_check.sh | ./horizon_update.sh | ./spi_speed_test | ./rp2354_spi_update | ./openocd/openocd) echo "sif $p mode 0100755" ;; *) echo "sif $p mode 0100644" ;; esac
    done
    (cd "$st" && find . -mindepth 1 | sort) | while read -r f; do echo "sif /usr/share/horizon/${f#./} uid 0"; echo "sif /usr/share/horizon/${f#./} gid 0"; done
    echo "sif /usr/share/horizon uid 0"; echo "sif /usr/share/horizon gid 0"
    echo "write $here/S99horizon_vtx /etc/init.d/S99horizon_vtx"
    echo "sif /etc/init.d/S99horizon_vtx mode 0100755"
    echo "write $here/luckfox.cfg /etc/luckfox.cfg"
    echo "sif /etc/luckfox.cfg mode 0100644"
    for p in /etc/init.d/S99horizon_vtx /etc/luckfox.cfg; do echo "sif $p uid 0"; echo "sif $p gid 0"; done
} >"$cmds"
debugfs -w -f "$cmds" "$work/rootfs.img" >"$work/debugfs.log" 2>&1
# debugfs echoes every command as "debugfs: ..."; anything else it prints is a complaint (the version banner aside)
if grep -v '^debugfs[: ]\|^Allocated inode: [0-9]*$' "$work/debugfs.log" | grep -q .; then
    grep -v '^debugfs[: ]\|^Allocated inode: [0-9]*$' "$work/debugfs.log" | head; fail "debugfs"
fi
e2fsck -fn "$work/rootfs.img" >/dev/null 2>&1 || fail "root filesystem check after writing"
# read back what matters
for p in /usr/share/horizon/hz_vtx /usr/share/horizon/rp2354.elf /usr/share/horizon/openocd/openocd /etc/init.d/S99horizon_vtx; do
    debugfs -R "dump $p $work/check" "$work/rootfs.img" >/dev/null 2>&1
    src=$st/${p#/usr/share/horizon/}; [ "$p" = /etc/init.d/S99horizon_vtx ] && src=$here/S99horizon_vtx
    cmp -s "$work/check" "$src" || fail "read-back of $p differs"
done

# The update bundle: everything but what only changes with a new card (OpenOCD, the kernel modules).
echo "EVENT update bundle"
upd=$work/update
mkdir -p "$upd/share" "$upd/etc/init.d"
(cd "$st" && tar -cf - --exclude ./openocd --exclude ./modules .) | tar -xf - -C "$upd/share"
cp "$here/S99horizon_vtx" "$upd/etc/init.d/S99horizon_vtx"
cp "$here/luckfox.cfg" "$upd/etc/luckfox.cfg"
chmod 755 "$upd/etc/init.d/S99horizon_vtx"
for f in hz_vtx service.sh rp2354_update.sh horizon_pair.sh horizon_check.sh horizon_update.sh spi_speed_test rp2354_spi_update; do
    chmod 755 "$upd/share/$f"
done
(cd "$upd" && tar -cf - --owner=0 --group=0 --sort=name share etc) | xz -c --lzma2=preset=6,dict=1MiB >"$out/$name.update.tar.xz"
# (a 1 MiB dictionary: unpacking it on the Air needs ~1 MB of RAM instead of ~9 MB with xz's default)
usha=$(sha256sum "$out/$name.update.tar.xz" | cut -d' ' -f1)
usize=$(stat -c %s "$out/$name.update.tar.xz")

echo "EVENT card image"
raw=$work/$name.img
: >"$raw"
# "#rootfs.img 0x190640:0x320C8000 ..." -> image file, offset in 512-byte sectors (the SDK's own SD layout)
grep '^#' "$img/sd_update.txt" | while read -r tag at _rest; do
    f=${tag#\#}
    sector=$((${at%%:*}))
    src=$img/$f; [ "$f" = rootfs.img ] && src=$work/rootfs.img
    test -f "$src" || fail "sd_update.txt names $f, not in $img"
    dd if="$src" of="$raw" bs=512 seek="$sector" conv=notrunc,sparse status=none
    echo "  $f at sector $sector ($(stat -c %s "$src") bytes)"
done
card_bytes=$(stat -c %s "$raw")

echo "EVENT compress"
xz -T0 -6 -c "$raw" >"$out/$name.img.xz"
sha=$(sha256sum "$out/$name.img.xz" | cut -d' ' -f1)
echo "$sha  $name.img.xz" >"$out/$name.img.xz.sha256"
size=$(stat -c %s "$out/$name.img.xz")
pwr=$(sed -n 's/^HZ_ARGS=.*--tx-power \(-\{0,1\}[0-9]*\).*/\1/p' "$repo/luckfox_image/horizon_vtx.conf")
# the boot frequency is compiled into the firmware (cmake -DHZ_FREQ_HZ=..., default 868 MHz)
hz=$(sed -n 's/^HZ_FREQ_HZ:STRING=//p' "$(dirname "$fw")/CMakeCache.txt" 2>/dev/null)
freq_mhz=$(awk -v h="${hz:-868000000}" 'BEGIN { print h / 1e6 }')
cat >"$out/manifest.json" <<EOF
{"file": "$name.img.xz", "version": "$version", "git": "$git", "size": $size, "card_bytes": $card_bytes,
 "sha256": "$sha", "board": "Horizon Dev Air rev-1A", "product": "Horizon Dev Air rev-1A",
 "update": {"file": "$name.update.tar.xz", "sha256": "$usha", "size": $usize},
 "rp2354": "$(sed -n 's/^rp2354=//p' "$st/VERSION")", "hz_vtx": "$(sed -n 's/^hz_vtx=//p' "$st/VERSION")", "freq_mhz": $freq_mhz, "power_dbm": $(awk -v p="${pwr:--19}" 'BEGIN { print p / 2 }')}
EOF
echo "EVENT image $out/$name.img.xz ($size bytes, $card_bytes on the card)"
echo "RESULT PASS"
