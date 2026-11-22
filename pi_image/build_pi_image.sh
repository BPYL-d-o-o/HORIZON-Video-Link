#!/bin/sh
# Builds the Horizon Pi image: Raspberry Pi OS Lite (64-bit, pi_image/raspios.pin) with the Pi transmitter
# (pi5_vtx/live_sender.py, its x265 frame encoder, swfec.py) installed in /opt/horizon and started at boot by the
# lora-vlink-tx service. For a Raspberry Pi 5 with a camera and the Horizon HAT (TX RP2350 + LR2021 on SPI0.0).
#
#   sudo sh pi_image/build_pi_image.sh OUT_DIR [VERSION]          (Linux or WSL as root; needs qemu-user-static
#                                                                   with binfmt for aarch64, losetup, parted, resize2fs)
#
# The image keeps nothing per unit: the frequency plan, the TX power and the callsign live in the HAT (the setup site's
# Pair saves them there over the HAT's USB port), and the sender uses what the HAT has. No login is set up: the unit
# works without one; Raspberry Pi Imager's OS customisation can add a user, Wi-Fi and SSH when writing the card.
# Writes OUT_DIR/horizon-pi5-VERSION.img.xz, its .sha256 and manifest.json. Downloads are cached in CACHE_DIR
# (default ~/horizon-image/cache). Prints RESULT PASS|FAIL.
set -eu
here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repo=$(dirname "$here")
out=${1:?usage: build_pi_image.sh OUT_DIR [VERSION]}
ver=${2:-$(git -C "$repo" rev-parse --short=8 HEAD 2>/dev/null || echo dev)}
cache=${CACHE_DIR:-$HOME/horizon-image/cache}
grow_mb=${GROW_MB:-1024}   # room for the packages (the card's first boot grows the file system to the whole card)
fail() { echo "FATAL $*"; echo "RESULT FAIL"; exit 1; }
[ "$(id -u)" = 0 ] || fail "run as root (sudo; WSL: wsl -u root)"
for t in losetup parted resize2fs e2fsck xz curl sha256sum chroot; do command -v $t >/dev/null || fail "$t missing"; done
[ -e /proc/sys/fs/binfmt_misc/qemu-aarch64 ] || fail "no aarch64 binfmt (apt install qemu-user-static binfmt-support)"

url=$(sed -n 's/^url=//p' "$here/raspios.pin"); sum=$(sed -n 's/^sha256=//p' "$here/raspios.pin")
base=$(basename "$url")
mkdir -p "$cache" "$out"
if ! echo "$sum  $cache/$base" | sha256sum -c --status 2>/dev/null; then
    echo "EVENT downloading $base"
    curl -fL --retry 3 -o "$cache/$base.part" "$url" || fail "download $url"
    mv "$cache/$base.part" "$cache/$base"
    echo "$sum  $cache/$base" | sha256sum -c --status || fail "$base: SHA-256 does not match raspios.pin"
fi

work=$(mktemp -d)
img=$work/horizon.img
root=$work/root
loop= sysloop=
cleanup() {
    set +e
    for m in boot/firmware dev/pts dev proc sys run; do mountpoint -q "$root/$m" && umount -l "$root/$m"; done
    mountpoint -q "$root" && umount -l "$root"
    mountpoint -q "$work/sys" && umount -l "$work/sys"
    [ -n "$loop" ] && losetup -d "$loop"
    [ -n "$sysloop" ] && losetup -d "$sysloop"
    rm -rf "$work"
}
trap cleanup EXIT INT TERM

echo "EVENT unpacking and growing the image by $grow_mb MB"
xz -dc "$cache/$base" >"$img"
# The image's ext4 uses features newer than many hosts' e2fsprogs (orphan_file: e2fsprogs 1.47): its own aarch64
# e2fsck / resize2fs do the resize, run by qemu from a read-only copy of the base image.
cp "$img" "$work/sys.img"
sysloop=$(losetup -fP -r --show "$work/sys.img")
mkdir -p "$work/sys"
mount -o ro "${sysloop}p2" "$work/sys"
e2tool() { qemu-aarch64-static -L "$work/sys" "$work/sys/usr/sbin/$1" "$2" "$3"; }
truncate -s +"${grow_mb}M" "$img"
parted -s "$img" resizepart 2 100%
loop=$(losetup -fP --show "$img")
e2tool e2fsck -fy "${loop}p2" >/dev/null 2>&1 || true
e2tool resize2fs -p "${loop}p2" >/dev/null 2>&1 || fail "resize2fs"
mkdir -p "$root"
mount "${loop}p2" "$root"
mount "${loop}p1" "$root/boot/firmware"
for m in proc sys dev dev/pts run; do mount --bind "/$m" "$root/$m"; done
cp /etc/resolv.conf "$root/etc/resolv.conf.horizon"
mv "$root/etc/resolv.conf" "$root/etc/resolv.conf.orig" 2>/dev/null || true
cp "$root/etc/resolv.conf.horizon" "$root/etc/resolv.conf"
printf '#!/bin/sh\nexit 101\n' >"$root/usr/sbin/policy-rc.d"; chmod 755 "$root/usr/sbin/policy-rc.d"   # no services start in the chroot

echo "EVENT installing packages (emulated, several minutes)"
chroot "$root" /bin/sh -ec 'export DEBIAN_FRONTEND=noninteractive
    apt-get update -qq
    apt-get install -y -qq --no-install-recommends python3-picamera2 python3-numpy python3-serial libx265-dev gcc libc6-dev >/dev/null
    apt-get clean' || fail "apt in the image"

echo "EVENT installing the Horizon transmitter in /opt/horizon"
h=$root/opt/horizon
mkdir -p "$h/pi5_vtx"
# the sender and the modules it imports: swtx (packets), swfec (format, CRC, RS, callsign), blockfec (block FEC parity)
for f in live_sender.py sender.py swtx.py swfec.py blockfec.py frame_enc.c tx_power.sh; do cp "$repo/pi5_vtx/$f" "$h/pi5_vtx/"; done
echo "$ver" >"$h/VERSION"
chroot "$root" gcc -O2 -o /opt/horizon/pi5_vtx/frame_enc /opt/horizon/pi5_vtx/frame_enc.c -lx265 -lm || fail "frame_enc"
chmod 755 "$h/pi5_vtx/tx_power.sh"
# every module the sender needs must import in the image (a missing one only shows when its code first runs)
chroot "$root" /usr/bin/python3 -c 'import sys; sys.path[:0] = ["/opt/horizon/pi5_vtx"]
import swtx, sender, blockfec, swfec, live_sender
s = swtx.Scheduler(fec=(8, 4, 1)); s.push(bytes(3000))
for _ in range(20): s.next_record()
print("sender modules ok")' || fail "the sender's modules do not import in the image"
# the service user: camera, SPI, the HAT's USB serial port
chroot "$root" /bin/sh -ec 'id horizon >/dev/null 2>&1 || useradd --system --no-create-home --home-dir /opt/horizon --shell /usr/sbin/nologin horizon
    for g in video spi gpio dialout; do getent group $g >/dev/null && usermod -aG $g horizon; done'
# (a camera unit usually has no network: the service must not wait for one)
sed -e 's|^User=.*|User=horizon|' -e 's|/home/test/pi5_vtx|/opt/horizon/pi5_vtx|g' -e '/network-online/d' \
    "$repo/pi5_vtx/lora-vlink-tx.service" >"$root/etc/systemd/system/lora-vlink-tx.service"
cat >"$root/etc/default/lora-vlink-tx" <<'EOF'
# Extra options for /opt/horizon/pi5_vtx/live_sender.py (python3 live_sender.py --help). Empty: the camera at
# 576x320, block FEC 8+4, and the frequency plan, TX power and callsign the HAT keeps (the setup site's Pair).
LORA_TX_ARGS="--link spi"
EOF
chroot "$root" systemctl enable lora-vlink-tx.service >/dev/null 2>&1 || fail "enable lora-vlink-tx"
# SPI0 to the HAT's RP2350; the camera is auto-detected (the image's default)
cfg=$root/boot/firmware/config.txt
grep -q '^dtparam=spi=on' "$cfg" || sed -i 's/^#\{0,1\}dtparam=spi=.*/dtparam=spi=on/' "$cfg"
grep -q '^dtparam=spi=on' "$cfg" || echo 'dtparam=spi=on' >>"$cfg"
echo horizon-air >"$root/etc/hostname"
sed -i 's/127\.0\.1\.1.*/127.0.1.1\thorizon-air/' "$root/etc/hosts"

rm -f "$root/usr/sbin/policy-rc.d" "$root/etc/resolv.conf.horizon"
mv "$root/etc/resolv.conf.orig" "$root/etc/resolv.conf" 2>/dev/null || true
card=$(stat -c %s "$img")
used=$(df -B1 --output=used "$root" | tail -1)
sync
for m in boot/firmware dev/pts dev proc sys run; do umount "$root/$m"; done
umount "$root"
e2tool e2fsck -fy "${loop}p2" >/dev/null 2>&1 || true
losetup -d "$loop"; loop=
umount "$work/sys"; losetup -d "$sysloop"; sysloop=

file=horizon-pi5-$ver.img.xz
echo "EVENT compressing $file"
xz -T0 -6 -c "$img" >"$out/$file.part" && mv "$out/$file.part" "$out/$file"
sha=$(sha256sum "$out/$file" | cut -d' ' -f1)
echo "$sha  $file" >"$out/$file.sha256"
cat >"$out/manifest.json" <<EOF
{"file": "$file", "size": $(stat -c %s "$out/$file"), "sha256": "$sha", "version": "$ver", "card_bytes": $card,
 "base": "${base%.img.xz}", "product": "Horizon Pi 5 Air", "used_bytes": $used}
EOF
echo "EVENT image $out/$file ($(( $(stat -c %s "$out/$file") / 1000000 )) MB, card $(( card / 1000000 )) MB)"
echo "RESULT PASS"
