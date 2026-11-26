#!/bin/sh
# Builds the Horizon Pi image: Raspberry Pi OS Lite (64-bit, pi_image/raspios.pin) with the Pi transmitter
# (hz_vtx and its encoder hz_x265, built from transmitter/ in the image) installed in /opt/horizon and started at boot
# by the horizon_vtx service. For a Raspberry Pi 5 with a camera and the Horizon HAT (TX RP2350 + LR2021 on SPI0.0).
#
#   sudo sh pi_image/build_pi_image.sh OUT_DIR [VERSION]          (Linux or WSL as root; needs qemu-user-static
#                                                                   with binfmt for aarch64, losetup, parted, resize2fs)
#
# The image keeps nothing per unit: the frequency plan, the TX power and the callsign live in the HAT (the setup site's
# Pair saves them there over the HAT's USB port), and the sender uses what the HAT has. Login: user "pi", password
# "raspberry", SSH on (hostname horizon-air); Raspberry Pi Imager's OS customisation can add a user and Wi-Fi.
# Writes OUT_DIR/horizon-pi5-VERSION.img.xz, its .sha256 and manifest.json. The Raspberry Pi OS download and the base
# with the packages installed are cached in CACHE_DIR (default ~/horizon-image/cache): the first build after a new
# release or package list takes ~10 min (emulated apt), later ones a few. Prints RESULT PASS|FAIL.
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

# The base (Raspberry Pi OS grown by grow_mb, packages installed: slow, apt runs emulated) is cached as
# base-<key>.img, keyed by the release, the package list, the grow size and BASE_REV (bump it when the base steps change).
PKGS="libcamera-dev libx265-dev gcc g++ pkg-config libc6-dev"
BASE_REV=1
key=$(printf '%s|%s|%s|%s' "$sum" "$PKGS" "$grow_mb" "$BASE_REV" | sha256sum | cut -c1-16)
based=$cache/base-$key.img

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

# The image's ext4 uses features newer than many hosts' e2fsprogs (orphan_file: e2fsprogs 1.47): its own aarch64
# e2fsck / resize2fs do the work, run by qemu from a read-only mount of a Raspberry Pi OS image ("sys").
e2tool() { qemu-aarch64-static -L "$work/sys" "$work/sys/usr/sbin/$1" "$2" "$3"; }
sys_up() { sysloop=$(losetup -fP -r --show "$1"); mkdir -p "$work/sys"; mount -o ro "${sysloop}p2" "$work/sys"; }
sys_down() { umount "$work/sys"; losetup -d "$sysloop"; sysloop=; }
# the image ($loop) mounted at $root as a chroot: host DNS, no services started
chroot_up() {
    mkdir -p "$root"
    mount "${loop}p2" "$root"
    mount "${loop}p1" "$root/boot/firmware"
    for m in proc sys dev dev/pts run; do mount --bind "/$m" "$root/$m"; done
    cp /etc/resolv.conf "$root/etc/resolv.conf.horizon"
    mv "$root/etc/resolv.conf" "$root/etc/resolv.conf.orig" 2>/dev/null || true
    cp "$root/etc/resolv.conf.horizon" "$root/etc/resolv.conf"
    printf '#!/bin/sh\nexit 101\n' >"$root/usr/sbin/policy-rc.d"; chmod 755 "$root/usr/sbin/policy-rc.d"
}
chroot_down() {
    rm -f "$root/usr/sbin/policy-rc.d" "$root/etc/resolv.conf.horizon"
    mv "$root/etc/resolv.conf.orig" "$root/etc/resolv.conf" 2>/dev/null || true
    sync
    for m in boot/firmware dev/pts dev proc sys run; do umount "$root/$m"; done
    umount "$root"
}

if [ ! -s "$based" ]; then
    echo "EVENT building the base (Raspberry Pi OS + packages, emulated, several minutes; cached as $(basename "$based"))"
    xz -dc "$cache/$base" >"$img"
    cp "$img" "$work/sys.img"; sys_up "$work/sys.img"
    truncate -s +"${grow_mb}M" "$img"
    parted -s "$img" resizepart 2 100%
    loop=$(losetup -fP --show "$img")
    e2tool e2fsck -fy "${loop}p2" >/dev/null 2>&1 || true
    e2tool resize2fs -p "${loop}p2" >/dev/null 2>&1 || fail "resize2fs"
    chroot_up
    chroot "$root" /bin/sh -ec "export DEBIAN_FRONTEND=noninteractive
        apt-get update -qq
        apt-get install -y -qq --no-install-recommends $PKGS >/dev/null
        apt-get clean" || fail "apt in the image"
    chroot_down
    e2tool e2fsck -fy "${loop}p2" >/dev/null 2>&1 || true
    losetup -d "$loop"; loop=
    sys_down; rm -f "$work/sys.img"
    rm -f "$cache"/base-*.img "$cache"/base-*.img.part   # the base of an earlier release or package list
    cp --sparse=always "$img" "$based.part" && mv "$based.part" "$based"
else
    echo "EVENT using the cached base $(basename "$based")"
    cp --sparse=always "$based" "$img"
fi
sys_up "$based"
loop=$(losetup -fP --show "$img")
chroot_up

echo "EVENT installing the Horizon transmitter in /opt/horizon"
h=$root/opt/horizon
# hz_vtx and hz_x265, built in the image (emulated): the sources in a scratch folder, only the programs stay
mkdir -p "$h" "$root/tmp/hz-src"
cp -r "$repo/transmitter" "$repo/format" "$root/tmp/hz-src/"
rm -rf "$root/tmp/hz-src/transmitter/build" "$root/tmp/hz-src/transmitter/test"
chroot "$root" env HZ_GIT="$ver" sh /tmp/hz-src/transmitter/build_pi.sh /tmp/hz-src/out >/dev/null || fail "hz_vtx build"
install -m 755 "$root/tmp/hz-src/out/hz_vtx" "$root/tmp/hz-src/out/hz_x265" "$repo/pi_image/tx_power.sh" "$repo/pi_image/tx_source.sh" "$h/"
rm -rf "$root/tmp/hz-src"
echo "$ver" >"$h/VERSION"
# both programs must start in the image (a missing library only shows on the Pi)
chroot "$root" /opt/horizon/hz_vtx --version | grep -q "git=$ver" || fail "hz_vtx does not run in the image"
chroot "$root" /opt/horizon/hz_x265 2>&1 | grep -q usage || fail "hz_x265 does not run in the image"
echo "EVENT hz_vtx and hz_x265 built and run"
# the service user: camera, SPI, the HAT's USB serial port
chroot "$root" /bin/sh -ec 'id horizon >/dev/null 2>&1 || useradd --system --no-create-home --home-dir /opt/horizon --shell /usr/sbin/nologin horizon
    for g in video spi gpio dialout; do getent group $g >/dev/null && usermod -aG $g horizon; done'
# Login: user "pi", password "raspberry", SSH on. The first-boot "create a user" dialog is off, and cloud-init must not
# lock the password at first boot. Imager's OS customisation still adds the user it is given.
chroot "$root" /bin/sh -ec 'usermod -s /bin/bash -d /home/pi -m pi 2>/dev/null || usermod -s /bin/bash pi
    mkdir -p /home/pi && chown pi:pi /home/pi
    for g in adm dialout cdrom audio users sudo video plugdev input gpio spi i2c netdev render; do
        getent group $g >/dev/null && usermod -aG $g pi; done
    echo pi:raspberry | chpasswd
    printf "pi ALL=(ALL) NOPASSWD:ALL\n" >/etc/sudoers.d/010_pi-nopasswd; chmod 440 /etc/sudoers.d/010_pi-nopasswd
    systemctl disable userconfig.service >/dev/null 2>&1 || true
    systemctl enable ssh.service >/dev/null 2>&1' || fail "pi user / ssh"
# Horizon maintainer key (pi_image/authorized_keys): a public key that can SSH into every unit written from this image.
if [ -s "$here/authorized_keys" ]; then
    mkdir -p "$root/home/pi/.ssh"
    cp "$here/authorized_keys" "$root/home/pi/.ssh/authorized_keys"
    chroot "$root" /bin/sh -ec 'chown -R pi:pi /home/pi/.ssh; chmod 700 /home/pi/.ssh; chmod 600 /home/pi/.ssh/authorized_keys'
    echo "EVENT installed the maintainer key ($(wc -l <"$here/authorized_keys") key(s)) for pi"
fi
cat >"$root/etc/cloud/cloud.cfg.d/99_zz-horizon.cfg" <<'EOF'
# Horizon: the image's user "pi" (password "raspberry") keeps its password: the base config would lock it at first boot.
system_info:
  distro: raspberry-pi-os
  default_user:
    name: pi
    lock_passwd: False
    gecos: Raspberry Pi OS
    groups: [adm, dialout, cdrom, audio, users, sudo, video, games, plugdev, input, gpio, spi, i2c, netdev, render, lpadmin]
    sudo: ["ALL=(ALL) NOPASSWD:ALL"]
    shell: /bin/bash
  network:
    renderers: ['netplan', 'network-manager']
    activators: ['netplan', 'network-manager']
  disable_fallback_netcfg: true
  ntp_client: 'systemd-timesyncd'
  ssh_svcname: ssh
ssh_pwauth: true
EOF
# (a camera unit usually has no network: the service must not wait for one)
cp "$repo/pi_image/horizon_vtx.service" "$root/etc/systemd/system/horizon_vtx.service"
cp "$repo/pi_image/horizon_vtx.default" "$root/etc/default/horizon_vtx"
chroot "$root" systemctl enable horizon_vtx.service >/dev/null 2>&1 || fail "enable horizon_vtx"
# SPI0 to the HAT's RP2350; the camera is auto-detected (the image's default)
cfg=$root/boot/firmware/config.txt
grep -q '^dtparam=spi=on' "$cfg" || sed -i 's/^#\{0,1\}dtparam=spi=.*/dtparam=spi=on/' "$cfg"
grep -q '^dtparam=spi=on' "$cfg" || echo 'dtparam=spi=on' >>"$cfg"
echo horizon-air >"$root/etc/hostname"
sed -i 's/127\.0\.1\.1.*/127.0.1.1\thorizon-air/' "$root/etc/hosts"

card=$(stat -c %s "$img")
used=$(df -B1 --output=used "$root" | tail -1)
chroot_down
e2tool e2fsck -fy "${loop}p2" >/dev/null 2>&1 || true
losetup -d "$loop"; loop=
sys_down

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
