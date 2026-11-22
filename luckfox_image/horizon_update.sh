#!/bin/sh
# Horizon Dev Air: installs an update bundle (horizon-dev-air-VERSION.update.tar.xz, from luckfox_image/build_image.sh)
# without writing the card again. The setup site sends it over the USB serial port (horizon_pair.sh "U"); it also
# works by hand:  sh /usr/share/horizon/horizon_update.sh BUNDLE
#
# The bundle carries share/ (the files of /usr/share/horizon except OpenOCD and the kernel modules, which only change
# with a new card image) and etc/ (init.d/S99horizon_vtx, luckfox.cfg). Steps:
#   1. unpack and check it (an Air bundle with programs, scripts and radio controller firmware)
#   2. keep the installed files in /usr/share/horizon.prev, stop the transmitter
#   3. install: each file written beside the old one and renamed over it (a running script keeps its old copy)
#   4. the radio controller: flashed over SWD when its firmware changed (rp2354_update.sh flash, which restarts the
#      transmitter), else the transmitter restarts
#   5. the transmitter must run and identify within 60 s; otherwise the previous files come back (and the previous
#      radio controller firmware is flashed again)
# The settings (/userdata/horizon_vtx/horizon_vtx.conf: callsign, frequency, power) are kept.
#
# Prints EVENT lines and a final "RESULT PASS version=<image> reboot=0|1" (reboot=1: luckfox.cfg changed, the board
# restarts itself 3 s later) or "RESULT FAIL <why>".
set -u
export PATH=/sbin:/usr/sbin:/bin:/usr/bin
bundle=${1:?usage: horizon_update.sh BUNDLE.tar.xz}
share=/usr/share/horizon
prev=/usr/share/horizon.prev
dir=/userdata/horizon_vtx
# on the card, not in /tmp: /tmp is RAM, and the Air has ~15 MB free with the camera running (an update in RAM once
# starved it until everything stalled)
work=/userdata/horizon_update.work
fail() { echo "RESULT FAIL $*"; logger -t horizon_update "FAIL $*"; rm -rf "$work"; exit 1; }
say() { echo "EVENT $*"; logger -t horizon_update "$*"; }

# 1. unpack and check
rm -rf "$work"; mkdir -p "$work"
unxz -c "$bundle" | tar -xf - -C "$work" 2>/dev/null || fail "bundle: not an xz-compressed tar"
for f in share/VERSION share/hz_vtx share/rp2354.elf share/service.sh share/rp2354_update.sh share/horizon_pair.sh \
         share/horizon_update.sh etc/init.d/S99horizon_vtx etc/luckfox.cfg; do
    test -s "$work/$f" || fail "bundle: $f missing"
done
new=$(sed -n 's/^image=//p' "$work/share/VERSION")
old=$(sed -n 's/^image=//p' "$share/VERSION" 2>/dev/null)
case "$new" in horizon-dev-air-*) ;; *) fail "bundle: not a Horizon Dev Air bundle (image=$new)" ;; esac
say "update $old -> $new"

# 2. keep the installed files (without OpenOCD and the modules, which an update does not change), stop the transmitter
rm -rf "$prev"
mkdir -p "$prev"
(cd "$share" && tar -cf - --exclude ./openocd --exclude ./modules .) | tar -xf - -C "$prev" || fail "no room to keep the installed files"
cp -p /etc/init.d/S99horizon_vtx "$prev/S99horizon_vtx.etc" 2>/dev/null
cp -p /etc/luckfox.cfg "$prev/luckfox.cfg.etc" 2>/dev/null
sh "$dir/service.sh" stop >/dev/null 2>&1

put() {  # src dst: write beside, then rename over (atomic; a running script keeps reading its old file)
    cp -p "$1" "$2.new" && mv "$2.new" "$2"
}
install_from() {  # root (with share/ and etc/): files into place, then the programs into /userdata (S99 install)
    (cd "$1/share" && find . -type f) | while read -r f; do
        mkdir -p "$(dirname "$share/${f#./}")"
        put "$1/share/${f#./}" "$share/${f#./}" || return 1
    done
    put "$1/etc/init.d/S99horizon_vtx" /etc/init.d/S99horizon_vtx || return 1
    chmod 755 /etc/init.d/S99horizon_vtx
    sh /etc/init.d/S99horizon_vtx install
}
reboot=0
cmp -s "$work/etc/luckfox.cfg" /etc/luckfox.cfg || reboot=1

restore() {  # back to the files in $prev, then fail with $1
    say "restoring $old"
    mkdir -p "$work/restore/share" "$work/restore/etc/init.d"
    # (OpenOCD and the modules are not part of an update: they stay where they are)
    (cd "$prev" && tar -cf - --exclude ./S99horizon_vtx.etc --exclude ./luckfox.cfg.etc --exclude ./openocd --exclude ./modules .) |
        tar -xf - -C "$work/restore/share"
    cp -p "$prev/S99horizon_vtx.etc" "$work/restore/etc/init.d/S99horizon_vtx"
    install_from "$work/restore"
    if ! cmp -s "$prev/rp2354.elf" "$work/share/rp2354.elf"; then sh "$share/rp2354_update.sh" flash >/dev/null 2>&1
    else sh "$dir/service.sh" start >/dev/null 2>&1; fi
    fail "$1 (the previous version $old is back)"
}

# 3. install
say "installing"
install_from "$work" || restore "install"
if [ "$reboot" = 1 ]; then put "$work/etc/luckfox.cfg" /etc/luckfox.cfg; fi
sync

# 4. the radio controller, and the transmitter (it writes /run/hz_vtx.id once the radio controller has identified)
rm -f /run/hz_vtx.id
if cmp -s "$work/share/rp2354.elf" "$prev/rp2354.elf" && [ "$(cat "$dir/rp2354.flashed" 2>/dev/null)" = "$(md5sum <"$share/rp2354.elf" | cut -d' ' -f1)" ]; then
    say "radio controller firmware unchanged"
    sh "$dir/service.sh" start >/dev/null 2>&1
else
    say "flashing the radio controller (about 20 s)"
    out=$(sh "$share/rp2354_update.sh" flash 2>&1)
    echo "$out" | grep -q '^RESULT PASS' || restore "radio controller flash failed: $(echo "$out" | grep -m1 -E 'ERROR|FAIL')"
fi

# 5. the transmitter runs and its radio controller identifies (not by the syslog: it rotates)
say "waiting for the transmitter"
i=0
while [ "$i" -lt 60 ]; do
    grep -q '^rp2350_board=' /run/hz_vtx.id 2>/dev/null && break
    sleep 1; i=$((i + 1))
done
[ "$i" -lt 60 ] || restore "the transmitter did not start"
# the LED and its 30 s self-check: the new script
pid=$(ps | grep '[h]orizon_led.sh' | awk '{ print $1 }'); [ -n "$pid" ] && kill $pid 2>/dev/null
setsid sh -c "sh $share/horizon_led.sh 2>&1 | logger -t horizon_led" </dev/null >/dev/null 2>&1 &
rm -rf "$work"
sync
say "installed $new"
echo "RESULT PASS version=$new reboot=$reboot"
if [ "$reboot" = 1 ]; then ( sleep 3; reboot ) </dev/null >/dev/null 2>&1 & fi
