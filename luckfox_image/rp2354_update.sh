#!/bin/sh
# On the Horizon Dev Air: keeps the RP2354A radio controller on the firmware this image carries, then starts the
# transmitter. The RP2354A has no USB on this board; it is flashed over SWD by OpenOCD on the Luckfox
# (SWCLK = GPIO1_D2, SWDIO = GPIO1_D3; I2C3_M1 must stay off in /etc/luckfox.cfg).
#
#   sh rp2354_update.sh boot     at boot (S99horizon_vtx): flash if the image's firmware is not the one last flashed,
#                                start the service, and flash once more if the RP2354A then does not answer
#   sh rp2354_update.sh flash [ELF]
#                                flash now (stops and restarts the service); ELF: another firmware (a test build,
#                                luckfox_vtx/flash_rp2354.sh). It stays until the next power-up or update, which
#                                put the image's own firmware back.
#   sh rp2354_update.sh check    print what would be done
#
# Prints EVENT lines and RESULT PASS|FAIL (to syslog, tag horizon_vtx, when run at boot).
set -u
share=/usr/share/horizon
dir=/userdata/horizon_vtx
elf=$share/rp2354.elf
if [ "${1:-}" = flash ] && [ -n "${2:-}" ]; then
    elf=$2
    [ -s "$elf" ] || { echo "FATAL no firmware $elf"; echo "RESULT FAIL"; exit 1; }
fi
mark=$dir/rp2354.flashed          # md5 of the firmware last flashed successfully
export PATH=/sbin:/usr/sbin:/bin:/usr/bin

want=$(md5sum <"$elf" | cut -d' ' -f1)
have=$(cat "$mark" 2>/dev/null)

flash() {
    chip=1   # gpiochip of GPIO bank 1 (sysfs base 32), for libgpiod
    for d in /sys/class/gpio/gpiochip*; do
        [ "$(cat "$d/base" 2>/dev/null)" = 32 ] || continue
        for s in "$d"/device/gpiochip*; do [ -e "$s" ] && chip=${s##*gpiochip}; done
    done
    cat >/tmp/rp2354.cfg <<EOF
adapter driver linuxgpiod
adapter gpio swclk 26 -chip $chip
adapter gpio swdio 27 -chip $chip
transport select swd
adapter speed 1000
reset_config none
source [find target/rp2350.cfg]
EOF
    echo "EVENT rp2354_flash start firmware=$want"
    out=$("$share/openocd/openocd" -s "$share/openocd/scripts" -f /tmp/rp2354.cfg -c "program $elf verify reset exit" 2>&1)
    rc=$?
    rm -f /tmp/rp2354.cfg
    echo "$out" | grep -E "Programming|[Vv]erified|Error|DPIDR" | sed 's/^/  /'
    if [ "$rc" = 0 ] && echo "$out" | grep -qi verified; then
        echo "$want" >"$mark" && sync
        echo "EVENT rp2354_flash ok"
        return 0
    fi
    echo "ERROR rp2354_flash failed rc=$rc"
    return 1
}

answers() {   # after the service started: did hz_vtx identify the RP2354A?
    n=0
    while [ ! -f /run/hz_vtx.id ] && [ "$n" -lt 30 ]; do sleep 1; n=$((n + 1)); done
    [ -f /run/hz_vtx.id ] && ! grep -q '^rp2350_board=unknown' /run/hz_vtx.id
}

case "${1:-check}" in
check)
    if [ "$want" = "$have" ]; then echo "EVENT rp2354 up to date ($want)"; else echo "EVENT rp2354 needs flashing (image $want, last flashed ${have:-never})"; fi
    ;;
flash)
    sh "$dir/service.sh" stop >/dev/null 2>&1
    flash; ok=$?
    sh "$dir/service.sh" start
    [ "$ok" = 0 ] && echo "RESULT PASS" || { echo "RESULT FAIL"; exit 1; }
    ;;
boot)
    flashed=0
    if [ "$want" != "$have" ]; then
        flash && flashed=1
    fi
    sh "$dir/service.sh" start
    if ! answers; then
        if [ "$flashed" = 0 ]; then
            echo "EVENT rp2354 does not answer: flashing it"
            sh "$dir/service.sh" stop >/dev/null 2>&1
            flash
            sh "$dir/service.sh" start
            answers && { echo "RESULT PASS"; exit 0; }
        fi
        echo "ERROR rp2354 does not answer (radio board, SWD wiring or power)"
        echo "RESULT FAIL"
        exit 1
    fi
    echo "EVENT transmitter running: $(tr '\n' ' ' </run/hz_vtx.id)"
    echo "RESULT PASS"
    ;;
*)
    echo "usage: $0 boot|flash [ELF]|check"; exit 2
    ;;
esac
