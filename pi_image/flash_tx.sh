#!/bin/sh
# Flash the TX RP2350 with the Pi transmitter firmware (hat_tx) over USB, with picotool (no BOOTSEL button needed:
# the running firmware's USB reset interface is used; a board in BOOTSEL mode works too).
#   flash_tx.sh [--ser <serial no.>] [UF2]
#     UF2           the HAT firmware: default radio_firmware/build/hat_tx.uf2 (your own build of radio_firmware).
#                   The setup site (https://horizon.bpyl.si, Air unit, Raspberry Pi 5 + Horizon HAT) flashes the HAT
#                   from a browser without this script, and offers the same firmware for download.
#                   hat_tx boots at 22 dBm, which needs a proper supply (HAT / 5 V header); on a board powered from
#                   the Pi's USB port, build with -DHZ_HAT_TX_POWER=-19 or lower it at run time (hz_vtx --tx-power)
#     --ser         serial number of the TX board (picotool info / ls /dev/serial/by-id); needed when more than one
#                   Pico-type board is plugged in (e.g. receivers on the same machine)
# Stops horizon_vtx while flashing (if it runs) and starts it again afterwards.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
UF2="$HERE/../radio_firmware/build/hat_tx.uf2"
SER=${PICO_SERIAL:-}
while [ $# -gt 0 ]; do
    case "$1" in
        --low-power) echo "--low-power is gone: build radio_firmware with -DHZ_HAT_TX_POWER=-19, or use hz_vtx --tx-power"; exit 2 ;;
        --ser) SER=$2; shift 2 ;;
        -h|--help) sed -n 2,10p "$0"; exit 0 ;;
        *) UF2=$1; shift ;;
    esac
done
[ -f "$UF2" ] || { echo "firmware not found: $UF2"; echo "flash the HAT from https://horizon.bpyl.si (Air unit, Raspberry Pi 5 + Horizon HAT), or pass a .uf2 (downloaded there, or built from radio_firmware)"; exit 1; }
command -v picotool >/dev/null || { echo "picotool not installed (sudo apt install picotool), or copy $UF2 onto the RPI-RP2/RP2350 drive in BOOTSEL mode"; exit 1; }
if [ -z "$SER" ]; then
    n=$(ls /dev/serial/by-id/usb-Raspberry_Pi_Pico_* 2>/dev/null | wc -l)
    if [ "$n" -gt 1 ]; then
        echo "more than one Pico board is plugged in; name the TX board with --ser:"
        ls /dev/serial/by-id/usb-Raspberry_Pi_Pico_* | sed 's/.*Pico_\(.*\)-if00/  \1/'
        exit 1
    fi
fi
SUDO=""; [ "$(id -u)" -eq 0 ] || SUDO=sudo
RUNNING=0
if systemctl is-active --quiet horizon_vtx 2>/dev/null; then RUNNING=1; $SUDO systemctl stop horizon_vtx; fi
$SUDO picotool load -x -f ${SER:+--ser "$SER"} "$UF2" 2>&1 | tr '\r' '\n' | grep -v '%' | grep . | tail -2
[ "$RUNNING" -eq 1 ] && $SUDO systemctl start horizon_vtx
echo "TX RP2350 ${SER:+$SER }flashed with $(basename "$UF2")"
