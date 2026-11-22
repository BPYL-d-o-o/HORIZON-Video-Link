#!/bin/sh
# Flash the TX RP2350 with the Pi transmitter firmware (vlive_tx) over USB, with picotool (no BOOTSEL button needed:
# the running firmware's USB reset interface is used; a board in BOOTSEL mode works too).
#   flash_tx.sh [--low-power] [--ser <serial no.>] [UF2]
#     --low-power   boot at -9.5 dBm (prebuilt/firmware/vlive_tx_lowpower.uf2) for a board powered from the Pi's USB
#                   port; the default vlive_tx.uf2 boots at 22 dBm, which needs a proper supply (HAT / 5 V header)
#     --ser         serial number of the TX board (picotool info / ls /dev/serial/by-id); needed when more than one
#                   Pico-type board is plugged in (e.g. receivers on the same machine)
# Stops lora-vlink-tx while flashing (if it runs) and starts it again afterwards.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
UF2="$HERE/../prebuilt/firmware/vlive_tx.uf2"
SER=${PICO_SERIAL:-}
while [ $# -gt 0 ]; do
    case "$1" in
        --low-power) UF2="$HERE/../prebuilt/firmware/vlive_tx_lowpower.uf2"; shift ;;
        --ser) SER=$2; shift 2 ;;
        -h|--help) sed -n 2,10p "$0"; exit 0 ;;
        *) UF2=$1; shift ;;
    esac
done
[ -f "$UF2" ] || { echo "firmware not found: $UF2"; exit 1; }
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
if systemctl is-active --quiet lora-vlink-tx 2>/dev/null; then RUNNING=1; $SUDO systemctl stop lora-vlink-tx; fi
$SUDO picotool load -x -f ${SER:+--ser "$SER"} "$UF2" 2>&1 | tr '\r' '\n' | grep -v '%' | grep . | tail -2
[ "$RUNNING" -eq 1 ] && $SUDO systemctl start lora-vlink-tx
echo "TX RP2350 ${SER:+$SER }flashed with $(basename "$UF2")"
