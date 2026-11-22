#!/bin/sh
# TX power from the Pi while live_sender.py runs (as the Luckfox's `service.sh power`):
#   tx_power.sh                    show the current value (as the TX RP2350 reports it)
#   tx_power.sh <half_dbm>         e.g. -19 (-9.5 dBm, minimum) .. 44 (22 dBm, maximum)
#   tx_power.sh <x>dbm             e.g. 10dbm, -9.5dbm
#   tx_power.sh <value> save       also keep it for the next starts (--tx-power in /etc/default/lora-vlink-tx, sudo)
# The RP2350 applies it between two packets and announces it over the air (downlink only), so the receivers show it.
# The receiver can still change the power over the air. LORA_TX_CTL overrides the control FIFO path.
CTL=${LORA_TX_CTL:-}
if [ -z "$CTL" ]; then
    for c in /run/lora-vlink-tx/ctl /tmp/lora-vlink-tx.ctl; do [ -p "$c" ] && { CTL=$c; break; }; done
fi
[ -n "$CTL" ] && [ -p "$CTL" ] || { echo "live_sender.py is not running (no control FIFO /run/lora-vlink-tx/ctl or /tmp/lora-vlink-tx.ctl)"; exit 1; }
show() { awk -v p="$1" 'BEGIN { printf "%.1f", p / 2 }'; }
if [ $# -eq 0 ]; then
    p=$(cat "$CTL.pwr" 2>/dev/null)
    [ -n "$p" ] || { echo "TX power unknown (no status from the RP2350 yet)"; exit 1; }
    echo "TX power: $p (0.5 dBm units) = $(show "$p") dBm"; exit 0
fi
v=$1
case "$v" in
    *dbm|*dBm|*DBM)
        d=${v%???}
        half=$(awk -v d="$d" 'BEGIN { h = d * 2; if (h != int(h) || d == "") exit 1; printf "%d", h }') ||
            { echo "power must be a multiple of 0.5 dBm"; exit 2; } ;;
    *) half=$v ;;
esac
case "$half" in ''|-|*[!0-9-]*|?*-*) echo "power: not a number: $v"; exit 2;; esac
if [ "$half" -lt -19 ] || [ "$half" -gt 44 ]; then
    echo "power must be -19..44 (0.5 dBm units) = -9.5..22 dBm"; exit 2
fi
echo "pwr $half" > "$CTL"
n=0
while [ "$(cat "$CTL.pwr" 2>/dev/null)" != "$half" ]; do
    n=$((n + 1)); [ "$n" -lt 50 ] || { echo "TX power $half NOT confirmed by the RP2350"; exit 1; }
    sleep 0.1
done
echo "TX power set to $half = $(show "$half") dBm"
if [ "$2" = "save" ]; then
    f=/etc/default/lora-vlink-tx
    [ -f "$f" ] || echo 'LORA_TX_ARGS=""' | sudo tee "$f" >/dev/null
    sudo sed -i -e '/^LORA_TX_ARGS=/s/ *--tx-power -\{0,1\}[0-9]*//' \
        -e "s/^LORA_TX_ARGS=\"\(.*\)\"/LORA_TX_ARGS=\"\1 --tx-power $half\"/" -e 's/^LORA_TX_ARGS=" /LORA_TX_ARGS="/' "$f" &&
        echo "saved in $f (applied at every start)"
fi
