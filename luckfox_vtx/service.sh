#!/bin/sh
# Horizon video link transmitter service on the Luckfox Pico: keeps hz_vtx running (restarts it if it exits).
#   sh /userdata/horizon_vtx/service.sh start|stop|restart|status|id|log|power|run
# Started at boot by /etc/init.d/S99horizon_vtx. Settings: /userdata/horizon_vtx/horizon_vtx.conf (HZ_ARGS).
# Output goes to syslog (tag horizon_vtx): `service.sh log` shows it.
set -u
# At boot init runs this with a bare environment (no /etc/profile): set the library path here, or hz_vtx fails
# with "can't load library 'librockit.so'" (Rockit / rkaiq live in /oem/usr/lib) until restarted from a login shell.
export PATH=/sbin:/usr/sbin:/bin:/usr/bin:/oem/usr/bin
export LD_LIBRARY_PATH=/oem/usr/lib:/oem/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
dir=/userdata/horizon_vtx
pidfile=/run/horizon_vtx.pid

running() {
    [ -f "$pidfile" ] || return 1
    pid=$(cat "$pidfile")
    case "$pid" in ''|*[!0-9]*) return 1;; esac
    [ -r "/proc/$pid/cmdline" ] && tr '\000' ' ' <"/proc/$pid/cmdline" | grep -Fq "$dir/service.sh run"
}

case "${1:-status}" in
start)
    running && { echo "horizon_vtx already running (pid $pid)"; exit 0; }
    # nohup: SIGHUP stays ignored even if the calling (adb) shell exits before setsid has detached
    nohup setsid sh "$dir/service.sh" run </dev/null >/dev/null 2>&1 &
    echo $! >"$pidfile"
    sleep 1
    if running; then echo "horizon_vtx started (pid $pid)"; else echo "horizon_vtx failed to start"; exit 1; fi
    ;;
stop)
    if running; then
        kill -TERM "$pid"
        n=0
        while running; do
            n=$((n + 1)); [ "$n" -lt 20 ] || { echo "horizon_vtx did not stop"; exit 1; }
            sleep 1
        done
    fi
    rm -f "$pidfile"
    echo "horizon_vtx stopped"
    ;;
restart)
    sh "$dir/service.sh" stop
    exec sh "$dir/service.sh" start
    ;;
status)
    if running; then echo "horizon_vtx running (pid $pid)"; else echo "horizon_vtx stopped"; fi
    ;;
power)
    # TX power from the Luckfox side:  power            show the current value
    #   power <half_dbm>  e.g. -19 (-9.5 dBm, minimum) .. 44 (22 dBm, maximum)    power <x>dbm  e.g. 10dbm, -9.5dbm
    #   add "save" to also keep it across restarts / reboots (--tx-power in horizon_vtx.conf)
    # the receiver can still change the power over the air; both show the same (the TX announces it downlink).
    ctl=/run/hz_vtx.ctl
    v=${2:-}
    if [ -z "$v" ]; then
        if [ -f "$ctl.pwr" ]; then
            p=$(cat "$ctl.pwr")
            echo "TX power: $p (0.5 dBm units) = $(awk -v p="$p" 'BEGIN { printf "%.1f", p / 2 }') dBm"
        else
            echo "TX power unknown (hz_vtx not running?)"; exit 1
        fi
        exit 0
    fi
    case "$v" in
    *dbm|*dBm|*DBM)
        half=$(awk -v d="${v%???}" 'BEGIN { x = d * 2; if (d == "" || x != int(x)) exit 1; print x }') ||
            { echo "power must be a multiple of 0.5 dBm"; exit 2; } ;;
    *) half=$v ;;
    esac
    case "$half" in ''|-|*[!0-9-]*|?*-*) echo "power: not a number: $v"; exit 2;; esac
    if [ "$half" -lt -19 ] || [ "$half" -gt 44 ]; then
        echo "power must be -19..44 (0.5 dBm units) = -9.5..22 dBm"; exit 2
    fi
    if [ ! -p "$ctl" ] || ! pidof hz_vtx >/dev/null 2>&1; then echo "hz_vtx is not running (service.sh start)"; exit 1; fi
    echo "pwr $half" >"$ctl"
    n=0
    while [ "$(cat "$ctl.pwr" 2>/dev/null)" != "$half" ]; do
        n=$((n + 1)); [ "$n" -lt 50 ] || { echo "TX power $half NOT confirmed by the RP2350 (old firmware?)"; exit 1; }
        sleep 0.1
    done
    echo "TX power set to $half = $(awk -v p="$half" 'BEGIN { printf "%.1f", p / 2 }') dBm"
    if [ "${3:-}" = save ]; then
        conf="$dir/horizon_vtx.conf"
        sed -i -e '/^HZ_ARGS=/s/ --tx-power -\{0,1\}[0-9]*//' -e "s/^HZ_ARGS=\"\(.*\)\"/HZ_ARGS=\"\1 --tx-power $half\"/" "$conf"
        echo "saved in $conf (applied at every start)"
    fi
    ;;
id)
    # who am I: this Luckfox and the TX RP2350 it drives (the RP2350 part is written by hz_vtx when it starts)
    echo "host_model=$(tr -d '\000' </proc/device-tree/model 2>/dev/null)"
    echo "host_soc=$(tr '\000' ' ' </proc/device-tree/compatible 2>/dev/null | sed 's/ *$//')"
    echo "host_serial=$(sed -n 's/^Serial[[:space:]]*:[[:space:]]*//p' /proc/cpuinfo)"
    if [ -f /run/hz_vtx.id ]; then
        cat /run/hz_vtx.id
    else
        echo "hz_vtx_git=$("$dir/hz_vtx" --version 2>/dev/null | sed -n 's/.*git=\([^ ]*\).*/\1/p')"
        echo "rp2350_board=unknown (hz_vtx has not started yet)"
    fi
    ;;
log)
    grep -h horizon_vtx /var/log/messages.0 /var/log/messages 2>/dev/null | grep -E 'horizon_vtx: (hz|tx|  |hz_vtx)' | tail -n "${2:-40}"
    ;;
run)
    HZ_ARGS=""
    [ -f "$dir/horizon_vtx.conf" ] && . "$dir/horizon_vtx.conf"
    child=
    finish() {
        trap - EXIT INT TERM
        for p in $(pidof hz_vtx 2>/dev/null); do kill -TERM "$p"; done
        [ -n "$child" ] && wait "$child"
        rm -f "$pidfile"
        exit 0
    }
    trap finish EXIT INT TERM
    while [ ! -c /dev/spidev0.0 ] || [ ! -e /dev/mpi/venc ]; do sleep 1; done
    # The camera must be free: stop the OEM camera app if present.
    for app in $(pidof rkipc 2>/dev/null); do kill -TERM "$app"; done
    while pidof rkipc >/dev/null 2>&1; do sleep 1; done
    governor=/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
    [ -w "$governor" ] && echo performance >"$governor"
    while :; do
        # shellcheck disable=SC2086
        "$dir/hz_vtx" $HZ_ARGS 2>&1 | logger -t horizon_vtx &
        child=$!   # (the logger; it ends when hz_vtx ends)
        wait "$child"
        child=
        logger -t horizon_vtx "hz_vtx exited, restarting in 3 s"
        sleep 3
    done
    ;;
*)
    echo "usage: $0 start|stop|restart|status|id|log [N]|power [half_dbm|<x>dbm] [save]|run"; exit 2
    ;;
esac
