#!/bin/sh
# Horizon Dev Air: the pairing service on the board's USB serial port (/dev/ttyGS0, "Horizon Dev Air").
# The setup site (Pair) writes the same link profile here and to the receiver, so both always start on the same plan.
# Text lines, the same style as the receiver's:
#   W                                   -> W board=dev-air host=<serial> hz_vtx=<git> rp2354=<git> fmt=4 image=<installed image>
#                                          boot=<start number> up=<seconds since power-up>
#   K                                   -> K profile freq_hz=<hz> hop=<n>,<spacing_hz> pwr=<half_dbm> call=<text> saved=0|1
#   K <hz> <n> <spacing_hz> <pwr_half_dbm> [callsign]
#                                       saves it in horizon_vtx.conf (--freq --hop --tx-power --callsign), restarts the
#                                       transmitter and waits until it runs on the plan:
#                                       -> K saved ... applied=1   (or C fail <why>)
#   S                                   -> the self-check (horizon_check.sh): CHECK <part> ok|warn|fail <text> lines,
#                                          then CHECK result=PASS|WARN|FAIL
#   T                                   -> test of the SPI link to the RP2354A at the transmitter's clock
#                                          (spi_speed_test; the transmitter stops for ~10 s): T EVENT ... lines, then
#                                          T RESULT PASS|FAIL
#   U <base64_bytes> <sha256>           update (horizon_update.sh): -> U ready; then exactly <base64_bytes> characters of
#                                          the base64 bundle (no newline); -> U EVENT ... progress lines, then
#                                          U RESULT PASS version=<image> reboot=0|1 or U RESULT FAIL <why>. Afterwards
#                                          this service restarts itself (new version), or the board reboots.
# Started by S99horizon_vtx; logs to syslog (tag horizon_pair).
dir=/userdata/horizon_vtx
conf=$dir/horizon_vtx.conf
share=/usr/share/horizon
tty=/dev/ttyGS0
export PATH=/sbin:/usr/sbin:/bin:/usr/bin

default_hz=$(sed -n 's/^freq_hz=//p' "$share/VERSION" 2>/dev/null)
default_hz=${default_hz:-868000000}

args() { sed -n 's/^HZ_ARGS="\(.*\)"$/\1/p' "$conf" 2>/dev/null; }
# value of option $1 in HZ_ARGS (the word after it)
opt() { args | awk -v o="$1" '{ for (i = 1; i < NF; i++) if ($i == o) { print $(i + 1); exit } }'; }

profile() {
    hz=$(opt --freq); hop=$(opt --hop); pwr=$(opt --tx-power); call=$(opt --callsign)
    saved=1; [ -n "$hz" ] || { saved=0; hz=$default_hz; }
    # (an older config's --hop n,spacing,dwell: the slots per channel are dropped, one packet per channel)
    hop=$(echo "${hop:-0,0}" | cut -d, -f1-2)
    echo "K $1 freq_hz=$hz hop=$hop pwr=${pwr:-?} call=${call:--} saved=$saved"
}

save() {  # hz n step pwr [call]
    hz=$1 n=$2 step=$3 pwr=$4 call=${5:-}
    for v in "$hz" "$n" "$step"; do case "$v" in ''|*[!0-9]*) echo "C fail profile (numbers)"; return; esac; done
    case "$pwr" in ''|-|*[!0-9-]*|?*-*) echo "C fail profile (pwr)"; return; esac
    # the sub-GHz range or the 2.4 GHz band (the radio controller checks the whole plan)
    if { [ "$hz" -lt 150000000 ] || [ "$hz" -gt 960000000 ]; } && { [ "$hz" -lt 2400000000 ] || [ "$hz" -gt 2483500000 ]; }; then
        echo "C fail profile range freq_hz"; return
    fi
    if [ "$pwr" -lt -19 ] || [ "$pwr" -gt 44 ]; then echo "C fail profile range pwr"; return; fi
    if [ "$n" != 0 ]; then
        if [ "$n" -lt 2 ] || [ "$n" -gt 128 ] || [ $((step % 1000)) != 0 ] || [ "$step" = 0 ]; then
            echo "C fail profile range hop"; return
        fi
    fi
    case "$call" in *[!A-Za-z0-9/-]*) echo "C fail profile (callsign: letters, digits, / and - only)"; return; esac
    # HZ_ARGS without the four options, then the new values
    keep=$(args | awk '{ out = ""; for (i = 1; i <= NF; i++) { if ($i == "--freq" || $i == "--hop" || $i == "--tx-power" || $i == "--callsign") { i++; continue } out = out (out == "" ? "" : " ") $i } print out }')
    new="$keep --freq $hz --tx-power $pwr"
    [ "$n" != 0 ] && new="$new --hop $n,$step"
    [ -n "$call" ] || call=$(opt --callsign)   # no callsign given: keep the one it has (without one it stays silent)
    [ -n "$call" ] && new="$new --callsign $call"
    new=$(echo "$new" | sed 's/^ *//')
    if grep -q '^HZ_ARGS=' "$conf" 2>/dev/null; then
        sed "s|^HZ_ARGS=.*|HZ_ARGS=\"$new\"|" "$conf" >"$conf.new"
    else
        { cat "$conf" 2>/dev/null; echo "HZ_ARGS=\"$new\""; } >"$conf.new"
    fi
    mv "$conf.new" "$conf" && sync
    logger -t horizon_pair "saved: HZ_ARGS=\"$new\""
    # restart and wait until hz_vtx reports the plan in use: its identity file (written anew at every start), then the
    # plan line among the latest log lines (the syslog rotates, so not by line number)
    rm -f /run/hz_vtx.id
    sh "$dir/service.sh" restart >/dev/null 2>&1
    i=0 applied=0
    while [ "$i" -lt 60 ]; do
        if [ -f /run/hz_vtx.id ] && cat /var/log/messages.0 /var/log/messages 2>/dev/null | tail -300 |
            grep -q "hz: plan freq_hz=$hz hop=$n,$step in use"; then applied=1; break; fi
        sleep 0.5; i=$((i + 1))
    done
    echo "$(profile saved) applied=$applied"
}

who() {
    id=$(cat /run/hz_vtx.id 2>/dev/null)
    v() { echo "$id" | sed -n "s/^$1=//p"; }
    echo "W board=dev-air host=$(sed -n 's/^Serial[[:space:]]*:[[:space:]]*//p' /proc/cpuinfo) hz_vtx=$(v hz_vtx_git) rp2354=$(v rp2350_git) fmt=$(v rp2350_format) image=$(sed -n 's/^image=//p' "$share/VERSION") boot=$(cat /run/horizon_boot 2>/dev/null || echo 0) up=$(cut -d. -f1 /proc/uptime)"
}

spitest() {
    sh "$dir/service.sh" stop >/dev/null 2>&1
    sleep 1
    "$share/spi_speed_test" --max-torn 8 --requests 20 14200000 2>&1 | grep -E '^(EVENT clock_req=[0-9]* miso|RESULT|FATAL)' | sed 's/^/T /'
    sh "$dir/service.sh" start >/dev/null 2>&1
}

restart_self=0
update() {  # base64 length, sha256 of the bundle
    n=$1 want=$2
    case "$n" in ''|*[!0-9]*) echo "U RESULT FAIL length"; return ;; esac
    [ "$n" -gt 0 ] && [ "$n" -le 12000000 ] || { echo "U RESULT FAIL length $n"; return; }
    case "$want" in [0-9a-f]*) ;; *) echo "U RESULT FAIL sha256"; return ;; esac
    up=/userdata/horizon_update   # on the card: /tmp is RAM, of which the Air has little to spare
    rm -f $up.b64 $up.tar.xz $up.out
    echo "U ready"
    # exactly n characters from the port, given up after 120 s (a site that went away must not leave this port
    # swallowing the next commands)
    head -c "$n" <"$tty" >$up.b64 &
    hp=$!
    ( sleep 120; kill "$hp" 2>/dev/null ) &
    wp=$!
    wait "$hp"
    kill "$wp" 2>/dev/null
    got=$(wc -c <$up.b64)
    [ "$got" = "$n" ] || { echo "U RESULT FAIL received $got of $n bytes"; rm -f $up.b64; return; }
    base64 -d $up.b64 >$up.tar.xz 2>/dev/null || { echo "U RESULT FAIL base64"; return; }
    rm -f $up.b64
    have=$(sha256sum $up.tar.xz | cut -d' ' -f1)
    [ "$have" = "$want" ] || { echo "U RESULT FAIL sha256 $have"; rm -f $up.tar.xz; return; }
    echo "U EVENT received $(wc -c <$up.tar.xz) bytes, sha256 ok"
    # each line as it comes (a shell loop: sed or tee into a pipe would hold the lines back until the end)
    sh "$share/horizon_update.sh" $up.tar.xz 2>&1 | while IFS= read -r l; do echo "U $l"; echo "U $l" >>$up.out; done
    rm -f $up.tar.xz
    grep -q '^U RESULT PASS' $up.out && restart_self=1
}

handle() {
    set -- $1
    case "${1:-}" in
    W) who ;;
    K) shift; if [ $# = 0 ]; then profile profile; else save "$@"; fi ;;
    S) sh "$share/horizon_check.sh" ;;   # self-check: CHECK lines, then CHECK result=...
    T) spitest ;;
    U) shift; update "$@" ;;
    '') ;;
    *) echo "C fail bad command" ;;
    esac
}

logger -t horizon_pair "started on $tty"
while :; do
    if [ ! -c "$tty" ]; then sleep 2; continue; fi
    stty -F "$tty" raw -echo 2>/dev/null
    # one line in, one reply out; a host that closes the port simply ends this read loop
    while IFS= read -r line; do
        line=$(printf '%s' "$line" | tr -d '\r')
        [ -n "$line" ] || continue
        handle "$line" >"$tty"
        # after an update: this service again, from its new file
        [ "$restart_self" = 1 ] && { logger -t horizon_pair "updated: restarting"; exec sh "$share/horizon_pair.sh"; }
    done <"$tty"
    sleep 1
done
