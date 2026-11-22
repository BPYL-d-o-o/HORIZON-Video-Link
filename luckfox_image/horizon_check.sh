#!/bin/sh
# Horizon Dev Air self-check: reports each part of the board separately, so a fault in one (camera, radio, radio
# controller, its SPI or SWD connection) is named even when the transmitter cannot run. Needs only the Luckfox and
# its card: it reads the system, the boot-time RP2354A update (rp2354_update.sh) and hz_vtx's log, nothing else.
#
#   sh horizon_check.sh      lines "CHECK <part> ok|warn|fail <text>", then "CHECK result=PASS|WARN|FAIL"
#
# Run at boot after the transmitter starts (rp2354_update.sh boot; result in syslog and /run/horizon_check.txt) and
# on request over the USB serial port (horizon_pair.sh "S": the setup site's Air page).
share=/usr/share/horizon
dir=/userdata/horizon_vtx
export PATH=/sbin:/usr/sbin:/bin:/usr/bin
worst=0  # 0 PASS, 1 WARN, 2 FAIL
say() {  # part level text
    case "$2" in warn) [ "$worst" -lt 1 ] && worst=1 ;; fail) worst=2 ;; esac
    echo "CHECK $1 $2 $3"
}
log() { cat /var/log/messages.0 /var/log/messages 2>/dev/null; }
# seconds since a syslog line ("Jan  1 00:01:02 host tag: ..."), by time of day
now_s=$(date +%H:%M:%S | awk -F: '{ print $1 * 3600 + $2 * 60 + $3 }')
age() { echo "$1" | awk -v now="$now_s" '{ split($3, t, ":"); a = now - (t[1] * 3600 + t[2] * 60 + t[3]); if (a < 0) a += 86400; print a }'; }
vtx() { log | grep 'horizon_vtx: ' | sed 's/^.*horizon_vtx: //'; }
# the first ~90 s after power-up: parts that are not up yet are "starting", not faults (S99luckfoxconfigload enables
# SPI about 15 s in, the camera and the transmitter follow)
up=$(cut -d. -f1 /proc/uptime)
starting=0; [ "${up:-999}" -lt 90 ] && starting=1
# hz_vtx's lines of the last 60 s: errors and restarts count only while they are current (a fixed fault stops being
# reported within a minute, a transmitter that keeps failing keeps showing)
vtx_recent() {
    log | grep 'horizon_vtx: ' | awk -v now="$now_s" '{ split($3, t, ":"); a = now - (t[1] * 3600 + t[2] * 60 + t[3]); if (a < 0) a += 86400; if (a <= 60) print }' |
        sed 's/^.*horizon_vtx: //'
}

# 1. the card: image, root filesystem grown to its partition, settings partition writable
img=$(sed -n 's/^image=//p' "$share/VERSION" 2>/dev/null)
root_mb=$(df -m / 2>/dev/null | awk 'NR == 2 { print $2 }')
if [ -z "$img" ]; then say card fail "not a Horizon Dev camera image (no $share/VERSION): write the image again"
elif ! touch "$dir/.check" 2>/dev/null; then say card fail "the settings partition (/userdata) is not writable: card damaged or full"
elif [ "${root_mb:-0}" -lt 900 ]; then say card warn "$img, root filesystem ${root_mb} MB (not grown to the card yet: restart once)"
else say card ok "$img, $(df -h / | awk 'NR == 2 { print $2 }') root filesystem, start $(cat /run/horizon_boot 2>/dev/null || echo ?), up $((up / 60)) min"; fi
rm -f "$dir/.check"

# 2. SPI to the radio controller (Luckfox SPI0, /etc/luckfox.cfg)
if [ -c /dev/spidev0.0 ]; then say spi ok "/dev/spidev0.0"
elif [ "$starting" = 1 ]; then say spi warn "not enabled yet (the Air unit started ${up} s ago)"
else say spi fail "SPI0 to the radio controller is not enabled (/etc/luckfox.cfg SPI0_M0_STATUS=1): write the image again"; fi

# 3. the radio controller's firmware update over SWD (at boot, when the image carries other firmware)
want=$(md5sum <"$share/rp2354.elf" 2>/dev/null | cut -d' ' -f1)
have=$(cat "$dir/rp2354.flashed" 2>/dev/null)
last=$(vtx | grep -E '^EVENT rp2354_flash (start|ok)|^ERROR rp2354_flash|DPIDR' | tail -3)
if echo "$last" | tail -1 | grep -q '^ERROR rp2354_flash'; then
    if echo "$last" | grep -q DPIDR; then say swd fail "the radio controller answered over SWD, but writing its firmware failed: restart to retry"
    else say swd fail "no answer from the radio controller (RP2354A) over SWD: its power, or the SWD lines (Luckfox pins 9, 10)"; fi
elif echo "$last" | tail -1 | grep -q '^EVENT rp2354_flash start'; then say swd warn "updating the radio controller now (about 20 s)"
elif [ -n "$have" ] && [ "$have" = "$want" ]; then say swd ok "radio controller firmware up to date"; swd_current=1
else say swd warn "radio controller not updated yet (it is at the next start)"; fi

# 4. the transmitter program and the radio controller's SPI link (its identity)
running=0; sh "$dir/service.sh" status 2>/dev/null | grep -q running && running=1
restarts=$(vtx_recent | grep -c 'hz_vtx exited, restarting')
err=$(vtx_recent | grep '^hz: ERROR' | tail -1 | sed 's/^hz: ERROR //')
id=$(cat /run/hz_vtx.id 2>/dev/null)
idv() { echo "$id" | sed -n "s/^$1=//p"; }
board=$(idv rp2350_board)
if [ "$running" = 0 ]; then
    # why not, from its log: stopped on purpose, a start that failed (and what it told the receivers), or never started
    # its last run: from the last "hz: running" or start-up line to its "hz: stopped" line
    tail_run=$(log | grep -E 'horizon_vtx: hz: (hz_vtx git=|stop requested|ERROR|reported to the receivers|stopped)' |
        awk '/hz: hz_vtx git=/ { n = 0 } { l[n++] = $0 } END { for (i = 0; i < n; i++) print l[i] }')
    lastline=$(echo "$tail_run" | tail -1)
    ago=""; [ -n "$lastline" ] && ago=" $(age "$lastline") s ago"
    lasterr=$(echo "$tail_run" | grep 'hz: ERROR' | tail -1 | sed 's/^.*hz: ERROR //')
    reported=$(echo "$tail_run" | grep 'hz: reported to the receivers: ' | tail -1 | sed 's/^.*reported to the receivers: //')
    start="start it: sh $dir/service.sh start, or restart the Air unit"
    if [ -z "$tail_run" ]; then why="it has not run since power-up (the boot start was interrupted?)"
    elif echo "$tail_run" | grep -q 'hz: stop requested'; then why="stopped on purpose$ago (service.sh stop, or a test that stopped it)"
    elif [ -n "$reported" ]; then why="its last start failed$ago ($reported)${lasterr:+: $lasterr}"
    elif [ -n "$lasterr" ]; then why="it stopped with an error$ago: $lasterr"
    else why="it stopped$ago"; fi
    say service fail "the transmitter is not running: $why; $start"
elif [ "$restarts" -gt 2 ]; then say service fail "the transmitter keeps restarting ($restarts times in a minute)${err:+: $err}"
else say service ok "running"; fi
if echo "$err" | grep -q 'no status from the RP2350'; then
    say link fail "the radio controller does not answer over SPI: its power, firmware, or the SPI lines"
elif [ -z "$id" ]; then say link warn "not identified yet (the transmitter is starting)"
elif [ "$board" = unknown ] || [ -z "$board" ]; then
    # the image flashes its own firmware (which identifies itself) first, so with current firmware this means the
    # RP2354A is not running: its SPI slave hardware only repeats its last status
    if [ "${swd_current:-0}" = 1 ]; then say link fail "the radio controller does not respond over SPI (not running): restart the board; if it stays, its power or the SPI lines"
    else say link warn "the radio controller answers but does not identify itself (old firmware)"; fi
else
    fw_git=$(idv rp2350_git); img_git=$(sed -n 's/^rp2354=//p' "$share/VERSION")
    if [ -n "$img_git" ] && [ "$fw_git" != "$img_git" ]; then say link warn "answering, firmware $fw_git (the card carries $img_git)"
    else say link ok "answering, firmware $fw_git"; fi
fi

# 5. the radio (LR2021) as the radio controller reports it, then what it actually sends
# the callsign: without one the radio controller does not transmit at all (no transmission without identification)
callsign=$(idv rp2350_callsign); call_conf=$(sed -n 's/^HZ_ARGS=.*--callsign \([^ "]*\).*/\1/p' "$dir/horizon_vtx.conf" 2>/dev/null)
if [ "$callsign" = none ]; then say callsign fail "cleared: the Air unit does not transmit until you set one (Pair, step 3)"
elif [ "$callsign" = default ] || [ "$call_conf" = N0CALL ]; then say callsign warn "N0CALL, the default for debugging: set your own callsign with Pair (step 3)"
elif [ -n "$call_conf" ]; then say callsign ok "$call_conf, sent every 5 minutes"
elif [ "$callsign" = set ]; then say callsign ok "set, sent every 5 minutes"; fi
radio=$(idv rp2350_radio)
t=$(vtx | grep 'RP2350: T seq=' | tail -1)
f=$(vtx | grep 'RP2350: freq_hz=' | tail -1)
sent=$(echo "$t" | sed -n 's/.* sent=\([0-9]*\).*/\1/p'); fail=$(echo "$t" | sed -n 's/.* fail=\([0-9]*\).*/\1/p')
fresh=0; [ -n "$t" ] && [ "$(age "$(log | grep 'RP2350: T seq=' | tail -1)")" -le 15 ] && fresh=1
[ "$callsign" = none ] && [ "$radio" != failing ] && radio=nocall
case "$radio" in
nocall) say radio warn "silent: waiting for a callsign" ;;
failing) say radio fail "the radio module does not start (error -$(idv rp2350_radio_rc)): the LR2021 module, its power or its SPI lines" ;;
starting) say radio warn "the radio is starting" ;;
*)
    if [ "$fresh" = 0 ]; then
        [ "$running" = 1 ] && say radio warn "no transmit report in the last 15 s" || say radio fail "not transmitting (the transmitter is not running)"
    elif [ "${sent:-0}" = 0 ]; then say radio fail "the radio sends nothing (RP2354A: sent=0)"
    elif [ "${fail:-0}" -gt 0 ]; then say radio fail "the radio fails to send $fail of $sent packets: LR2021 module or its power"
    else
        hz=$(echo "$f" | sed -n 's/.*freq_hz=\([0-9]*\).*/\1/p'); pw=$(echo "$f" | sed -n 's/.*pwr=\(-\{0,1\}[0-9]*\).*/\1/p')
        se=$(echo "$f" | sed -n 's/.*spi_errors=\([0-9]*\).*/\1/p')
        txt="sending $sent packets in 5 s on $(awk -v h="${hz:-0}" 'BEGIN { printf "%.3f", h / 1e6 }') MHz, LR2021 $(awk -v p="${pw:-0}" 'BEGIN { print p / 2 }') dBm"
        if [ "${se:-0}" -gt 0 ]; then say radio warn "$txt, $se SPI errors"; else say radio ok "$txt"; fi
    fi ;;
esac

# 6. the camera: detected by the kernel at power-up (the sensor answered on I2C: a video device named after it,
# e.g. "m00_b_sc3336 4-0030"), then started by the transmitter, then the video it gives
ksensor=$(cat /sys/class/video4linux/*/name 2>/dev/null | grep -m1 '^m[0-9]*_[a-z]_' | sed 's/^m[0-9]*_[a-z]_//; s/ .*//' | tr a-z A-Z)
v=$(vtx | grep '^tx: [0-9.]* fps' | tail -1)
fps=$(echo "$v" | sed -n 's/^tx: \([0-9.]*\) fps.*/\1/p'); kbps=$(echo "$v" | sed -n 's/.*, \([0-9.]*\) kbit\/s video.*/\1/p')
vfresh=0; [ -n "$v" ] && [ "$(age "$(log | grep 'horizon_vtx: tx: [0-9.]* fps' | tail -1)")" -le 15 ] && vfresh=1
if [ -z "$ksensor" ]; then
    if [ "$starting" = 1 ]; then say camera warn "not detected yet (the Air unit started ${up} s ago)"
    else say camera fail "no camera detected: check that its ribbon cable is in fully, contacts the right way round"; fi
elif echo "$err" | grep -qiE 'sensor|rk_aiq|VI_|ISP|camera'; then say camera fail "$ksensor camera detected, but it does not start ($err)"
elif [ "$vfresh" = 0 ]; then say camera warn "$ksensor camera detected, no video yet"
elif awk -v f="${fps:-0}" 'BEGIN { exit !(f < 10) }'; then say camera warn "$ksensor camera, only ${fps} pictures/s"
else say camera ok "$ksensor camera, ${fps} pictures/s, ${kbps} kbit/s video"; fi

case "$worst" in 0) echo "CHECK result=PASS" ;; 1) echo "CHECK result=WARN" ;; *) echo "CHECK result=FAIL" ;; esac
