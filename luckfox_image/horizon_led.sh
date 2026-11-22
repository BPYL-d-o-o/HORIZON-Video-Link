#!/bin/sh
# Horizon Dev Air: the Luckfox's LED ("work") shows the board's self-check (horizon_check.sh), so the Air can be read
# without a laptop. The RP2354A's red LED shows the radio side (lora_vlink/src/main.c, led_tick()).
#
#   short flash every second  starting (the first check after power-up is not done yet)
#   on                        every part ok
#   slow blink (1 s / 1 s)    a warning: the Air works, something needs attention (the N0CALL default callsign,
#                             a low frame rate, ...): the setup site's Air page names it
#   fast blink (5 per second) a fault: a part does not work (camera, transmitter, radio controller, card)
#   blinking with activity    the stock behaviour, before this script runs (early boot)
#
# Started by S99horizon_vtx. Runs horizon_check.sh every 30 s at low priority (about 1.7 s of CPU each), keeps the
# latest report in /run/horizon_check.txt and logs the parts that are not ok to syslog (tag horizon_check) whenever the
# result changes.
share=/usr/share/horizon
led=/sys/class/leds/work
[ -d "$led" ] || { echo "ERROR no LED $led"; exit 1; }
echo none >"$led/trigger"
state=/run/horizon_led.state
echo starting >"$state"
rm -f /run/horizon_led.busy /run/horizon_led.logged
next=45  # the first check: once the transmitter has had time to start

check() {  # in the background: the blinking must not stop while the check runs
    nice -n 19 sh "$share/horizon_check.sh" >/run/horizon_check.tmp 2>&1
    mv /run/horizon_check.tmp /run/horizon_check.txt
    r=$(sed -n 's/^CHECK result=//p' /run/horizon_check.txt)
    read up _ </proc/uptime
    # the check reports parts that are still coming up as warnings for the first 90 s: keep "starting" until then
    if [ "$r" = WARN ] && [ "${up%.*}" -lt 90 ]; then r=starting; fi
    echo "${r:-FAIL}" >"$state"
    # syslog (tag horizon_check): the parts that are not ok, whenever the result changes
    if [ "$r" != starting ] && [ "$r" != "$(cat /run/horizon_led.logged 2>/dev/null)" ]; then
        echo "$r" >/run/horizon_led.logged
        grep -v '^CHECK [a-z]* ok ' /run/horizon_check.txt | logger -t horizon_check
    fi
    rm -f /run/horizon_led.busy
}

i=0
cur=starting
while :; do
    if [ "$i" = 0 ]; then  # every 2 s: the latest result, and start a check when one is due
        read cur <"$state"
        read up _ </proc/uptime
        if [ "${up%.*}" -ge "$next" ] && [ ! -e /run/horizon_led.busy ]; then
            next=$(( ${up%.*} + 30 ))
            : >/run/horizon_led.busy
            check </dev/null >/dev/null 2>&1 &
        fi
    fi
    case "$cur" in
    PASS) b=255 ;;
    WARN) if [ "$i" -lt 10 ]; then b=255; else b=0; fi ;;
    starting) if [ "$i" = 0 ] || [ "$i" = 10 ]; then b=255; else b=0; fi ;;
    *) b=$(( (i % 2) * 255 )) ;;  # FAIL
    esac
    echo "$b" >"$led/brightness"
    i=$(( (i + 1) % 20 ))
    usleep 100000
done
