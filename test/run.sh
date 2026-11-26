#!/bin/bash
# Tests one device's software, then the device itself (plugged into this computer by USB); or software only.
#
#   bash test/run.sh [WHAT] [--capture FILE]      without WHAT: lists the plugged-in boards and asks
#
#   air          transmitter host tests, the Air scripts parse, its radio firmware builds; device: identity,
#                self-check, SPI link test (test/device.py)
#   pi-hat       hz_vtx compiles for the Pi (main.c), the HAT transmitter firmware builds; device: identity, radio, link settings
#   ground-unit  the receiver firmware builds; device: identity, radio, link settings, status
#   hat-rx       the same for the Horizon HAT as receiver
#   pair         factory test: an Air and a Ground (both checks, same link settings, 10 s of radio link)
#   app          desktop_receiver unit tests; with --capture (a receiver capture, packets.rxlog) test/block_fec.sh
#   site         the setup site's JavaScript parses, its Reed-Solomon self-test
#   all          all the software above, no device
#
# Linux, or Git Bash on Windows (the C and Python tests and the firmware build then run in WSL). Prints a summary and
# RESULT PASS|FAIL; each step's output is kept in a log folder.
set -u
cd "$(dirname "$0")/.." || exit 1
root=$PWD
py=python3; "$py" -c '' 2>/dev/null || py=python
what= capture=
while [ $# -gt 0 ]; do
    case "$1" in --capture) capture=$2; shift ;; *) what=$1 ;; esac
    shift
done

choices="air:Horizon Dev Air
pi-hat:Raspberry Pi 5 + Horizon HAT
ground-unit:Horizon Dev Ground Unit
hat-rx:Horizon HAT as receiver
pair:Air + Ground over the radio (factory test)
app:Horizon Receiver for Windows (software only)
site:Setup site (software only)
all:All the software (no device)"
if [ -z "$what" ]; then
    list=$("$py" test/device.py list)
    echo "Plugged in:"; echo "${list:-(nothing)}" | sed 's/^/  /'
    echo "$choices" | L=$list awk -F: 'BEGIN { n = split(ENVIRON["L"], l, "\n"); for (i = 1; i <= n; i++) { split(l[i], f, " "); port[f[1]] = f[2] } }
        { printf "  %d) %s%s\n", NR, $2, ($1 in port) ? "  (plugged in: " port[$1] ")" : "" }'
    printf "What to test? "; read -r pick
    what=$(echo "$choices" | awk -F: -v n="$pick" 'NR == n { print $1 }')
fi
echo "$choices" | grep -q "^$what:" || { echo "FATAL test what? $(echo "$choices" | cut -d: -f1 | tr '\n' ' ')"; exit 2; }

if [ -n "${WINDIR:-}" ]; then  # Git Bash: Linux commands in WSL
    lin() { MSYS_NO_PATHCONV=1 wsl.exe --exec bash -lc "cd \"\$(wslpath '$(cygpath -m "$root")')\" && $1"; }
    command -v node >/dev/null || node() { lin "node $*"; }
else
    lin() { bash -c "$1"; }
fi

log=$(mktemp -d)
results=
step() {  # step NAME COMMAND...: PASS / FAIL (its last lines shown) / SKIP (exit status 3)
    echo "EVENT step $1"
    "${@:2}" >"$log/$1.log" 2>&1
    case $? in 0) r=PASS ;; 3) r=SKIP ;; *) r=FAIL ;; esac
    if [ $r = FAIL ]; then tail -15 "$log/$1.log"; else grep -E '^(EVENT|SKIP)|: OK$|^test result' "$log/$1.log" | tail -8; fi |
        sed 's/^/  /'
    results="$results  $1 $r
"
}

luckfox_host() {
    lin "set -e; mkdir -p transmitter/build
         cc -O2 -Wall -Wextra -Werror transmitter/test/test_sched_dump.c -lpthread -o transmitter/build/test_sched_dump
         python3 transmitter/test/test_swfec.py transmitter/build/test_sched_dump"
}
air_scripts() { lin 'set -e; for f in luckfox_image/*.sh luckfox_image/S99horizon_vtx transmitter/*.sh; do sh -n "$f"; done'; }
pi_host() { lin "cc -O2 -Wall -Wextra -Werror -DHZ_PI -c transmitter/main.c -o /tmp/hz_pi_main.o"; }
firmware() {  # the radio firmware targets build (incremental: HZ_FW_BUILD, else a folder per checkout in ~/hz-build)
    lin "set -e; export PICO_SDK_PATH=\${PICO_SDK_PATH:-\$HOME/pico-sdk-2.2.0}
         b=\${HZ_FW_BUILD:-\$HOME/hz-build/test/fw-\$(pwd | md5sum | cut -c1-8)}
         cmake -S radio_firmware -B \$b -DCMAKE_BUILD_TYPE=Release >/dev/null; cmake --build \$b -j\$(nproc) --target $*"
}
app() {  # Horizon Receiver is a Windows app
    [ -n "${WINDIR:-}" ] || { echo "SKIP the desktop app builds on Windows (CI: the windows job)"; return 3; }
    (cd desktop_receiver && cargo test --lib -q)
}
site() {
    [ -d setup_site ] || { echo "SKIP no setup_site/ here"; return 3; }
    for f in setup_site/js/*.js setup_site/js/*/*.js; do node --check "$f" || return 1; done
    node setup_site/test/check_swfec.mjs --rs
}
block_fec() {
    [ -n "$capture" ] && [ -f test/block_fec.sh ] || { echo "SKIP needs --capture FILE"; return 3; }
    sh test/block_fec.sh "$capture" "$log/block_fec"
}
device() { if [ $what = pair ]; then "$py" test/device.py pair; else "$py" test/device.py check "$what"; fi; }

echo "EVENT testing $what at $(git rev-parse --short=8 HEAD)$(git diff HEAD --quiet || echo ' + uncommitted changes')"
case $what in
    air) step luckfox_host luckfox_host; step air_scripts air_scripts; step firmware firmware air_radio ;;
    pi-hat) step pi_host pi_host; step firmware firmware hat_tx ;;
    ground-unit) step firmware firmware ground_unit ;;
    hat-rx) step firmware firmware hat_rx ;;
    app) step app app; step block_fec block_fec ;;
    site) step site site ;;
    all) step luckfox_host luckfox_host; step air_scripts air_scripts; step pi_host pi_host; step app app
         step site site; step block_fec block_fec
         step firmware firmware air_radio hat_tx ground_unit hat_rx ;;
esac
case $what in air|pi-hat|ground-unit|hat-rx|pair) step device device ;; esac

printf -- "---- %s (logs: %s)\n%s" "$what" "$log" "$results"
case $results in *FAIL*) echo "RESULT FAIL"; exit 1 ;; esac
echo "RESULT PASS"
