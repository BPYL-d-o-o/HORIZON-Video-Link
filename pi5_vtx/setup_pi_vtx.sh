#!/usr/bin/env bash
#
# setup_pi_vtx.sh
#
# Setup script for Raspberry Pi (Any model) as a Horizon video link transmitter.
#
# This script sets up a fresh Debian /
# Raspberry Pi OS install. It installs system packages, builds the frame_enc encoder from
# the existing pi5_vtx/frame_enc.c, configures a systemd service from the
# pi5_vtx/lora-vlink-tx.service unit file, and references the existing firmware.
#
# Prerequisites: run as a normal user with sudo access (or as root) on a Pi
# with internet access and the RP2350 TX board available for flashing (plugged in via USB).
# In operation the Pi talks to the TX RP2350 over SPI (40-pin header) or over USB; live_sender.py
# picks whichever answers (--link auto). This script enables SPI so both work.
#
# Usage:
#   bash setup_pi_vtx.sh [--repo-url <git-remote>] [--repo-dir <dir>]
#                        [--no-systemd] [--skip-flash] [--non-interactive]
#
# Default repo:    https://github.com/BPYL-d-o-o/HORIZON-Video-Link.git
# Default repo dir: ~/lora-video-system
#
# The script is normally run from inside a cloned copy of this repo
# (pi5_vtx/setup_pi_vtx.sh). In that case it reuses the existing checkout and
# skips the git clone step.
#
set -euo pipefail

# ---------------------------------------------------------------------------
# Defaults (overridable via flags or environment)
# ---------------------------------------------------------------------------
DEFAULT_REPO_URL="${SETUP_REPO_URL:-https://github.com/BPYL-d-o-o/HORIZON-Video-Link.git}"
DEFAULT_REPO_DIR="${SETUP_REPO_DIR:-$HOME/lora-video-system}"

REPO_URL="$DEFAULT_REPO_URL"
REPO_DIR="$DEFAULT_REPO_DIR"
DO_SYSTEMD=1
DO_FLASH=1
NON_INTERACTIVE=0
SKIP_CLONE=0

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
msg() { printf '\n==> %s\n' "$*"; }
ok()  { printf '   [ok] %s\n' "$*"; }
warn() { printf '   [warn] %s\n' "$*" >&2; }
err() { printf '   [error] %s\n' "$*" >&2; }

usage() {
    cat <<EOF
Usage: $(basename "$0") [--repo-url URL] [--repo-dir DIR] [--no-systemd]
                          [--skip-flash] [--non-interactive] [-h|--help]

Set up a Raspberry Pi (any model) as a Horizon video link transmitter.

Options:
  --repo-url URL        Git remote to clone (default: $DEFAULT_REPO_URL)
  --repo-dir DIR        Directory to clone into / use (default: $DEFAULT_REPO_DIR)
  --no-systemd          Skip installing the systemd service unit
  --skip-flash          Skip the RP2350 flashing instructions
  --non-interactive     Do not prompt (use defaults; useful for automation)
  -h, --help            Show this help

Steps performed:
  1. Install system packages and add the user to the dialout group
  2. Clone / use the repository
  3. Build the x265 frame encoder (pi5_vtx/frame_enc)
  4. Flash the TX RP2350 (vlive_tx.uf2) with picotool, or manual BOOTSEL instructions
  5. Document the manual test run (live_sender.py)
  6. Install and enable the systemd service lora-vlink-tx.service (+ /etc/default/lora-vlink-tx)

EOF
}

# ---------------------------------------------------------------------------
# Parse arguments
# ---------------------------------------------------------------------------
while [ $# -gt 0 ]; do
    case "$1" in
        --repo-url)       REPO_URL="$2"; shift 2 ;;
        --repo-dir)       REPO_DIR="$2"; shift 2 ;;
        --no-systemd)     DO_SYSTEMD=0; shift ;;
        --skip-flash)     DO_FLASH=0; shift ;;
        --non-interactive) NON_INTERACTIVE=1; shift ;;
        -h|--help)        usage; exit 0 ;;
        *)                err "unknown option: $1"; usage; exit 2 ;;
    esac
done

# ---------------------------------------------------------------------------
# Privilege handling (support running as root or as a sudoer)
# ---------------------------------------------------------------------------
if [ "$(id -u)" -eq 0 ]; then
    SUDO=""
    TARGET_USER="${SUDO_USER:-$USER}"
    msg "Running as root; target user is $TARGET_USER"
else
    SUDO="sudo"
    if ! $SUDO -n true 2>/dev/null; then
        err "This script needs sudo access. Please run as a user with sudo privileges."
        exit 1
    fi
    TARGET_USER="$USER"
    msg "Running as user $TARGET_USER with sudo"
fi
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_REPO_DIR="$(cd "$SCRIPT_DIR/.." 2>/dev/null && pwd || true)"

# ---------------------------------------------------------------------------
# Determine the repository directory to use.
# If the script itself lives inside a checked-out repo, prefer that checkout
# and skip cloning. Otherwise use --repo-dir (default ~/lora-video-system).
# ---------------------------------------------------------------------------
if [ -f "$SCRIPT_REPO_DIR/pi5_vtx/frame_enc.c" ] \
   && [ -f "$SCRIPT_REPO_DIR/pi5_vtx/lora-vlink-tx.service" ] \
   && [ -f "$SCRIPT_REPO_DIR/prebuilt/firmware/vlive_tx.uf2" ]; then
    REPO_DIR="$SCRIPT_REPO_DIR"
    SKIP_CLONE=1
    msg "Detected existing repo checkout at $REPO_DIR (script lives here); skipping git clone."
elif [ -d "$REPO_DIR" ] \
     && [ -f "$REPO_DIR/pi5_vtx/frame_enc.c" ] \
     && [ -f "$REPO_DIR/pi5_vtx/lora-vlink-tx.service" ]; then
    SKIP_CLONE=1
    msg "Using existing repo at $REPO_DIR; skipping git clone."
fi

if [ -d "$REPO_DIR" ]; then
    REPO_DIR="$(cd "$REPO_DIR" && pwd)"
fi

# ---------------------------------------------------------------------------
# Step 0: sanity / environment
# ---------------------------------------------------------------------------
msg "Step 0: environment check"
if [ -z "$REPO_DIR" ] && [ "$SKIP_CLONE" -eq 1 ]; then
    err "Could not determine repository directory."
    exit 1
fi
msg "Repository directory: $REPO_DIR"

# ---------------------------------------------------------------------------
# Step 1: System packages
# ---------------------------------------------------------------------------
msg "Step 1: installing system packages"
$SUDO apt-get update
$SUDO apt-get install -y \
    git \
    python3-picamera2 \
    python3-numpy \
    python3-serial \
    libx265-dev \
    build-essential \
    ffmpeg
# picotool flashes the TX RP2350 without the BOOTSEL button (optional: not in every release)
$SUDO apt-get install -y picotool 2>/dev/null || warn "picotool not available; flash by BOOTSEL + copying the .uf2"

# SPI to the TX RP2350 (40-pin header). USB works without it; live_sender.py --link auto uses SPI when the
# RP2350 answers there. Takes effect after a reboot.
if command -v raspi-config >/dev/null 2>&1; then
    if [ "$($SUDO raspi-config nonint get_spi)" != "0" ]; then
        $SUDO raspi-config nonint do_spi 0 && ok "SPI enabled (reboot to activate /dev/spidev0.0)"
    else
        ok "SPI already enabled."
    fi
else
    warn "raspi-config not found: enable SPI yourself (dtparam=spi=on in config.txt) if the RP2350 is wired by SPI"
fi

# Add the user to the dialout (RP2350 USB serial), spi (/dev/spidev*) and video (camera) groups.
# This takes effect at next login / after reboot.
for g in dialout spi video; do
    getent group "$g" >/dev/null || continue
    if ! id -nG "$TARGET_USER" | tr ' ' '\n' | grep -qx "$g"; then
        $SUDO usermod -aG "$g" "$TARGET_USER"
        ok "Added $TARGET_USER to $g — log out and back in (or reboot) for it to take effect."
    else
        ok "User $TARGET_USER is already in $g."
    fi
done

# ---------------------------------------------------------------------------
# Step 2: Get the code
# ---------------------------------------------------------------------------
if [ "$SKIP_CLONE" -eq 1 ]; then
    msg "Step 2: code already present at $REPO_DIR (skipping clone)"
else
    msg "Step 2: cloning repository from $REPO_URL into $REPO_DIR"
    if [ -d "$REPO_DIR" ]; then
        warn "$REPO_DIR already exists; skipping clone."
    else
        git clone "$REPO_URL" "$REPO_DIR"
        ok "Cloned to $REPO_DIR"
    fi
fi

# Re-derive absolute repo dir after a possible clone.
REPO_DIR="$(cd "$REPO_DIR" && pwd)"
cd "$REPO_DIR"
# ---------------------------------------------------------------------------
# Step 3: Build the encoder
# ---------------------------------------------------------------------------
msg "Step 3: building the x265 frame encoder (pi5_vtx/frame_enc)"
gcc -O2 -o "$REPO_DIR/pi5_vtx/frame_enc" "$REPO_DIR/pi5_vtx/frame_enc.c" -lx265 -lm
ok "Built $REPO_DIR/pi5_vtx/frame_enc"
msg "Note: live_sender.py looks for frame_enc next to itself; use --encoder PATH to point elsewhere."

# ---------------------------------------------------------------------------
# Step 4: Flash the TX RP2350
# ---------------------------------------------------------------------------
if [ "$DO_FLASH" -eq 1 ]; then
    msg "Step 4: flash the TX RP2350"
    UF2="$REPO_DIR/prebuilt/firmware/vlive_tx.uf2"
    if [ ! -f "$UF2" ]; then
        err "Firmware not found at $UF2 — cannot proceed with flash instructions."
        exit 1
    fi
    cat <<EOF

Plug the TX RP2350 into the Pi over USB (for flashing; in operation it can stay on USB or use SPI).
  vlive_tx.uf2           boots at 22 dBm: the board needs a proper 5 V supply (HAT / header)
  vlive_tx_lowpower.uf2  boots at -9.5 dBm: for a board powered from the Pi's USB port (22 dBm can trip the Pi's
                         USB over-current protection); live_sender.py --tx-power / tx_power.sh change it later

With picotool (no button needed):   $REPO_DIR/pi5_vtx/flash_tx.sh [--low-power] [--ser <serial no.>]
Without picotool: hold BOOTSEL while plugging the board in, then copy the .uf2 onto the RP2350 drive:
   cp $UF2 /media/\$USER/RP2350/

Check:  ls /dev/serial/by-id/usb-Raspberry_Pi_Pico*

EOF
    if [ "$NON_INTERACTIVE" -eq 0 ]; then
        if command -v picotool >/dev/null 2>&1; then
            read -r -p "Flash now with flash_tx.sh? [y/N/l = low power] " a
            case "$a" in
                y|Y) bash "$REPO_DIR/pi5_vtx/flash_tx.sh" || warn "flash failed; flash it by hand" ;;
                l|L) bash "$REPO_DIR/pi5_vtx/flash_tx.sh" --low-power || warn "flash failed; flash it by hand" ;;
            esac
        fi
        read -r -p "Press Enter once the RP2350 has been flashed and is plugged in (or Ctrl-C to abort): "
    fi
    ok "Flash step complete (verify /dev/serial/by-id/usb-Raspberry_Pi_Pico* exists)."
else
    msg "Step 4: skipping RP2350 flash (--skip-flash)"
fi

# ---------------------------------------------------------------------------
# Step 5: Manual test run — informational
# ---------------------------------------------------------------------------
msg "Step 5: manual test run"
cat <<EOF

To test the sender manually (once the camera and RP2350 are connected):

    python3 -u $REPO_DIR/pi5_vtx/live_sender.py

It finds the TX RP2350 by itself: over SPI if it answers there, else the first free Pico USB port that
answers as a vlive_tx. To choose:

    python3 -u $REPO_DIR/pi5_vtx/live_sender.py --link spi
    python3 -u $REPO_DIR/pi5_vtx/live_sender.py --link usb --serial /dev/serial/by-id/usb-Raspberry_Pi_Pico_<SERIAL>-if00

Common flags (all optional):
  --tx-power N    keep        TX power at start, 0.5 dBm units (-19 .. 44 = -9.5 .. 22 dBm)
  --width/--height 576 320    encode size
  --refresh       120         intra refresh period in frames (no keyframes after the first)
  --fec           off         block FEC K,M[,D], e.g. 16,2 (rebuilds lost packets; costs some bitrate)
  --stamp         off         wall-clock stamps for the receiver's link latency (needs NTP on both ends)
  --source FILE   camera      loop a video file instead of the camera

TX power while it runs:  $REPO_DIR/pi5_vtx/tx_power.sh [N | <x>dbm] [save]
Test video instead of the camera:  $REPO_DIR/pi5_vtx/tx_source.sh FILE   (back: tx_source.sh camera)

EOF
if [ "$NON_INTERACTIVE" -eq 0 ]; then
    read -r -p "Press Enter to continue to systemd setup (or Ctrl-C to abort): "
fi


# ---------------------------------------------------------------------------
# Step 6: Run at boot via systemd
# ---------------------------------------------------------------------------
if [ "$DO_SYSTEMD" -eq 1 ]; then
    msg "Step 6: installing systemd service lora-vlink-tx.service"
    $SUDO cp "$REPO_DIR/pi5_vtx/lora-vlink-tx.service" /etc/systemd/system/
    # (installs made before 2026-11-18 have a drop-in that ran the legacy sender.py: remove it)
    $SUDO rm -f /etc/systemd/system/lora-vlink-tx.service.d/live-single-freq.conf
    if [ ! -f /etc/default/lora-vlink-tx ]; then
        $SUDO cp "$REPO_DIR/pi5_vtx/lora-vlink-tx.default" /etc/default/lora-vlink-tx
        ok "Created /etc/default/lora-vlink-tx (live_sender.py options for the service)"
    else
        ok "Keeping the existing /etc/default/lora-vlink-tx"
    fi

    # Fix user and paths for this machine (the unit hard-codes User=test and
    # /home/test/pi5_vtx). It runs live_sender.py $LORA_TX_ARGS.
    $SUDO sed -i \
        -e "s|^User=test|User=$TARGET_USER|" \
        -e "s|/home/test/pi5_vtx|$REPO_DIR/pi5_vtx|g" \
        /etc/systemd/system/lora-vlink-tx.service

    $SUDO systemctl daemon-reload
    $SUDO systemctl enable --now lora-vlink-tx.service
    ok "systemd service installed, enabled, and started."
    msg "Watch the logs with: sudo journalctl -u lora-vlink-tx -f"
    msg "Day-to-day: sudo systemctl restart lora-vlink-tx (after code changes / reflash)"
    msg "            sudo systemctl stop lora-vlink-tx   (before reflashing the TX RP2350)"
else
    msg "Step 6: skipping systemd setup (--no-systemd)"
fi

# ---------------------------------------------------------------------------
# Done
# ---------------------------------------------------------------------------
msg "Setup complete."
cat <<EOF

Checklist for a new TX Pi:
  [x] packages installed, user in dialout
  [x] repo cloned, frame_enc built
  [ ] TX RP2350 flashed with vlive_tx.uf2 (or vlive_tx_lowpower.uf2) of the same release as the receivers' vlive_rx.uf2
  [ ] camera detected (rpicam-hello --list-cameras)
  [ ] manual test run works
  [x] systemd unit installed with corrected paths, enabled

Reboot (or at least log out and back in) so the dialout group membership takes
effect, then confirm the service is running:

    sudo systemctl status lora-vlink-tx
    sudo journalctl -u lora-vlink-tx -f

EOF

