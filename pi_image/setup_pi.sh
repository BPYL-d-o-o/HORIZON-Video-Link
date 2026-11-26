#!/bin/sh
# Sets up a Raspberry Pi 5 with Raspberry Pi OS (64-bit) and a Horizon HAT as a Horizon transmitter, from a checkout
# of this repository (the Horizon Pi image, pi_image/build_pi_image.sh, comes this way ready-made):
#
#   sh pi_image/setup_pi.sh            (as a user with sudo)
#
# Installs the build packages, turns SPI on, builds hz_vtx and its encoder hz_x265 (transmitter/build_pi.sh) into
# /opt/horizon, and installs the horizon_vtx service (user horizon), which starts at boot. The HAT needs its
# transmitter firmware: flash it on the setup site (Air unit, Raspberry Pi 5 + Horizon HAT) or with
# pi_image/flash_tx.sh. Prints RESULT PASS|FAIL.
set -eu
here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repo=$(dirname "$here")
fail() { echo "FATAL $*"; echo "RESULT FAIL"; exit 1; }

echo "EVENT packages"
sudo apt-get update -qq && sudo apt-get install -y -qq gcc g++ pkg-config libcamera-dev libx265-dev >/dev/null ||
    fail "apt-get install"
echo "EVENT SPI on (takes effect after a reboot)"
sudo raspi-config nonint do_spi 0 || fail "raspi-config: enable SPI"
echo "EVENT building hz_vtx"
sudo mkdir -p /opt/horizon
sudo sh "$repo/transmitter/build_pi.sh" /opt/horizon/build >/dev/null || fail "transmitter/build_pi.sh"
sudo install -m 755 /opt/horizon/build/hz_vtx /opt/horizon/build/hz_x265 "$here/tx_power.sh" "$here/tx_source.sh" /opt/horizon/
sudo rm -rf /opt/horizon/build
echo "EVENT service horizon_vtx"
id horizon >/dev/null 2>&1 ||
    sudo useradd --system --no-create-home --home-dir /opt/horizon --shell /usr/sbin/nologin horizon
for g in video spi gpio; do getent group $g >/dev/null && sudo usermod -aG $g horizon; done
for old in lora-vlink-tx vlink-tx; do  # the service's earlier names: their options carry over
    [ -f /etc/systemd/system/$old.service ] || continue
    sudo systemctl disable --now $old >/dev/null 2>&1 || true
    sudo rm -f /etc/systemd/system/$old.service
    sudo systemctl reset-failed $old >/dev/null 2>&1 || true
    [ -f /etc/default/$old ] && [ ! -f /etc/default/horizon_vtx ] &&
        sudo sh -c "sed 's/^[A-Z_]*_TX_ARGS=/HORIZON_VTX_ARGS=/' /etc/default/$old >/etc/default/horizon_vtx"
    sudo rm -f /etc/default/$old
done
sudo install -m 644 "$here/horizon_vtx.service" /etc/systemd/system/horizon_vtx.service
[ -f /etc/default/horizon_vtx ] || sudo install -m 644 "$here/horizon_vtx.default" /etc/default/horizon_vtx
sudo systemctl daemon-reload && sudo systemctl enable horizon_vtx >/dev/null 2>&1 || fail "enable horizon_vtx"
echo "EVENT done: reboot, then journalctl -u horizon_vtx -f shows the transmitter"
echo "RESULT PASS"
