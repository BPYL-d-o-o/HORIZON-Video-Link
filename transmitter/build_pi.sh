#!/bin/sh
# Builds hz_vtx for the Raspberry Pi 5 + Horizon HAT, on the Pi itself or in the Pi image's chroot
# (pi_image/build_pi_image.sh): hz_vtx and its encoder hz_x265, which must sit next to it. Needs gcc, g++,
# pkg-config, libcamera-dev and libx265-dev.
#
#   sh build_pi.sh [OUT_DIR]          (default: build/pi next to this script; HZ_GIT=<commit> stamps the version)
set -eu
here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
out=${1:-$here/build/pi}
mkdir -p "$out"
if [ -n "${HZ_GIT:-}" ]; then git=$HZ_GIT; else
    git=$(git -C "$here" rev-parse --short=8 HEAD 2>/dev/null || echo unknown)
    [ "$git" = unknown ] || [ -z "$(git -C "$here" status --porcelain --untracked-files=no 2>/dev/null)" ] || git="$git-dirty"
fi
gcc -O2 -Wall -Wextra -Werror -DHZ_PI -DHZ_GIT="\"$git\"" -c "$here/main.c" -o "$out/main.o"
g++ -O2 -std=c++20 -Wall -Wextra -Werror -DHZ_PI $(pkg-config --cflags libcamera) -c "$here/camera_pi.cpp" \
    -o "$out/camera_pi.o"
g++ -o "$out/hz_vtx" "$out/main.o" "$out/camera_pi.o" $(pkg-config --libs libcamera) -lpthread -lm
# the encoder: its own program, so hz_vtx does not link x265 (GPL)
gcc -O2 -Wall -Wextra -Werror "$here/x265_encoder.c" -o "$out/hz_x265" -lx265
echo "built $out/hz_vtx $out/hz_x265"
