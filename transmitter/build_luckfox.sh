#!/bin/sh
# Builds the Luckfox transmitter (hz_vtx) and runs the host-side tests.
#
#   LUCKFOX_SDK=~/luckfox-sdk sh transmitter/build_luckfox.sh
#
# Needs the official Luckfox SDK (uclibc toolchain + Rockit / rkaiq headers). The toolchain is x86-64 Linux;
# on an ARM host (Raspberry Pi) set RK_CC to a wrapper that runs it through box64.
# Rockit / rkaiq are linked against the libraries of the target itself: copy /oem/usr/lib/librockit.so and
# /oem/usr/lib/librkaiq.so from the Luckfox into transmitter/build/targetlib/ (install.sh --pull-libs does it).
set -eu
here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
sdk=${LUCKFOX_SDK:-$HOME/luckfox-sdk}
cc=${RK_CC:-$sdk/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/bin/arm-rockchip830-linux-uclibcgnueabihf-gcc}
mkdir -p "$here/build"

# host tests (native compiler): the C transmitter side against the format's reference (format/packet.py)
"${HOST_CC:-cc}" -O2 -Wall -Wextra -Werror "$here/test/test_sched_dump.c" -lpthread -o "$here/build/test_sched_dump"
python3 "$here/test/test_swfec.py" "$here/build/test_sched_dump"
test -e "$here/build/targetlib/librockit.so" || { echo "missing build/targetlib/librockit.so (install.sh --pull-libs)" >&2; exit 1; }
aiq="$sdk/media/isp/release_camera_engine_rkaiq_rv1106_arm-rockchip830-linux-uclibcgnueabihf/include/rkaiq"
# the version stamped into hz_vtx: HZ_GIT (set by luckfox_image/air_build.sh for an exported commit), else this checkout
if [ -n "${HZ_GIT:-}" ]; then git=$HZ_GIT; else
    git=$(git -C "$here" rev-parse --short=8 HEAD 2>/dev/null || echo unknown)
    [ "$git" = unknown ] || [ -z "$(git -C "$here" status --porcelain --untracked-files=no 2>/dev/null)" ] || git="$git-dirty"
fi
"$cc" -O2 -Wall -Wextra -Werror -mcpu=cortex-a7 -mfpu=neon -mfloat-abi=hard -DHZ_GIT="\"$git\"" \
    -I "$sdk/media/rockit/rockit/mpi/sdk/include" \
    -I "$aiq" -I "$aiq/uAPI2" -I "$aiq/common" -I "$aiq/xcore" -I "$aiq/algos" \
    -I "$aiq/iq_parser" -I "$aiq/iq_parser_v2" \
    "$here/main.c" "$here/camera_luckfox.c" -L "$here/build/targetlib" \
    -Wl,--allow-shlib-undefined -lrockit -lrkaiq -lm -lpthread -ldl -o "$here/build/hz_vtx"
# the SPI link test (test/device.py, horizon_pair.sh "T"); no camera libraries
"$cc" -O2 -Wall -Wextra -Werror -mcpu=cortex-a7 -mfpu=neon -mfloat-abi=hard "$here/test/spi_speed_test.c" -o "$here/build/spi_speed_test"
# the radio controller's firmware update over SPI (rp2354_update.sh; SWD is the fallback)
"$cc" -O2 -Wall -Wextra -Werror -mcpu=cortex-a7 -mfpu=neon -mfloat-abi=hard "$here/../luckfox_image/rp2354_spi_update.c" -o "$here/build/rp2354_spi_update"
echo "built $here/build/hz_vtx $here/build/spi_speed_test $here/build/rp2354_spi_update"
