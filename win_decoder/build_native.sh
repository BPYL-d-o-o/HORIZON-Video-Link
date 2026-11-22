#!/bin/sh
# Build the H.265 decoder for the Windows desktop receiver (desktop_receiver/): decoder.c (identical to the browser
# decoder's, web_decoder/) with unmodified FFmpeg libavcodec (HEVC decoder only), plus native_mux.c (libavformat with
# only the mp4 muxer: the app's "Export video"), cross-compiled with MinGW-w64 into one DLL that exports only
# decoder.h's functions and hz_mux_*. Linux or WSL (apt: mingw-w64 nasm). The build is reproducible (no timestamps,
# stripped): the same sources give the same DLL, byte for byte.
#
#   sh win_decoder/build_native.sh [WORK_DIR]
#
# Writes win_decoder/native/hz_decoder.dll (not committed: releases publish it). FFmpeg is LGPL 2.1; this build enables
# no GPL parts, and the DLL is a separate library the app loads at run time. Prints RESULT PASS|FAIL.
set -eu
FFMPEG_VERSION=8.0
HERE=$(cd "$(dirname "$0")" && pwd)
WORK=${1:-$HOME/.cache/horizon-native-decoder}
SOURCE=$WORK/ffmpeg-$FFMPEG_VERSION
CROSS=x86_64-w64-mingw32-
mkdir -p "$WORK" "$HERE/native"
# decoder.c / decoder.h are the same files as web_decoder's (the browser build): where both folders are present they
# must be identical, so the Windows app and the browser decode alike (tools/git-hooks also refuses a commit that differs)
for f in decoder.c decoder.h; do
    other=$HERE/../web_decoder/$f
    if [ -f "$other" ] && ! cmp -s "$HERE/$f" "$other"; then
        echo "FATAL win_decoder/$f differs from web_decoder/$f: they must stay identical"; echo "RESULT FAIL"; exit 1
    fi
done
command -v ${CROSS}gcc >/dev/null || { echo "FATAL ${CROSS}gcc not found (apt install mingw-w64)"; echo "RESULT FAIL"; exit 1; }

if [ ! -f "$SOURCE/configure" ]; then
    echo "EVENT download=ffmpeg-$FFMPEG_VERSION"
    curl -sSfL "https://ffmpeg.org/releases/ffmpeg-$FFMPEG_VERSION.tar.xz" -o "$WORK/ffmpeg.tar.xz"
    tar -xf "$WORK/ffmpeg.tar.xz" -C "$WORK"
fi
# (configured again whenever these flags change)
FLAGS="--enable-cross-compile --target-os=mingw32 --arch=x86_64 --cross-prefix=$CROSS
    --disable-everything --disable-autodetect --enable-decoder=hevc --enable-w32threads
    --enable-muxer=mp4 --enable-protocol=file
    --disable-programs --disable-doc --disable-network
    --disable-avdevice --disable-avfilter --disable-swresample --disable-swscale
    --disable-debug --enable-static --disable-shared --extra-cflags=-O3"
if [ ! -f "$SOURCE/libavformat/libavformat.a" ] || [ "$(cat "$WORK/configure.flags" 2>/dev/null)" != "$FLAGS" ]; then
    echo "EVENT configure"
    (cd "$SOURCE" && { make distclean >/dev/null 2>&1 || true; } && ./configure $FLAGS > "$WORK/configure.log" 2>&1) ||
        { tail -20 "$WORK/configure.log"; echo "RESULT FAIL"; exit 1; }
    echo "EVENT make"
    (cd "$SOURCE" && make -j"$(nproc)" libavcodec/libavcodec.a libavutil/libavutil.a libavformat/libavformat.a \
        > "$WORK/make.log" 2>&1) || { tail -20 "$WORK/make.log"; echo "RESULT FAIL"; exit 1; }
    echo "$FLAGS" > "$WORK/configure.flags"
fi
echo "EVENT link"
cat > "$WORK/hz_decoder.def" <<EOF
EXPORTS
decoder_new
decoder_free
decoder_send
decoder_receive
decoder_copy_bgra
hz_mux_open
hz_mux_write
hz_mux_close
EOF
# relative source paths and the folders mapped out: the DLL does not depend on where the checkout or WORK_DIR is
(cd "$HERE" && ${CROSS}gcc -O2 -shared -ffile-prefix-map="$HERE"=. -ffile-prefix-map="$WORK"=ffmpeg -I. -I"$SOURCE" \
    decoder.c native_mux.c "$WORK/hz_decoder.def" \
    "$SOURCE/libavformat/libavformat.a" "$SOURCE/libavcodec/libavcodec.a" "$SOURCE/libavutil/libavutil.a" \
    -static-libgcc -Wl,--no-insert-timestamp -lbcrypt -o native/hz_decoder.dll)
${CROSS}strip "$HERE/native/hz_decoder.dll"
exports=$(${CROSS}objdump -p "$HERE/native/hz_decoder.dll" | sed -n '/\[Ordinal\/Name Pointer\] Table/,/^$/p' | grep -cE 'decoder_|hz_mux_' || true)
[ "$exports" = 8 ] || { echo "FATAL the DLL exports $exports of its 8 functions"; echo "RESULT FAIL"; exit 1; }
ls -l "$HERE/native/hz_decoder.dll"
echo "RESULT PASS"
