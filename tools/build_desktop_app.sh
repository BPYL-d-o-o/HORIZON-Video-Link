#!/bin/sh
# Builds Horizon Receiver for Windows (desktop_receiver/, Tauri + Rust, with decoder/'s hz_decoder.dll) from a git
# tree, as ONE executable. Run from Git Bash on Windows with the Rust MSVC toolchain (stable-x86_64-pc-windows-msvc,
# Visual Studio Build Tools with the C++ workload: it links Microsoft's WebView2 loader into the executable) and WSL
# (the decoder DLL is cross-built there with MinGW: decoder/native/build_native.sh; the executable carries it, build.rs).
#
#   sh tools/build_desktop_app.sh check   TREE-ISH                    compile only (the pre-commit check)
#   sh tools/build_desktop_app.sh release TREE-ISH VERSION OUT_DIR    build, smoke-test (--shot), for the setup site:
#        OUT_DIR/HorizonReceiver-VERSION-windows.exe and OUT_DIR/app.json ({file, version, size, sha256, bytes_unpacked})
#
# The sources are exported from git into $DESKTOP_BUILD/src (unstaged edits are not part of it); Cargo's target folder
# ($DESKTOP_BUILD/target) stays between builds, so a build after a small change takes about 20 s.
# HZ_GROUND_FW (the Ground firmware commits the app accepts, newest first) is taken from git here: an exported tree has
# no history for desktop_receiver/build.rs to read. Prints EVENT lines and RESULT PASS|FAIL.
set -eu
mode=${1:?usage: build_desktop_app.sh check|release TREE-ISH [VERSION OUT_DIR]}
tree=${2:?usage: build_desktop_app.sh check|release TREE-ISH [VERSION OUT_DIR]}
build=${DESKTOP_BUILD:-$HOME/horizon-build/desktop}  # not AppData: packaged apps get a private copy of it
fail() { echo "FATAL $*"; echo "RESULT FAIL"; exit 1; }
command -v cargo >/dev/null || fail "cargo not found (Rust, with the stable-x86_64-pc-windows-msvc toolchain)"
TOOLCHAIN=${TOOLCHAIN:-stable-x86_64-pc-windows-msvc}
command -v wsl.exe >/dev/null || fail "WSL not found (the decoder DLL is built there)"
repo=$(git rev-parse --show-toplevel)
t0=$(date +%s)

# the Ground firmware this build accepts: every commit whose firmware sources are the same as the tree's
# (the paths tools/git-hooks/air_hooks.sh fingerprints; build.rs does the same in a checkout)
FW_PATHS="radio_firmware/src format radio_firmware/CMakeLists.txt radio_firmware/pico_sdk_import.cmake radio_firmware/third_party transmitter luckfox_image"
commit=$(git rev-parse --verify -q "$tree^{commit}" 2>/dev/null || git rev-parse HEAD)
last=$(git log -1 --format=%H "$commit" -- $FW_PATHS)
fw=
if [ -n "$last" ]; then
    for c in $(git rev-list "$last..$commit"); do
        git diff --quiet "$last" "$c" -- $FW_PATHS && fw="$fw,$(echo "$c" | cut -c1-8)"
    done
    fw="${fw#,}${fw:+,}$(echo "$last" | cut -c1-8)"
fi
export HZ_GROUND_FW=${fw:-unknown}

# the sources from git, in place of the previous ones (Cargo's target folder is elsewhere and stays)
src=$build/src
rm -rf "$src"; mkdir -p "$src" "$build/target"
git -c core.autocrlf=false archive --format=tar "$tree" -- desktop_receiver decoder/decoder.c decoder/decoder.h decoder/native | tar -x -C "$src"

echo "EVENT decoder dll"
wsrc=$(wsl.exe --exec wslpath -u "$(cygpath -w "$src")" | tr -d '\r')
MSYS_NO_PATHCONV=1 wsl.exe --exec sh "$wsrc/decoder/native/build_native.sh" >"$build/dll.log" 2>&1 || { tail -5 "$build/dll.log"; fail "decoder/native/build_native.sh (see $build/dll.log)"; }
grep -q '^RESULT PASS' "$build/dll.log" || fail "decoder/native/build_native.sh (see $build/dll.log)"

echo "EVENT cargo build (Ground firmware accepted: $HZ_GROUND_FW)"
(cd "$src/desktop_receiver" && CARGO_TARGET_DIR=$(cygpath -w "$build/target") cargo +"$TOOLCHAIN" build --release \
    >"$build/cargo.log" 2>&1) || { grep -E '^error' -A6 "$build/cargo.log" | head -30; fail "cargo build (see $build/cargo.log)"; }
rel=$build/target/release
if [ "$mode" = check ]; then
    echo "EVENT built $(( $(date +%s) - t0 )) s"
    echo "RESULT PASS"; exit 0
fi
[ "$mode" = release ] || fail "unknown mode $mode"
ver=${3:?usage: build_desktop_app.sh release TREE-ISH VERSION OUT_DIR}
out=${4:?usage: build_desktop_app.sh release TREE-ISH VERSION OUT_DIR}

# what users get: the executable alone (the WebView2 loader is linked in, the decoder carried inside)
pkg=$build/pkg
rm -rf "$pkg"; mkdir -p "$pkg"
cp "$rel/horizon_receiver.exe" "$pkg/HorizonReceiver.exe"
objdump -p "$pkg/HorizonReceiver.exe" 2>/dev/null | grep -qi 'DLL Name: WebView2Loader' &&
    fail "the executable still needs WebView2Loader.dll (built with a GNU toolchain? TOOLCHAIN=$TOOLCHAIN)"

# it must start and draw from a folder with nothing else in it: a frame saved from a hidden window, no receiver
# needed (it shows "no receiver"); it writes its decoder out to %LOCALAPPDATA%\HorizonReceiver\decoder
echo "EVENT smoke test"
rm -f "$build/shot.bmp"
timeout 60 "$pkg/HorizonReceiver.exe" --shot "$(cygpath -w "$build/shot.bmp")" --shot-after 3 >"$build/shot.log" 2>&1 || true
grep -q '^RESULT PASS' "$build/shot.log" && [ -s "$build/shot.bmp" ] || fail "the built app does not start (see $build/shot.log)"

mkdir -p "$out"
name=HorizonReceiver-$ver-windows.exe
cp "$pkg/HorizonReceiver.exe" "$out/$name"
size=$(stat -c %s "$out/$name")
sha=$(sha256sum "$out/$name" | cut -d' ' -f1)
unpacked=$size
printf '{"file": "%s", "version": "%s", "size": %s, "sha256": "%s", "bytes_unpacked": %s}\n' "$name" "$ver" "$size" "$sha" "$unpacked" >"$out/app.json"
echo "EVENT app $out/$name ($size bytes, $(( $(date +%s) - t0 )) s)"
echo "RESULT PASS"
