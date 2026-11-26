# H.265 decoder

The receivers' H.265 decoder: an unmodified FFmpeg 8.0 libavcodec (HEVC decoder only) behind a small C wrapper, built
for two places.

- `decoder.c`, `decoder.h`: H.265 decoding through FFmpeg, shared by both builds, so the Windows app and the browser
  decode alike. It keeps showing damaged pictures (`AV_CODEC_FLAG_OUTPUT_CORRUPT`, `AV_CODEC_FLAG2_SHOW_ALL`, low
  delay) and converts each picture to BGRA.
- `native/`: `hz_decoder.dll` for Horizon Receiver for Windows (`desktop_receiver/`), which carries the DLL inside its
  executable.
  - `native_mux.c`: libavformat with only the mp4 muxer, for the app's "Export video".
  - `build_native.sh`: downloads FFmpeg 8.0, cross-compiles it with MinGW-w64 and links `build/hz_decoder.dll`,
    which exports only `decoder.h`'s functions and `hz_mux_*`.
- `wasm/` (where present): the setup site's viewer, FFmpeg compiled to WebAssembly, so nothing is installed on the
  viewing computer.
  - `wasm_decoder.c`: the functions the page calls (`wd_open`, `wd_input`, `wd_decode`, `wd_pixels`, `wd_width`,
    `wd_height`). The page copies one access unit into the input buffer, calls `wd_decode`, and reads RGBA pixels
    when it returns 1.
  - `build_wasm.sh`: downloads FFmpeg 8.0, builds only the HEVC decoder with Emscripten, and writes `build/hevc.js`
    and `build/hevc.wasm`.
- `build/`: the build products (`hz_decoder.dll`, `hevc.js`, `hevc.wasm`). Releases publish them; git ignores them.

Why not a ready-made FFmpeg: the usual Windows builds are the full GPL ones (around 100 MB), browsers do not decode
H.265 reliably and never show damaged pictures, and Windows' own HEVC decoder needs a paid extension.

## Build (Linux or WSL)

```sh
sudo apt install mingw-w64 nasm
sh decoder/native/build_native.sh

git clone https://github.com/emscripten-core/emsdk ~/emsdk
~/emsdk/emsdk install latest && ~/emsdk/emsdk activate latest
. ~/emsdk/emsdk_env.sh && sh decoder/wasm/build_wasm.sh
```

The first build of each compiles FFmpeg (a few minutes); later builds only link. Both print `RESULT PASS` or
`RESULT FAIL`. The DLL build is reproducible: no timestamps, no build paths, stripped. The same sources give the same
DLL, byte for byte, from any folder.

## License

FFmpeg is LGPL 2.1. Neither build enables GPL parts (`--disable-everything --enable-decoder=hevc`, no
`--enable-gpl`), and the DLL is a separate library the app loads at run time. Anyone publishing `hz_decoder.dll` or
`hevc.wasm` must also offer the FFmpeg source and the build script.
