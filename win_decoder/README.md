# Windows H.265 decoder

`hz_decoder.dll`, the H.265 decoder of Horizon Receiver for Windows (`desktop_receiver/`), built from an unmodified
FFmpeg 8.0 libavcodec (HEVC decoder only). The app carries the DLL inside its executable.

- `decoder.c`, `decoder.h`: H.265 decoding through FFmpeg. It keeps showing damaged pictures
  (`AV_CODEC_FLAG_OUTPUT_CORRUPT`, `AV_CODEC_FLAG2_SHOW_ALL`, low delay) and converts each picture to BGRA. The
  browser decoder (`web_decoder/`, where present) uses the same two files: they must stay identical, and
  `build_native.sh` (and the browser's `build_wasm.sh`) refuse to build when they differ.
- `native_mux.c`: libavformat with only the mp4 muxer, for the app's "Export video".
- `build_native.sh`: downloads FFmpeg 8.0, cross-compiles it with MinGW-w64 and links `native/hz_decoder.dll`,
  which exports only `decoder.h`'s functions and `hz_mux_*`.

## Build (Linux or WSL)

```sh
sudo apt install mingw-w64 nasm
sh win_decoder/build_native.sh
```

The first build compiles FFmpeg (a few minutes); later builds only link. The build is reproducible: no timestamps,
no build paths, stripped. The same sources give the same DLL, byte for byte, from any folder.

## License

FFmpeg is LGPL 2.1. This build enables no GPL parts (`--disable-everything --enable-decoder=hevc`, no
`--enable-gpl`), and the DLL is a separate library the app loads at run time. Anyone publishing `hz_decoder.dll` must
also offer the FFmpeg source and this build script.
