# Contributing

Thanks for helping. Experiments, ports and bug reports are welcome, on any hardware.

## Terms for contributions

By sending a contribution (a pull request, patch, or code in an issue) you agree that:

1. it is your own work, or you have the right to contribute it;
2. you license it to BPYL d.o.o. under the MIT License, so BPYL d.o.o. may use it in any of its products and
   under any license, including this repository's license;
3. it does not include code under a license that would require the whole project to change license (for example
   GPL code).

You keep the copyright in your contribution. Say so in the pull request if a contribution comes under other terms,
and we will discuss it before merging.

## Before a pull request

- `bash test/run.sh all` passes (CI runs it too, with the desktop app's tests); with hardware, also the device you
  changed (`bash test/run.sh` lists them).
- Formatting: `cargo fmt` for Rust (`desktop_receiver/rustfmt.toml`); for C and C++ `.clang-format` applies to new
  and changed code (existing files are not reformatted wholesale); `.editorconfig` for the rest.
- Keep tests apart from the code they test, in a `test/` folder next to it.
