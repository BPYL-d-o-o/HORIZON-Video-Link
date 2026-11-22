use std::process::Command;

/// The Ground firmware this app accepts as up to date: every commit whose firmware sources are the same as HEAD's
/// (the paths the git hooks fingerprint, tools/git-hooks/air_hooks.sh INPUT_PATHS). A Ground reports the commit it was
/// built from, and a build from a later commit that did not touch the firmware is the same firmware.
/// HZ_GROUND_FW (comma-separated, newest first) overrides it.
const FW_PATHS: &[&str] = &[
    "lora_vlink/src", "pi5_vtx/swfec.py", "lora_vlink/CMakeLists.txt", "lora_vlink/pico_sdk_import.cmake",
    "lora_vlink/third_party", "luckfox_vtx", "luckfox_image",
];

fn git(args: &[&str]) -> Option<String> {
    let o = Command::new("git").current_dir("..").args(args).output().ok()?;
    o.status.success().then(|| String::from_utf8_lossy(&o.stdout).trim().to_string())
}

fn accepted() -> Vec<String> {
    let mut last_args = vec!["log", "-1", "--format=%H", "--"];
    last_args.extend(FW_PATHS);
    let Some(last) = git(&last_args).filter(|s| s.len() >= 8) else { return Vec::new() };
    let mut out = Vec::new();
    for c in git(&["rev-list", &format!("{last}..HEAD")]).unwrap_or_default().lines() {
        let mut diff = vec!["diff", "--quiet", last.as_str(), c, "--"];
        diff.extend(FW_PATHS);
        if git(&diff).is_some() {
            out.push(c[..8].to_string());
        }
    }
    out.push(last[..8].to_string());
    out
}

fn main() {
    let fw = std::env::var("HZ_GROUND_FW").ok().filter(|s| !s.is_empty()).unwrap_or_else(|| {
        let a = accepted();
        if a.is_empty() { "unknown".into() } else { a.join(",") }
    });
    println!("cargo:rustc-env=HZ_GROUND_FW={fw}");
    println!("cargo:rerun-if-env-changed=HZ_GROUND_FW");
    println!("cargo:rerun-if-changed=../.git/logs/HEAD"); // every commit and checkout
    embed_decoder();
    tauri_build::build()
}

/// hz_decoder.dll (win_decoder/build_native.sh) goes inside the executable: src/decoder.rs writes it out once and
/// loads it. HZ_DECODER_DLL (a path) overrides where it is taken from. Without it the build still works (a
/// development build next to a hz_decoder.dll), and the app says so if it has none.
fn embed_decoder() {
    let path = std::env::var("HZ_DECODER_DLL").ok().filter(|s| !s.is_empty())
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| std::path::Path::new("../win_decoder/native/hz_decoder.dll").to_path_buf());
    println!("cargo:rerun-if-env-changed=HZ_DECODER_DLL");
    println!("cargo:rerun-if-changed={}", path.display());
    let out = std::path::PathBuf::from(std::env::var("OUT_DIR").unwrap()).join("hz_decoder.dll");
    let bytes = std::fs::read(&path).unwrap_or_default();
    if bytes.is_empty() {
        println!("cargo:warning=no {} to embed: the app needs a hz_decoder.dll next to it", path.display());
    }
    // FNV-1a 64: names the written-out copy, so a new build never loads an older decoder
    let hash = bytes.iter().fold(0xcbf29ce484222325u64, |h, &b| (h ^ b as u64).wrapping_mul(0x100000001b3));
    std::fs::write(&out, &bytes).unwrap();
    println!("cargo:rustc-env=HZ_DECODER_EMBED={}", out.display());
    println!("cargo:rustc-env=HZ_DECODER_HASH={hash:016x}");
}
