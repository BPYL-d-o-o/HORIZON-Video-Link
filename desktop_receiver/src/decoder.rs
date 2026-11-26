//! The H.265 decoder: decoder/decoder.c (shared with the browser's) with unmodified FFmpeg, built as hz_decoder.dll
//! (decoder/native/build_native.sh). The executable carries it (build.rs) and writes it out once to
//! %LOCALAPPDATA%\HorizonReceiver\decoder; a hz_decoder.dll next to the executable is used instead when present
//! (a development build, or a user's own build of the library: FFmpeg is LGPL).
use libloading::{Library, Symbol};
use std::ffi::c_void;
use std::path::PathBuf;
use std::sync::OnceLock;

static EMBEDDED: &[u8] = include_bytes!(env!("HZ_DECODER_EMBED"));

/// Where hz_decoder.dll is loaded from (found or written once per run).
fn dll_path() -> Result<PathBuf, String> {
    static PATH: OnceLock<Result<PathBuf, String>> = OnceLock::new();
    PATH.get_or_init(|| {
        let exe = std::env::current_exe().map_err(|e| e.to_string())?;
        let beside = exe.parent().unwrap().join("hz_decoder.dll");
        if beside.is_file() {
            return Ok(beside);
        }
        if EMBEDDED.is_empty() {
            return Err(format!(
                "no decoder: this build carries none and there is no {}",
                beside.display()
            ));
        }
        let base = std::env::var_os("LOCALAPPDATA")
            .map(PathBuf::from)
            .unwrap_or_else(std::env::temp_dir);
        let dir = base.join("HorizonReceiver").join("decoder");
        let path = dir.join(format!("hz_decoder-{}.dll", env!("HZ_DECODER_HASH")));
        let ok = std::fs::metadata(&path)
            .map(|m| m.len() == EMBEDDED.len() as u64)
            .unwrap_or(false);
        if !ok {
            std::fs::create_dir_all(&dir).map_err(|e| format!("cannot create {}: {e}", dir.display()))?;
            // a whole file or none: written beside, then renamed (another copy of the app may be starting too)
            let tmp = dir.join(format!(
                "hz_decoder-{}.{}.tmp",
                env!("HZ_DECODER_HASH"),
                std::process::id()
            ));
            std::fs::write(&tmp, EMBEDDED).map_err(|e| format!("cannot write {}: {e}", tmp.display()))?;
            if std::fs::rename(&tmp, &path).is_err() {
                let _ = std::fs::remove_file(&tmp);
                if !path.is_file() {
                    return Err(format!("cannot write {}", path.display()));
                }
            }
        }
        Ok(path)
    })
    .clone()
}

type New = unsafe extern "C" fn() -> *mut c_void;
type Free = unsafe extern "C" fn(*mut c_void);
type Send = unsafe extern "C" fn(*mut c_void, *const u8, i32) -> i32;
type Receive = unsafe extern "C" fn(*mut c_void, *mut i32, *mut i32) -> i32;
type CopyBgra = unsafe extern "C" fn(*const c_void, *mut u8, i32);

pub struct Decoder {
    _lib: Library,
    handle: *mut c_void,
    free: Free,
    send: Send,
    receive: Receive,
    copy: CopyBgra,
}
// The handle is only used from the decode thread that owns the Decoder.
unsafe impl std::marker::Send for Decoder {}

impl Decoder {
    pub fn load() -> Result<Decoder, String> {
        let path = dll_path()?;
        unsafe {
            let lib = Library::new(&path).map_err(|e| format!("cannot load {}: {e}", path.display()))?;
            let new: Symbol<New> = lib.get(b"decoder_new").map_err(|e| e.to_string())?;
            let free = *lib.get::<Free>(b"decoder_free").map_err(|e| e.to_string())?;
            let send = *lib.get::<Send>(b"decoder_send").map_err(|e| e.to_string())?;
            let receive = *lib.get::<Receive>(b"decoder_receive").map_err(|e| e.to_string())?;
            let copy = *lib.get::<CopyBgra>(b"decoder_copy_bgra").map_err(|e| e.to_string())?;
            let handle = new();
            if handle.is_null() {
                return Err("decoder_new failed".into());
            }
            Ok(Decoder {
                _lib: lib,
                handle,
                free,
                send,
                receive,
                copy,
            })
        }
    }
    /// One access unit (Annex-B). Then `picture()` for the newest decoded picture, if any.
    pub fn send(&mut self, au: &[u8]) -> bool {
        unsafe { (self.send)(self.handle, au.as_ptr(), au.len() as i32) >= 0 }
    }
    /// The newest picture as BGRA (width, height, pixels with pitch width*4), if a new one is ready.
    pub fn picture(&mut self, into: &mut Vec<u8>) -> Option<(u32, u32)> {
        let (mut w, mut h) = (0i32, 0i32);
        unsafe {
            if (self.receive)(self.handle, &mut w, &mut h) != 1 || w <= 0 || h <= 0 {
                return None;
            }
            into.resize(w as usize * h as usize * 4, 0);
            (self.copy)(self.handle, into.as_mut_ptr(), w * 4);
        }
        Some((w as u32, h as u32))
    }
}
impl Drop for Decoder {
    fn drop(&mut self) {
        unsafe { (self.free)(self.handle) }
    }
}

type MuxOpen = unsafe extern "C" fn(*const i8, *const u8, i32, i32, i32) -> *mut c_void;
type MuxWrite = unsafe extern "C" fn(*mut c_void, *const u8, i32, i64, i32) -> i32;
type MuxClose = unsafe extern "C" fn(*mut c_void) -> i32;

/// An .mp4 being written (native_mux.c in hz_decoder.dll): the received access units as they are, no re-encoding.
pub struct Mux {
    _lib: Library,
    handle: *mut c_void,
    write: MuxWrite,
    close: MuxClose,
}

impl Mux {
    /// `extradata`: the stream's VPS/SPS/PPS in Annex-B.
    pub fn create(path: &std::path::Path, extradata: &[u8], width: u32, height: u32) -> Result<Mux, String> {
        let dll = dll_path()?;
        let cpath = std::ffi::CString::new(path.to_string_lossy().as_bytes()).map_err(|e| e.to_string())?;
        unsafe {
            let lib = Library::new(&dll).map_err(|e| format!("cannot load {}: {e}", dll.display()))?;
            let open = *lib.get::<MuxOpen>(b"hz_mux_open").map_err(|_| {
                "hz_decoder.dll without hz_mux_open: rebuild it (decoder/native/build_native.sh)".to_string()
            })?;
            let write = *lib.get::<MuxWrite>(b"hz_mux_write").map_err(|e| e.to_string())?;
            let close = *lib.get::<MuxClose>(b"hz_mux_close").map_err(|e| e.to_string())?;
            let handle = open(
                cpath.as_ptr(),
                extradata.as_ptr(),
                extradata.len() as i32,
                width as i32,
                height as i32,
            );
            if handle.is_null() {
                return Err(format!("cannot write {}", path.display()));
            }
            Ok(Mux {
                _lib: lib,
                handle,
                write,
                close,
            })
        }
    }
    pub fn write(&mut self, au: &[u8], time_us: i64, key: bool) -> bool {
        unsafe { (self.write)(self.handle, au.as_ptr(), au.len() as i32, time_us, key as i32) >= 0 }
    }
    /// Finishes the file; false if it could not be completed.
    pub fn finish(mut self) -> bool {
        let h = std::mem::replace(&mut self.handle, std::ptr::null_mut());
        unsafe { (self.close)(h) == 0 }
    }
}
impl Drop for Mux {
    fn drop(&mut self) {
        if !self.handle.is_null() {
            unsafe {
                (self.close)(self.handle);
            }
        }
    }
}
// Only used from the thread that owns it.
unsafe impl std::marker::Send for Mux {}
