//! Horizon Receiver: the Ground's packets in, video on the screen with as little delay as the machine allows, plus the
//! Settings window (a web UI, ui/) for the link, recordings and opened flights. Threads:
//!   serial   reads the Ground (USB serial), runs the receiver engine (engine.rs), statistics (stats.rs), recording
//!            (session.rs), and sends the Settings window's commands to the Ground
//!   replay   while a recorded flight is open: its packets, paced, through the same engine (replay.rs)
//!   decode   decodes every access unit of the source shown (live or the replay), keeps the newest picture
//!   render   presents the newest picture as soon as it is decoded (render.rs: flip model, one frame queued)
//!   stats    samples the statistics twice a second: history, stats.csv, the bar, the Settings window
//! The video window is the app: one full-screen swap chain with the bar drawn into its top 10 % and the video below,
//! so Windows flips it straight to the screen. The Settings window is separate: a web view is always composed.
#[cfg(windows)]
pub mod decoder;
pub mod diag;
pub mod engine;
#[cfg(windows)]
mod export;
pub mod hold;
#[cfg(windows)]
mod render;
pub mod replay;
pub mod session;
pub mod stats;

#[cfg(windows)]
pub use app::run;

#[cfg(windows)]
mod app {
    use crate::decoder::Decoder;
    use crate::engine::{self, IdentAssembler, Line, Stream, StreamSink};
    use crate::render::*;
    use crate::replay;
    use crate::session::{self, unix_now, Recorder};
    use crate::stats::{Sample, Stats, STAT_FIELDS};
    use serde::Serialize;
    use std::collections::VecDeque;
    use std::io::{Read, Write};
    use std::sync::atomic::{AtomicBool, AtomicIsize, AtomicU64, Ordering};
    use std::sync::mpsc::{channel, sync_channel, Receiver, Sender, SyncSender};
    use std::sync::{Arc, Condvar, Mutex, OnceLock};
    use std::time::Duration;
    use tauri::{Emitter, Manager, WebviewWindow};
    use windows::core::w;
    use windows::Win32::Foundation::{HWND, LPARAM, LRESULT, RECT, WPARAM};
    use windows::Win32::Graphics::Gdi::{
        GetMonitorInfoW, GetStockObject, MonitorFromPoint, MonitorFromWindow, BLACK_BRUSH, HBRUSH, HMONITOR,
        MONITORINFO, MONITOR_DEFAULTTONEAREST, MONITOR_DEFAULTTOPRIMARY,
    };
    use windows::Win32::System::LibraryLoader::GetModuleHandleW;
    use windows::Win32::System::Performance::{QueryPerformanceCounter, QueryPerformanceFrequency};
    use windows::Win32::System::Threading::{AttachThreadInput, GetCurrentThreadId};
    use windows::Win32::UI::Input::KeyboardAndMouse::{SetFocus, VK_ESCAPE, VK_LEFT, VK_RIGHT, VK_SPACE};
    use windows::Win32::UI::WindowsAndMessaging::*;

    const GROUND_VID: u16 = 0x2E8A;
    const HISTORY: usize = 6 * 3600 * 2; // six hours of samples, twice a second

    fn qpc() -> i64 {
        let mut t = 0;
        unsafe {
            let _ = QueryPerformanceCounter(&mut t);
        }
        t
    }
    fn qpc_ms(d: i64) -> f64 {
        static F: OnceLock<f64> = OnceLock::new();
        let f = *F.get_or_init(|| {
            let mut f = 0;
            unsafe {
                let _ = QueryPerformanceFrequency(&mut f);
            }
            f as f64
        });
        d as f64 * 1000.0 / f
    }
    fn mmss(s: f64) -> String {
        let s = s.max(0.0) as u64;
        if s >= 3600 {
            format!("{}:{:02}:{:02}", s / 3600, s / 60 % 60, s % 60)
        } else {
            format!("{:02}:{:02}", s / 60, s % 60)
        }
    }

    #[derive(Clone, Default, Serialize)]
    struct Cfg {
        freq_hz: u64,
        pwr_dbm: f32,
        locked: bool,
        hop: Option<(u32, u64)>,
    }
    #[derive(Default)]
    struct Live {
        stats: Stats,
        port: String,
        error: String,
        who: Vec<(String, String)>,
        cfg: Option<Cfg>,
        profile: Option<String>,
        ident: Option<(f64, String)>,
        status: String,
        lost: u64,
        repaired: u64,
        /// video packets rebuilt by the block FEC (not in stats.csv)
        fec_recovered: u64,
        /// what is wrong with the Grounds plugged in: one that does not open, firmware not up to date
        problems: Vec<String>,
        /// the receiving Ground's own firmware problem (from its W reply)
        fw_problem: Option<String>,
        /// for diag.rs: the newest packet of any kind, video packet, Air status (unix seconds, 0 = none)
        last_heard: f64,
        /// when packets started arriving after a gap of a second or more (diag.rs: no notice while the link comes up)
        link_since: f64,
        last_video: f64,
        air: Option<(f64, engine::AirStatus)>,
        /// the newest packet from another system (another link ID), and its link
        foreign: Option<(f64, u16)>,
    }
    #[derive(Default, Clone, Serialize)]
    struct RenderInfo {
        gpu: String,
        mode: String,
        screen: String,
        tearing: bool,
        error: String,
    }
    struct ReplayCtl {
        tx: Sender<replay::Cmd>,
        state: Arc<Mutex<replay::State>>,
        title: String,
        /// the flight's statistics, for the bar's values at the cursor: time, and rssi/snr/lq/fps/kbps
        time: Vec<f64>,
        readout: Vec<(&'static str, &'static str, Vec<Option<f64>>)>,
        marks: Vec<f64>,
    }
    struct Latest {
        seq: u64,
        w: u32,
        h: u32,
        px: Vec<u8>,
        arrival: i64,
        decoded: i64,
    }
    struct DecMsg {
        au: Vec<u8>,
        /// the first access unit after lost packets: its picture has grey areas until the row refresh
        after_gap: bool,
        arrival: i64,
        show: bool,
        live: bool,
        epoch: u64,
    }
    struct Shared {
        live: Mutex<Live>,
        history: Mutex<VecDeque<Sample>>,
        latest: Mutex<Latest>,
        cv: Condvar,
        bar_shown: AtomicBool,
        bar: Mutex<Bar>,
        bar_dirty: AtomicBool,
        /// which source the decoder follows: bumped on every switch between live and replay, and on every seek
        epoch: AtomicU64,
        replaying: AtomicBool,
        replay: Mutex<Option<ReplayCtl>>,
        recorder: Mutex<Option<Recorder>>,
        outbox: Mutex<Vec<String>>,
        render: Mutex<RenderInfo>,
        dec_tx: SyncSender<DecMsg>,
        /// developer: injected packet loss (share 0..1, average burst length in slots); never saved, off at start
        inject: Mutex<(f64, f64)>,
        /// the link warning shown as an outline around the video: (0 none, 1 weak, 2 critical, why)
        alert: Mutex<(u8, String)>,
        /// unix time of the newest live picture (f64 bits)
        last_picture: AtomicU64,
        /// "why is there no picture?" over the video (diag.rs): (title, explanation)
        notice: Mutex<Option<(String, String)>>,
    }

    /// What the Settings window changes that outlives the app: %APPDATA%\HorizonReceiver\settings.json.
    #[derive(Clone, Serialize, serde::Deserialize)]
    #[serde(default)]
    struct Prefs {
        /// 0 as the camera sends it, 3 rotated 180° (the renderer's mirror bits). Never mirrored: never right for a camera.
        flip: u8,
        out_dir: Option<String>,
        /// the middle of the monitor the video was last on (desktop coordinates): it starts there again
        monitor: Option<(i32, i32)>,
    }
    impl Default for Prefs {
        fn default() -> Self {
            Prefs {
                flip: 0,
                out_dir: None,
                monitor: None,
            }
        }
    }
    static PREFS: Mutex<Option<Prefs>> = Mutex::new(None);
    fn prefs_path() -> std::path::PathBuf {
        std::path::PathBuf::from(std::env::var_os("APPDATA").unwrap_or_default())
            .join("HorizonReceiver")
            .join("settings.json")
    }
    fn prefs() -> Prefs {
        let mut g = PREFS.lock().unwrap();
        let mut p = g
            .get_or_insert_with(|| {
                std::fs::read(prefs_path())
                    .ok()
                    .and_then(|b| serde_json::from_slice(&b).ok())
                    .unwrap_or_default()
            })
            .clone();
        if p.flip != 3 {
            p.flip = 0;
        }
        p
    }
    fn save_prefs(p: Prefs) {
        *session::OUT_DIR.lock().unwrap() = p.out_dir.clone().map(Into::into);
        let path = prefs_path();
        if let Some(d) = path.parent() {
            let _ = std::fs::create_dir_all(d);
        }
        let _ = std::fs::write(&path, serde_json::to_vec_pretty(&p).unwrap_or_default());
        *PREFS.lock().unwrap() = Some(p);
        if let Some(sh) = SHARED.get() {
            sh.bar_dirty.store(true, Ordering::Relaxed);
            sh.cv.notify_all();
        }
    }

    static WINDOW: OnceLock<WebviewWindow> = OnceLock::new();
    static SHARED: OnceLock<Arc<Shared>> = OnceLock::new();
    /// --frame-log FILE: one CSV line per presented picture with QPC times (last packet received, decoded, Present
    /// called, Present returned), to pair with PresentMon's display times (test/measure_latency.cmd)
    static FRAME_LOG: OnceLock<String> = OnceLock::new();
    /// --shot FILE [--shot-after SECONDS]: run without showing anything (a hidden 1920x1080 window), save the frame
    /// (video and bar) as a BMP after that long (default 6 s), and exit. For testing without taking over the screen.
    static SHOT: OnceLock<(std::path::PathBuf, f64)> = OnceLock::new();
    fn sh() -> &'static Arc<Shared> {
        SHARED.get().expect("shared state")
    }

    struct Sink {
        aus: Vec<(Vec<u8>, bool)>,
        lost: u64,
        resyncs: u64,
        recovered: u64,
    }
    impl StreamSink for Sink {
        fn au(&mut self, au: Vec<u8>, after_gap: bool) {
            self.aus.push((au, after_gap));
        }
        fn source(&mut self, lost: bool) {
            if lost {
                self.lost += 1;
            }
        }
        fn event(&mut self, _text: String) {
            self.resyncs += 1;
        }
        fn recovered(&mut self) {
            self.recovered += 1;
        }
    }

    /// The USB product name of each serial port ("COM8" -> "Horizon HAT receiver"): serialport only gives Windows'
    /// generic name; the board's own is the device property "bus reported device description".
    fn usb_products() -> std::collections::HashMap<String, String> {
        use windows::Win32::Devices::DeviceAndDriverInstallation::*;
        use windows::Win32::Devices::Properties::{DEVPKEY_Device_BusReportedDeviceDesc, DEVPROPTYPE};
        let mut out = std::collections::HashMap::new();
        let wide = |b: &[u8]| {
            let w: Vec<u16> = b
                .chunks_exact(2)
                .map(|c| u16::from_le_bytes([c[0], c[1]]))
                .take_while(|&c| c != 0)
                .collect();
            String::from_utf16_lossy(&w)
        };
        unsafe {
            let Ok(set) = SetupDiGetClassDevsW(
                Some(&GUID_DEVCLASS_PORTS),
                windows::core::PCWSTR::null(),
                None,
                DIGCF_PRESENT,
            ) else {
                return out;
            };
            for i in 0.. {
                let mut dev = SP_DEVINFO_DATA {
                    cbSize: std::mem::size_of::<SP_DEVINFO_DATA>() as u32,
                    ..Default::default()
                };
                if SetupDiEnumDeviceInfo(set, i, &mut dev).is_err() {
                    break;
                }
                let mut buf = [0u8; 512];
                // "USB Serial Device (COM8)" -> COM8
                if SetupDiGetDeviceRegistryPropertyW(set, &dev, SPDRP_FRIENDLYNAME, None, Some(&mut buf), None).is_err()
                {
                    continue;
                }
                let friendly = wide(&buf);
                let Some(port) = friendly
                    .rsplit_once('(')
                    .and_then(|(_, r)| r.strip_suffix(')'))
                    .filter(|p| p.starts_with("COM"))
                else {
                    continue;
                };
                let mut ty = DEVPROPTYPE::default();
                let mut buf = [0u8; 512];
                if SetupDiGetDevicePropertyW(
                    set,
                    &dev,
                    &DEVPKEY_Device_BusReportedDeviceDesc,
                    &mut ty,
                    Some(&mut buf),
                    None,
                    0,
                )
                .is_ok()
                {
                    out.insert(port.to_string(), wide(&buf));
                }
            }
            let _ = SetupDiDestroyDeviceInfoList(set);
        }
        out
    }

    /// Our boards that are not receivers, by USB product name (radio_firmware/CMakeLists.txt HZ_USB_PRODUCT_*): the Pi
    /// transmitter's and the Air unit's radios.
    fn is_transmitter(product: &str) -> bool {
        product.contains("transmitter") || product.contains("Air radio")
    }

    /// The Grounds plugged in: (port, label). Transmitter boards (a Pi HAT or Air radio on USB for flashing) are left out.
    fn grounds() -> Vec<(String, String)> {
        let products = usb_products();
        serialport::available_ports()
            .unwrap_or_default()
            .into_iter()
            .filter_map(|p| match p.port_type {
                serialport::SerialPortType::UsbPort(u) if u.vid == GROUND_VID => {
                    let product = products.get(&p.port_name).cloned();
                    if product.as_deref().is_some_and(is_transmitter) {
                        return None;
                    }
                    let name = product.unwrap_or_else(|| "Ground".into());
                    let label = format!("{} {}", name, u.serial_number.unwrap_or_default());
                    Some((p.port_name, label.trim().to_string()))
                }
                _ => None,
            })
            .collect()
    }

    /// Counts one parsed line into the statistics. `t`: unix time.
    fn count(live: &mut Live, what: &Line, t: f64) {
        match what {
            Line::Packet {
                ok: true,
                filler,
                repaired,
                video,
                radio,
                air,
                ..
            } => {
                if t - live.last_heard >= 1.0 {
                    live.link_since = t;
                }
                live.last_heard = t;
                if *video {
                    live.last_video = t;
                }
                if let Some(a) = air {
                    live.air = Some((t, *a));
                }
                live.stats.packet(t, radio, *filler);
                if *repaired {
                    live.stats.repaired(t);
                    live.repaired += 1;
                }
                if *video {
                    live.stats.block(t, false);
                }
            }
            Line::Packet { ok: false, foreign, .. } => {
                if let Some(l) = foreign {
                    live.foreign = Some((t, *l));
                }
                if t - live.last_heard >= 1.0 {
                    live.link_since = t;
                }
                live.last_heard = t; // heard, too damaged to use: the link is there but weak
                live.stats.missed(t, true)
            }
            Line::Miss { .. } => live.stats.missed(t, false),
            Line::Cfg {
                freq_hz,
                pwr_dbm,
                locked,
                hop,
            } => {
                live.cfg = Some(Cfg {
                    freq_hz: *freq_hz,
                    pwr_dbm: *pwr_dbm,
                    locked: *locked,
                    hop: *hop,
                });
            }
            Line::Who(kv) => {
                live.who = kv.clone();
                live.fw_problem = fw_problem(&live.port, kv);
            }
            Line::Profile(p) => live.profile = Some(p.clone()),
            Line::Ident(text) => {
                live.stats.ident();
                live.ident = Some((t, text.clone()));
            }
            Line::Status(s) => live.status = s.clone(),
            Line::Other => {}
        }
    }

    /// Developer tool: drops received packets in bursts (two-state Gilbert model), before the engine, so FEC,
    /// decoding, statistics and recordings all see it.
    struct Injector {
        in_burst: bool,
        rng: u64,
    }
    impl Injector {
        fn new() -> Self {
            Injector {
                in_burst: false,
                rng: (unix_now() * 1e6) as u64 | 1,
            }
        }
        fn uniform(&mut self) -> f64 {
            self.rng ^= self.rng << 13;
            self.rng ^= self.rng >> 7;
            self.rng ^= self.rng << 17;
            (self.rng >> 11) as f64 / (1u64 << 53) as f64
        }
        /// for one slot: drop it?
        fn drop(&mut self, share: f64, burst: f64) -> bool {
            if share <= 0.0 {
                self.in_burst = false;
                return false;
            }
            let exit = 1.0 / burst.max(1.0);
            let enter = (share * exit / (1.0 - share).max(1e-6)).min(1.0);
            self.in_burst = if self.in_burst {
                self.uniform() >= exit
            } else {
                self.uniform() < enter
            };
            self.in_burst
        }
    }

    /// The Ground firmware versions this app accepts (build.rs: commits with the same firmware sources, newest first)
    /// and the over-the-air format it speaks.
    const GROUND_FW: &str = env!("HZ_GROUND_FW");
    const AIR_FORMAT: &str = "5";
    fn bare(v: &str) -> &str {
        v.trim_end_matches("-dirty")
    }
    /// What is wrong with a Ground's firmware, from its W reply (key=value pairs), if anything.
    fn fw_problem(port: &str, who: &[(String, String)]) -> Option<String> {
        let get = |k: &str| who.iter().find(|(a, _)| a == k).map(|(_, v)| v.as_str());
        if let Some(f) = get("fmt").filter(|f| *f != AIR_FORMAT) {
            return Some(format!("{port}: the Ground speaks over-the-air format {f}, this app format {AIR_FORMAT}: update the Ground (setup site, Ground page)"));
        }
        match get("git") {
            None | Some("unknown") => Some(format!(
                "{port}: the Ground does not report its firmware version: update it (setup site, Ground page)"
            )),
            Some(g) if GROUND_FW != "unknown" && !GROUND_FW.split(',').any(|ok| ok == bare(g)) => {
                let want = GROUND_FW.split(',').next().unwrap_or("");
                Some(format!("{port}: Ground firmware {g} is not up to date (this app: {want}): update it on the setup site, Ground page"))
            }
            _ => None,
        }
    }
    /// Asks a Ground that is not the one receiving who it is (W), and closes it again.
    fn probe(name: &str) -> Result<Vec<(String, String)>, String> {
        let mut p = serialport::new(name, 115200)
            .timeout(Duration::from_millis(50))
            .open()
            .map_err(|e| format!("{name} does not open: {e}"))?;
        let _ = p.write_data_terminal_ready(true);
        p.write_all(b"W\n").map_err(|e| format!("{name}: {e}"))?;
        let (mut buf, mut chunk) = (Vec::new(), [0u8; 4096]);
        let until = std::time::Instant::now() + Duration::from_millis(1500);
        while std::time::Instant::now() < until {
            if let Ok(n) = p.read(&mut chunk) {
                buf.extend_from_slice(&chunk[..n]);
                for line in String::from_utf8_lossy(&buf).lines() {
                    if let Some(rest) = line.strip_prefix("W ") {
                        return Ok(rest
                            .split_whitespace()
                            .filter_map(|kv| kv.split_once('='))
                            .map(|(k, v)| (k.into(), v.into()))
                            .collect());
                    }
                }
            }
        }
        Err(format!("{name}: no answer (is it a Horizon Ground?)"))
    }

    fn serial_thread(sh: Arc<Shared>) {
        let mut injector = Injector::new();
        loop {
            let all = grounds();
            if all.is_empty() {
                sh.live.lock().unwrap().port = String::new();
                sh.live.lock().unwrap().error = "No receiver: plug in the Ground".into();
                std::thread::sleep(Duration::from_secs(1));
                continue;
            }
            // every Ground must open and be up to date; the first that opens is the one received from
            let mut opened = None;
            let mut problems = Vec::new();
            for (name, _) in &all {
                if opened.is_none() {
                    match serialport::new(name, 115200).timeout(Duration::from_millis(20)).open() {
                        Ok(p) => opened = Some((name.clone(), p)),
                        Err(e) => problems.push(format!("{name} does not open: {e} (another program using it?)")),
                    }
                } else {
                    match probe(name) {
                        Ok(who) => problems.extend(fw_problem(name, &who)),
                        Err(e) => problems.push(e),
                    }
                }
            }
            let names: Vec<String> = all.iter().map(|g| g.0.clone()).collect();
            let Some((name, mut port)) = opened else {
                let mut l = sh.live.lock().unwrap();
                l.port = String::new();
                l.error = "No receiver could be opened".into(); // the why is in `problems`
                l.problems = problems;
                drop(l);
                std::thread::sleep(Duration::from_secs(1));
                continue;
            };
            let _ = port.write_data_terminal_ready(true); // the Ground sends only while DTR is set
            {
                let mut l = sh.live.lock().unwrap();
                l.port = name.clone();
                l.error.clear();
                l.who.clear();
                l.profile = None;
                l.problems = problems;
                l.fw_problem = None;
            }
            let _ = port.write_all(b"W\nK\n"); // who it is, and the link it keeps
            let mut stream = Stream::default();
            let mut idents = IdentAssembler::default();
            let mut sink = Sink {
                aus: Vec::new(),
                lost: 0,
                resyncs: 0,
                recovered: 0,
            };
            let mut buf: Vec<u8> = Vec::with_capacity(1 << 16);
            let mut chunk = [0u8; 8192];
            // the first moments after opening: what the USB buffer held from before, with gaps that are not losses
            let opened = std::time::Instant::now();
            let mut checked = std::time::Instant::now();
            loop {
                // a Ground plugged in or out: check them all again
                if checked.elapsed() > Duration::from_secs(2) {
                    checked = std::time::Instant::now();
                    if grounds().iter().map(|g| g.0.clone()).collect::<Vec<_>>() != names {
                        break;
                    }
                }
                for cmd in std::mem::take(&mut *sh.outbox.lock().unwrap()) {
                    let _ = port.write_all(format!("{cmd}\n").as_bytes());
                }
                match port.read(&mut chunk) {
                    Ok(0) => continue,
                    Ok(n) => buf.extend_from_slice(&chunk[..n]),
                    Err(e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
                    Err(e) => {
                        sh.live.lock().unwrap().error = format!("{name} lost: {e}");
                        break;
                    }
                }
                let arrival = qpc();
                let t = unix_now();
                let mut start = 0;
                while let Some(i) = buf[start..].iter().position(|&b| b == b'\n') {
                    let mut text = String::from_utf8_lossy(&buf[start..start + i])
                        .trim_end_matches('\r')
                        .to_string();
                    start += i + 1;
                    if text.starts_with("P ") {
                        let (share, burst) = *sh.inject.lock().unwrap();
                        if injector.drop(share, burst) {
                            // the slot is "missed": an M line with reason I (injected), as the Ground would send it
                            let f: Vec<&str> = text.split(' ').collect();
                            text = format!("M {} {} I 9999", f.get(1).unwrap_or(&"0"), f.get(2).unwrap_or(&"0"));
                        }
                    }
                    if let Some(r) = sh.recorder.lock().unwrap().as_mut() {
                        r.line(t, &text);
                    }
                    let what = engine::line(&text, &mut stream, &mut sink, &mut idents);
                    {
                        let mut l = sh.live.lock().unwrap();
                        count(&mut l, &what, t);
                        if opened.elapsed() < Duration::from_secs(2) {
                            sink.lost = 0;
                        }
                        for _ in 0..std::mem::take(&mut sink.lost) {
                            l.stats.block(t, true);
                            l.lost += 1;
                        }
                        for _ in 0..std::mem::take(&mut sink.resyncs) {
                            l.stats.resync();
                        }
                        for _ in 0..std::mem::take(&mut sink.recovered) {
                            l.stats.block(t, false); // rebuilt by the block FEC: received after all
                            l.fec_recovered += 1;
                        }
                    }
                    let showing_live = !sh.replaying.load(Ordering::Relaxed);
                    for (au, after_gap) in sink.aus.drain(..) {
                        if !showing_live {
                            continue; // a replay has the viewer
                        }
                        if let Some((data, _sent)) = engine::prepare_au(&au) {
                            let epoch = sh.epoch.load(Ordering::Relaxed);
                            let _ = sh.dec_tx.send(DecMsg {
                                au: data,
                                after_gap,
                                arrival,
                                show: true,
                                live: true,
                                epoch,
                            });
                        }
                    }
                }
                buf.drain(..start);
            }
        }
    }

    // Decodes every access unit of the source shown and keeps the newest picture.
    fn decode_thread(sh: Arc<Shared>, rx: Receiver<DecMsg>) {
        let mut dec: Option<Decoder> = None;
        let mut epoch = 0u64;
        let mut px = Vec::new();
        let mut hold = crate::hold::Hold::default();
        let mut gap = false; // a loss whose picture has not been shown yet
        for m in rx {
            if m.epoch < epoch {
                continue; // from a source no longer shown (before a seek or a switch)
            }
            if m.epoch > epoch || dec.is_none() {
                // a new source: the decoder starts over, so nothing of the old one is predicted from
                epoch = m.epoch;
                hold = crate::hold::Hold::default();
                dec = match Decoder::load() {
                    Ok(d) => Some(d),
                    Err(e) => {
                        sh.live.lock().unwrap().error = e;
                        return;
                    }
                };
            }
            let d = dec.as_mut().unwrap();
            let t0 = qpc();
            if !d.send(&m.au) && m.live {
                sh.live.lock().unwrap().stats.decode_error();
            }
            gap |= m.after_gap;
            if let Some((w, h)) = d.picture(&mut px) {
                let g = std::mem::take(&mut gap);
                let held = hold.frame(&mut px, w, h, g) > 0;
                if held && m.live {
                    sh.live.lock().unwrap().stats.held();
                }
                let done = qpc();
                if m.live {
                    sh.live
                        .lock()
                        .unwrap()
                        .stats
                        .frame(unix_now(), m.au.len(), qpc_ms(done - t0));
                }
                if m.show {
                    if m.live {
                        sh.last_picture.store(unix_now().to_bits(), Ordering::Relaxed);
                    }
                    {
                        let mut l = sh.latest.lock().unwrap();
                        std::mem::swap(&mut l.px, &mut px);
                        l.w = w;
                        l.h = h;
                        l.arrival = m.arrival;
                        l.decoded = done;
                        l.seq += 1;
                    }
                    sh.cv.notify_all();
                }
            }
        }
    }

    fn toggle_record() {
        let sh = sh();
        let mut r = sh.recorder.lock().unwrap();
        match r.take() {
            Some(rec) => {
                rec.stop();
            }
            None => match Recorder::start(&session::out_dir()) {
                Ok(rec) => *r = Some(rec),
                Err(e) => sh.live.lock().unwrap().error = format!("cannot record: {e}"),
            },
        }
        sh.bar_dirty.store(true, Ordering::Relaxed);
    }

    /// The Settings panel, docked on the left under the bar. A separate owned, borderless window because a web view is
    /// always composed: closed, it is hidden and the video window is flipped straight to the screen again.
    static PANEL: AtomicBool = AtomicBool::new(false);
    fn panel_open() -> bool {
        PANEL.load(Ordering::Relaxed)
    }
    /// The panel's rectangle (physical pixels) on the monitor the video window is on.
    fn panel_rect() -> Option<RECT> {
        let video = VIDEO.load(Ordering::Relaxed);
        if video == 0 {
            return None;
        }
        let (_, m) = monitor_of(HWND(video as _));
        let (w, h) = (m.right - m.left, m.bottom - m.top);
        let pw = ((w as f32 * 0.42) as i32).max(900.min(w));
        let bar = (h as f32 * BAR_SHARE).round() as i32;
        // on the left, under the bar: where the setup site has its navigation
        Some(RECT {
            left: m.left,
            top: m.top + bar,
            right: m.left + pw,
            bottom: m.bottom,
        })
    }
    fn place_panel() {
        if let (Some(w), Some(r)) = (WINDOW.get(), panel_rect()) {
            let _ = w.set_position(tauri::PhysicalPosition::new(r.left, r.top));
            let _ = w.set_size(tauri::PhysicalSize::new(
                (r.right - r.left) as u32,
                (r.bottom - r.top) as u32,
            ));
        }
    }
    fn set_panel(open: bool) {
        let Some(w) = WINDOW.get() else { return };
        PANEL.store(open, Ordering::Relaxed);
        if open {
            // owned by the video window: always above it (a click on the video must not bury it)
            let video = VIDEO.load(Ordering::Relaxed);
            if let (Ok(h), true) = (w.hwnd(), video != 0) {
                unsafe {
                    SetWindowLongPtrW(HWND(h.0 as _), GWLP_HWNDPARENT, video);
                }
            }
            place_panel();
            set_bar(true);
            let _ = w.show();
            let _ = w.set_focus();
        } else {
            let _ = w.hide();
            let video = VIDEO.load(Ordering::Relaxed);
            if video != 0 {
                unsafe {
                    let _ = PostMessageW(Some(HWND(video as _)), WM_APP_FOCUS, WPARAM(0), LPARAM(0));
                }
            }
        }
        if let Some(sh) = SHARED.get() {
            sh.bar_dirty.store(true, Ordering::Relaxed);
            sh.cv.notify_all();
        }
    }
    fn sh_err(e: String) {
        if let Some(sh) = SHARED.get() {
            sh.live.lock().unwrap().error = e;
        }
    }
    fn show_settings() {
        set_panel(true);
    }
    fn toggle_settings() {
        set_panel(!panel_open());
    }

    fn replay_cmd(c: replay::Cmd) {
        if let Some(r) = sh().replay.lock().unwrap().as_ref() {
            let _ = r.tx.send(c);
        }
        sh().bar_dirty.store(true, Ordering::Relaxed);
    }

    fn back_to_live() {
        let sh = sh();
        if let Some(r) = sh.replay.lock().unwrap().take() {
            let _ = r.tx.send(replay::Cmd::Stop);
        }
        sh.replaying.store(false, Ordering::Relaxed);
        sh.epoch.fetch_add(1, Ordering::Relaxed);
        sh.bar_dirty.store(true, Ordering::Relaxed);
        if let Some(w) = WINDOW.get() {
            let _ = w.emit("replay-closed", ());
        }
    }

    /// Opens a recorded session: its statistics for the Flight page, and (with its packets) its replay in the viewer.
    fn open_flight_dir(dir: &std::path::Path) -> Result<session::Flight, String> {
        let flight = session::load_flight(dir)?;
        back_to_live();
        let rx = dir.join("packets.rxlog");
        if rx.is_file() {
            let sh = sh().clone();
            let (tx, cmds) = channel();
            let state = Arc::new(Mutex::new(replay::State {
                dir: dir.to_string_lossy().into_owned(),
                name: flight.summary.name.clone(),
                start: flight.summary.start,
                end: flight.summary.start + flight.summary.duration,
                ..Default::default()
            }));
            let readout_keys: [(&str, &str); 5] = [
                ("rssi_dbm", "RSSI"),
                ("snr_db", "SNR"),
                ("lq_pct", "link"),
                ("fps", "fps"),
                ("video_kbps", "kbit/s"),
            ];
            let readout = readout_keys
                .iter()
                .filter_map(|(k, l)| flight.series.iter().find(|s| s.0 == *k).map(|s| (*k, *l, s.1.clone())))
                .collect();
            let title = chrono::DateTime::from_timestamp(flight.summary.start as i64, 0)
                .map(|d| {
                    d.with_timezone(&chrono::Local)
                        .format("Flight %-d %b %H:%M")
                        .to_string()
                })
                .unwrap_or_else(|| flight.summary.name.clone());
            *sh.replay.lock().unwrap() = Some(ReplayCtl {
                tx,
                state: state.clone(),
                title,
                time: flight.time.clone(),
                readout,
                marks: flight.problems.iter().map(|p| p.start).collect(),
            });
            sh.replaying.store(true, Ordering::Relaxed);
            let mut stream = Stream::default();
            stream.link = engine::recording_link(&rx).unwrap_or(0); // the flight's own link ID
            let mut idents = IdentAssembler::default();
            let mut collect = Sink {
                aus: Vec::new(),
                lost: 0,
                resyncs: 0,
                recovered: 0,
            };
            let mut my_epoch = 0u64;
            let sh2 = sh.clone();
            let sink: replay::Sink = Box::new(move |text, _t, show, fresh| {
                if !sh2.replaying.load(Ordering::Relaxed) {
                    return;
                }
                if fresh {
                    // a seek (or the start): the stream and the decoder start over
                    my_epoch = sh2.epoch.fetch_add(1, Ordering::Relaxed) + 1;
                    let link = stream.link;
                    stream = Stream::default();
                    stream.link = link;
                }
                engine::line(text, &mut stream, &mut collect, &mut idents);
                for (au, after_gap) in collect.aus.drain(..) {
                    if let Some((data, _)) = engine::prepare_au(&au) {
                        let _ = sh2.dec_tx.send(DecMsg {
                            au: data,
                            after_gap,
                            arrival: qpc(),
                            show,
                            live: false,
                            epoch: my_epoch,
                        });
                    }
                }
            });
            std::thread::spawn(move || replay::run(rx, state, cmds, sink));
            sh.bar_shown.store(true, Ordering::Relaxed);
            sh.bar_dirty.store(true, Ordering::Relaxed);
        }
        Ok(flight)
    }

    // The video window: nothing but the swap chain.
    const WM_APP_FOCUS: u32 = WM_APP + 1;
    static VIDEO: AtomicIsize = AtomicIsize::new(0);
    static MOVED: AtomicBool = AtomicBool::new(true); // true: lay the window out at start
    static BUTTONS: Mutex<Vec<(u32, RECT)>> = Mutex::new(Vec::new());

    /// Brings the video window to the front with the keyboard focus: Windows only lets the foreground thread do that,
    /// so this thread joins its input queue for a moment. Without focus a full-screen window can stay composed.
    fn focus_window(hwnd: HWND) {
        unsafe {
            let prior = GetForegroundWindow();
            let own = GetCurrentThreadId();
            let other = if prior.0.is_null() {
                own
            } else {
                GetWindowThreadProcessId(prior, None)
            };
            let attached = other != own && AttachThreadInput(own, other, true).as_bool();
            let _ = BringWindowToTop(hwnd);
            let _ = SetForegroundWindow(hwnd);
            let _ = SetFocus(Some(hwnd));
            if attached {
                let _ = AttachThreadInput(own, other, false);
            }
        }
    }

    fn bar_shown() -> bool {
        SHARED.get().is_some_and(|s| s.bar_shown.load(Ordering::Relaxed))
    }
    fn set_bar(show: bool) {
        if !show && panel_open() {
            set_panel(false); // the panel hangs under the bar: video only closes both
        }
        if let Some(sh) = SHARED.get() {
            sh.bar_shown.store(show, Ordering::Relaxed);
            sh.bar_dirty.store(true, Ordering::Relaxed);
            sh.cv.notify_all();
        }
    }
    fn replaying() -> bool {
        SHARED.get().is_some_and(|s| s.replaying.load(Ordering::Relaxed))
    }
    fn quit_app() {
        if let Some(rec) = SHARED.get().and_then(|s| s.recorder.lock().unwrap().take()) {
            rec.stop();
        }
        if let Some(w) = WINDOW.get() {
            w.app_handle().exit(0);
        } else {
            std::process::exit(0);
        }
    }
    fn speed_next(cur: f64) -> f64 {
        match cur {
            x if x < 0.75 => 1.0,
            x if x < 1.5 => 2.0,
            x if x < 3.0 => 4.0,
            _ => 0.5,
        }
    }

    fn click(id: u32, frac: f64) {
        match id {
            BTN_QUIT => quit_app(),
            BTN_VIDEO => set_bar(false),
            BTN_REC => toggle_record(),
            BTN_SETTINGS => toggle_settings(),
            BTN_BACK => replay_cmd(replay::Cmd::Step(-10.0)),
            BTN_FWD => replay_cmd(replay::Cmd::Step(10.0)),
            BTN_PLAY => replay_cmd(replay::Cmd::Toggle),
            BTN_SPEED => {
                let cur = sh()
                    .replay
                    .lock()
                    .unwrap()
                    .as_ref()
                    .map(|r| r.state.lock().unwrap().speed)
                    .unwrap_or(1.0);
                replay_cmd(replay::Cmd::Speed(speed_next(cur)));
            }
            BTN_LIVE => back_to_live(),
            BTN_SEEK => {
                let t = sh().replay.lock().unwrap().as_ref().map(|r| {
                    let s = r.state.lock().unwrap();
                    s.start + frac * (s.end - s.start)
                });
                if let Some(t) = t {
                    replay_cmd(replay::Cmd::Seek(t));
                }
            }
            _ => {}
        }
    }

    unsafe extern "system" fn video_proc(hwnd: HWND, msg: u32, wp: WPARAM, lp: LPARAM) -> LRESULT {
        match msg {
            WM_KEYDOWN => {
                let key = wp.0 as u16;
                let rp = replaying();
                if key == b'Q' as u16 {
                    quit_app();
                } else if key == VK_ESCAPE.0 && rp {
                    back_to_live(); // as the bar's "Back to live (Esc)" says, also with the panel open
                } else if key == VK_ESCAPE.0 && panel_open() {
                    set_panel(false);
                } else if key == b'H' as u16 || key == VK_ESCAPE.0 {
                    set_bar(!bar_shown());
                } else if key == b'R' as u16 {
                    toggle_record();
                } else if key == b'S' as u16 {
                    toggle_settings();
                } else if rp && key == VK_SPACE.0 {
                    replay_cmd(replay::Cmd::Toggle);
                } else if rp && key == VK_LEFT.0 {
                    replay_cmd(replay::Cmd::Step(-10.0));
                } else if rp && key == VK_RIGHT.0 {
                    replay_cmd(replay::Cmd::Step(10.0));
                } else if !bar_shown() {
                    set_bar(true); // any other key: the bar back
                }
                LRESULT(0)
            }
            WM_LBUTTONDOWN => {
                if !bar_shown() {
                    set_bar(true);
                } else {
                    let (x, y) = ((lp.0 & 0xffff) as i16 as i32, ((lp.0 >> 16) & 0xffff) as i16 as i32);
                    let hit = BUTTONS
                        .lock()
                        .unwrap()
                        .iter()
                        .find(|(_, r)| x >= r.left && x < r.right && y >= r.top && y < r.bottom)
                        .copied();
                    if let Some((id, r)) = hit {
                        click(id, (x - r.left) as f64 / (r.right - r.left).max(1) as f64);
                    }
                }
                LRESULT(0)
            }
            WM_WINDOWPOSCHANGED => {
                MOVED.store(true, Ordering::Relaxed); // moved (Win+Shift+arrow): fill that monitor, with its GPU
                unsafe { DefWindowProcW(hwnd, msg, wp, lp) }
            }
            WM_APP_FOCUS => {
                focus_window(hwnd);
                LRESULT(0)
            }
            WM_SETCURSOR if (lp.0 & 0xffff) as u32 == HTCLIENT => {
                // the pointer only while the bar (and its buttons) is shown; none over the bare video
                unsafe {
                    SetCursor(if bar_shown() {
                        LoadCursorW(None, IDC_ARROW).ok()
                    } else {
                        None
                    })
                };
                LRESULT(1)
            }
            WM_CLOSE => {
                quit_app();
                LRESULT(0)
            }
            _ => unsafe { DefWindowProcW(hwnd, msg, wp, lp) },
        }
    }

    /// The monitor window `on` is on, and its rectangle.
    fn monitor_of(on: HWND) -> (HMONITOR, RECT) {
        unsafe {
            let m = MonitorFromWindow(on, MONITOR_DEFAULTTONEAREST);
            let mut mi = MONITORINFO {
                cbSize: std::mem::size_of::<MONITORINFO>() as u32,
                ..Default::default()
            };
            let _ = GetMonitorInfoW(m, &mut mi);
            (m, mi.rcMonitor)
        }
    }

    /// A borderless window covering the primary monitor (the user moves it with Win+Shift+arrow).
    fn video_window() -> windows::core::Result<HWND> {
        unsafe {
            let inst = GetModuleHandleW(None)?;
            let class = w!("HorizonReceiverVideo");
            let wc = WNDCLASSW {
                lpfnWndProc: Some(video_proc),
                hInstance: inst.into(),
                lpszClassName: class,
                hbrBackground: HBRUSH(GetStockObject(BLACK_BRUSH).0),
                ..Default::default()
            };
            RegisterClassW(&wc);
            let (cx, cy) = (GetSystemMetrics(SM_CXSCREEN), GetSystemMetrics(SM_CYSCREEN));
            if SHOT.get().is_some() {
                // never shown: no focus taken, nothing on the screen
                return CreateWindowExW(
                    WS_EX_TOOLWINDOW,
                    class,
                    w!("Horizon Receiver (shot)"),
                    WS_POPUP,
                    0,
                    0,
                    1920,
                    1080,
                    None,
                    None,
                    Some(inst.into()),
                    None,
                );
            }
            // the monitor it was last on (if that monitor is still there; else the main one)
            let (mut x, mut y, mut w, mut h) = (0, 0, cx, cy);
            if let Some((px, py)) = prefs().monitor {
                let m = MonitorFromPoint(
                    windows::Win32::Foundation::POINT { x: px, y: py },
                    MONITOR_DEFAULTTOPRIMARY,
                );
                let mut mi = MONITORINFO {
                    cbSize: std::mem::size_of::<MONITORINFO>() as u32,
                    ..Default::default()
                };
                if GetMonitorInfoW(m, &mut mi).as_bool() {
                    let r = mi.rcMonitor;
                    (x, y, w, h) = (r.left, r.top, r.right - r.left, r.bottom - r.top);
                }
            }
            CreateWindowExW(
                WINDOW_EX_STYLE(0),
                class,
                w!("Horizon Receiver"),
                WS_POPUP | WS_VISIBLE,
                x,
                y,
                w,
                h,
                None,
                None,
                Some(inst.into()),
                None,
            )
        }
    }

    // Presents the newest picture at once.
    fn render_thread(sh: Arc<Shared>) {
        let hwnd = match video_window() {
            Ok(h) => h,
            Err(e) => {
                sh.render.lock().unwrap().error = format!("video window: {e}");
                return;
            }
        };
        VIDEO.store(hwnd.0 as isize, Ordering::Relaxed);
        if SHOT.get().is_none() {
            focus_window(hwnd);
        }
        let started = std::time::Instant::now();
        let start = |hwnd: HWND| -> Option<Renderer> {
            match Renderer::new(hwnd) {
                Ok(r) => {
                    let mut ri = sh.render.lock().unwrap();
                    ri.tearing = r.tearing;
                    ri.gpu = r.gpu.clone();
                    Some(r)
                }
                Err(e) => {
                    sh.render.lock().unwrap().error = format!("Direct3D: {e}");
                    None
                }
            }
        };
        let Some(mut r) = start(hwnd) else { return };
        let mut monitor = monitor_of(hwnd).0;
        let mut last_size = (0u32, 0u32);
        let mut frame_log = FRAME_LOG
            .get()
            .and_then(|p| std::fs::File::create(p).ok())
            .map(std::io::BufWriter::new);
        if let Some(f) = frame_log.as_mut() {
            let mut freq = 0;
            unsafe {
                let _ = QueryPerformanceFrequency(&mut freq);
            }
            let _ = writeln!(
                f,
                "# qpc_frequency={freq}\narrival_qpc,decoded_qpc,present_qpc,present_end_qpc,dxgi_mode"
            );
        }
        let mut shown_seq = 0;
        let mut px = Vec::new();
        loop {
            unsafe {
                let mut m = MSG::default();
                while PeekMessageW(&mut m, None, 0, 0, PM_REMOVE).as_bool() {
                    let _ = TranslateMessage(&m);
                    DispatchMessageW(&m);
                }
            }
            // the window fills the monitor it is on; another monitor: drawn by that monitor's GPU
            if MOVED.swap(false, Ordering::Relaxed) && SHOT.get().is_none() {
                let (m, rect) = monitor_of(hwnd);
                let mut cur = RECT::default();
                unsafe {
                    let _ = GetWindowRect(hwnd, &mut cur);
                    if (cur.left, cur.top, cur.right, cur.bottom) != (rect.left, rect.top, rect.right, rect.bottom) {
                        let _ = SetWindowPos(
                            hwnd,
                            None,
                            rect.left,
                            rect.top,
                            rect.right - rect.left,
                            rect.bottom - rect.top,
                            SWP_NOZORDER | SWP_NOACTIVATE,
                        );
                    }
                }
                if m != monitor {
                    monitor = m;
                    let mut p = prefs();
                    p.monitor = Some(((rect.left + rect.right) / 2, (rect.top + rect.bottom) / 2));
                    save_prefs(p);
                    drop(r);
                    let Some(nr) = start(hwnd) else { return };
                    r = nr;
                }
                if panel_open() {
                    place_panel();
                }
                sh.bar_dirty.store(true, Ordering::Relaxed);
            }
            let (w, h, arrival, decoded, new) = {
                let mut l = sh.latest.lock().unwrap();
                if l.seq == shown_seq {
                    l = sh.cv.wait_timeout(l, Duration::from_millis(30)).unwrap().0;
                }
                if l.seq != shown_seq {
                    shown_seq = l.seq;
                    std::mem::swap(&mut l.px, &mut px);
                    (l.w, l.h, l.arrival, l.decoded, true)
                } else {
                    (0, 0, 0, 0, false)
                }
            };
            let bar = if sh.bar_shown.load(Ordering::Relaxed) {
                Some(sh.bar.lock().unwrap().clone())
            } else {
                None
            };
            let p = prefs();
            r.flip = p.flip;
            // the link warning around the video: amber weak, red critical (the Horizon status colours)
            r.outline = match sh.alert.lock().unwrap().0 {
                1 => Some([0.949, 0.722, 0.294]),
                2 => Some([1.0, 0.541, 0.494]),
                _ => None,
            };
            r.panel = if panel_open() {
                panel_rect().map(|p| (p.right - p.left) as f32).unwrap_or(0.0)
            } else {
                0.0
            };
            r.notice = sh.notice.lock().unwrap().clone();
            if let Some((path, after)) = SHOT.get() {
                if started.elapsed().as_secs_f64() >= *after {
                    // --shot: this frame to the file, then exit
                    let size = if new { (w, h) } else { last_size };
                    r.shot = Some(path.clone());
                    let pic = if px.is_empty() {
                        None
                    } else {
                        Some((px.as_slice(), size.0, size.1))
                    };
                    let ok = r.present(pic, bar.as_ref()).is_ok() && path.is_file();
                    println!(
                        "{} shot {}",
                        if ok { "RESULT PASS" } else { "RESULT FAIL" },
                        path.display()
                    );
                    std::process::exit(if ok { 0 } else { 1 });
                }
            }
            if !new {
                // no new picture: redraw only if the bar changed (stats, shown/hidden, moved)
                if sh.bar_dirty.swap(false, Ordering::Relaxed) {
                    r.wait_ready(100);
                    let pic = if px.is_empty() {
                        None
                    } else {
                        Some((px.as_slice(), last_size.0, last_size.1))
                    };
                    let _ = r.present(pic, bar.as_ref());
                    *BUTTONS.lock().unwrap() = r.buttons.clone();
                }
                continue;
            }
            sh.bar_dirty.store(false, Ordering::Relaxed);
            last_size = (w, h);
            r.wait_ready(100);
            let before = qpc();
            match r.present(
                if px.is_empty() {
                    None
                } else {
                    Some((px.as_slice(), w, h))
                },
                bar.as_ref(),
            ) {
                Ok(_) => {
                    let t = qpc();
                    *BUTTONS.lock().unwrap() = r.buttons.clone();
                    if let Some(f) = frame_log.as_mut() {
                        let _ = writeln!(f, "{arrival},{decoded},{before},{t},{}", r.mode());
                        let _ = f.flush();
                    }
                }
                Err(e) => sh.render.lock().unwrap().error = format!("present: {e}"),
            }
            let mut ri = sh.render.lock().unwrap();
            ri.mode = r.mode().to_string();
            ri.screen = format!("{}x{}", r.back().0, r.back().1);
        }
    }

    #[derive(Serialize, Clone)]
    struct Ground {
        port: String,
        error: String,
        who: Vec<(String, String)>,
        cfg: Option<Cfg>,
        profile: Option<String>,
        ident: Option<(f64, String)>,
        status: String,
        /// errors to show: a Ground that does not open, firmware not up to date
        problems: Vec<String>,
    }
    #[derive(Serialize, Clone)]
    struct Recording {
        dir: String,
        started: f64,
        bytes: u64,
    }
    #[derive(Serialize, Clone)]
    struct AppState {
        ground: Ground,
        recording: Option<Recording>,
        replay: Option<replay::State>,
        out_dir: String,
        bar: bool,
        render: RenderInfo,
        sample: Option<Sample>,
    }

    fn app_state(sample: Option<Sample>) -> AppState {
        let sh = sh();
        let ground = {
            let l = sh.live.lock().unwrap();
            Ground {
                port: l.port.clone(),
                error: l.error.clone(),
                who: l.who.clone(),
                cfg: l.cfg.clone(),
                profile: l.profile.clone(),
                ident: l.ident.clone(),
                status: l.status.clone(),
                problems: l.fw_problem.iter().chain(l.problems.iter()).cloned().collect(),
            }
        };
        let recording = sh.recorder.lock().unwrap().as_ref().map(|r| Recording {
            dir: r.dir.to_string_lossy().into_owned(),
            started: r.started,
            bytes: r.bytes,
        });
        let replay = sh
            .replay
            .lock()
            .unwrap()
            .as_ref()
            .map(|r| r.state.lock().unwrap().clone());
        AppState {
            ground,
            recording,
            replay,
            out_dir: session::out_dir().to_string_lossy().into_owned(),
            bar: sh.bar_shown.load(Ordering::Relaxed),
            render: sh.render.lock().unwrap().clone(),
            sample,
        }
    }

    fn live_bar(s: &Sample, st: &AppState) -> Bar {
        let live = s.get("lq_pct").unwrap_or(0.0) > 0.0;
        let f = |k: &str, unit: &str, d: usize| {
            if live {
                s.get(k).map(|v| format!("{v:.d$} {unit}")).unwrap_or("-".into())
            } else {
                "-".into()
            }
        };
        let g = &st.ground;
        let product = "Ground";
        let mut detail = if g.error.is_empty() {
            format!("{product} · {}", g.port)
        } else {
            g.error.clone()
        };
        if !g.problems.is_empty() {
            // errors first, so they are seen
            detail = format!("{}  ·  {detail}", g.problems.join("  ·  "));
        }
        if let Some(c) = &g.cfg {
            detail += &format!(
                "  ·  {:.3} MHz{}  ·  LR2021 {} dBm",
                c.freq_hz as f64 / 1e6,
                if c.locked { "" } else { " (searching)" },
                c.pwr_dbm
            );
        }
        if let Some((t, call)) = &g.ident {
            let at = chrono::DateTime::from_timestamp(*t as i64, 0)
                .map(|d| d.with_timezone(&chrono::Local).format("%H:%M").to_string())
                .unwrap_or_default();
            detail += &format!("  ·  callsign {call}, heard {at}");
        }
        let (rep, lost, fec) = {
            let l = sh().live.lock().unwrap();
            (l.repaired, l.lost, l.fec_recovered)
        };
        detail += &format!("  ·  repaired {rep} · lost {lost}");
        if fec > 0 {
            detail += &format!(" · rebuilt {fec} (block FEC)");
        }
        let (level, why) = sh().alert.lock().unwrap().clone();
        if level > 0 {
            detail = format!("{why}  ·  {detail}");
        }
        let (inj, burst) = *sh().inject.lock().unwrap();
        if inj > 0.0 {
            detail = format!("INJECTED LOSS {:.0} % (bursts of {burst:.0})  ·  {detail}", inj * 100.0);
        }
        Bar {
            stats: vec![
                ("RSSI".into(), f("rssi_dbm", "dBm", 0)),
                ("SNR".into(), f("snr_db", "dB", 1)),
                ("Link".into(), format!("{:.0} %", s.get("lq_pct").unwrap_or(0.0))),
                ("FPS".into(), format!("{:.0}", s.get("fps").unwrap_or(0.0))),
            ],
            detail,
            warn: !g.error.is_empty() || !g.problems.is_empty() || inj > 0.0 || level > 0,
            rec: st
                .recording
                .as_ref()
                .map(|r| format!("REC {}", mmss(unix_now() - r.started))),
            replay: None,
        }
    }

    fn replay_bar(sample: &Sample) -> Option<Bar> {
        let sh = sh();
        let g = sh.replay.lock().unwrap();
        let r = g.as_ref()?;
        let st = r.state.lock().unwrap().clone();
        let dur = (st.end - st.start).max(0.001);
        // the flight's statistics at the cursor
        let i = r.time.partition_point(|t| *t <= st.pos).saturating_sub(1);
        let at: Vec<String> = r
            .readout
            .iter()
            .filter_map(|(k, l, v)| {
                v.get(i).copied().flatten().map(|x| match *k {
                    "rssi_dbm" => format!("{l} {x:.0} dBm"),
                    "snr_db" => format!("{l} {x:.1} dB"),
                    "lq_pct" => format!("{l} {x:.0} %"),
                    _ => format!("{x:.0} {l}"),
                })
            })
            .collect();
        let detail = if !st.error.is_empty() {
            st.error.clone()
        } else {
            format!("at {}:  {}", mmss(st.pos - st.start), at.join("  ·  "))
        };
        Some(Bar {
            stats: Vec::new(),
            detail,
            warn: !st.error.is_empty(),
            rec: None,
            replay: Some(ReplayBar {
                title: r.title.clone(),
                time: format!("{} / {}", mmss(st.pos - st.start), mmss(dur)),
                playing: st.playing,
                speed: if st.speed == 0.5 {
                    "½×".into()
                } else {
                    format!("{}×", st.speed)
                },
                live: format!("LIVE link {:.0} %", sample.get("lq_pct").unwrap_or(0.0)),
                progress: (st.pos - st.start) / dur,
                marks: r.marks.iter().map(|m| (m - st.start) / dur).collect(),
            }),
        })
    }

    /// The link warning from one statistics sample. SNR margin: the SNR above what SF5 needs (-2.5 dB plus ~1 dB
    /// implementation loss), as on the Link budget page. Weak: link quality under 95 % or under 6 dB margin;
    /// critical: under 70 %, under 3 dB, or the signal gone. Only once the link has been up.
    struct Alerter {
        had_link: bool,
        level: u8,
        hold_until: f64,
        /// when the link (last) came up: no alert for diag::LINK_GRACE_S after it, while link quality fills its window
        up_since: f64,
        down: bool,
    }
    impl Alerter {
        fn new() -> Self {
            Alerter {
                had_link: false,
                level: 0,
                hold_until: 0.0,
                up_since: 0.0,
                down: true,
            }
        }
        fn next(&mut self, s: &Sample, port_ok: bool, now: f64) -> (u8, String) {
            let lq = s.get("lq_pct").unwrap_or(0.0);
            if lq > 0.0 {
                self.had_link = true;
                if self.down {
                    self.up_since = now;
                }
            }
            self.down = lq < 1.0;
            let coming_up = port_ok && lq >= 1.0 && now - self.up_since < crate::diag::LINK_GRACE_S;
            let margin = s.get("snr_db").map(|v| v + 1.5);
            let m = margin.map(|v| format!(", SNR margin {v:.1} dB")).unwrap_or_default();
            let (raw, why) = if !self.had_link || coming_up {
                (0, String::new())
            } else if !port_ok {
                (2, "RECEIVER LOST".to_string())
            } else if lq < 1.0 {
                (2, "SIGNAL LOST".to_string())
            } else if lq < 70.0 || margin.is_some_and(|v| v < 3.0) {
                (2, format!("LINK CRITICAL: link {lq:.0} %{m}"))
            } else if lq < 95.0 || margin.is_some_and(|v| v < 6.0) {
                (1, format!("LINK WEAK: link {lq:.0} %{m}"))
            } else {
                (0, String::new())
            };
            // up at once, down only after 2 s (no flicker)
            if raw >= self.level {
                self.level = raw;
                self.hold_until = now + 2.0;
            } else if now >= self.hold_until {
                self.level = raw;
            }
            (
                self.level,
                if self.level == 0 {
                    String::new()
                } else if raw == 0 {
                    "LINK RECOVERING".into()
                } else {
                    why
                },
            )
        }
    }

    /// "Why is there no picture?" (diag.rs), ten times a second while live; a change redraws the video, since no
    /// picture comes to trigger a redraw.
    fn diag_thread(sh: Arc<Shared>) {
        let started = unix_now();
        loop {
            std::thread::sleep(Duration::from_millis(100));
            let notice = if sh.replaying.load(Ordering::Relaxed)
                || SHOT.get().is_some_and(|_| std::env::var_os("HZ_TEST_NOTICE").is_none())
            {
                None
            } else {
                let last = sh.history.lock().unwrap().back().cloned();
                let l = sh.live.lock().unwrap();
                let d = crate::diag::Inputs {
                    now: unix_now(),
                    started,
                    ground: !l.port.is_empty(),
                    ground_error: if l.problems.is_empty() {
                        l.error.clone()
                    } else {
                        format!("{} {}", l.error, l.problems.join("; "))
                    },
                    last_picture: f64::from_bits(sh.last_picture.load(Ordering::Relaxed)),
                    last_heard: l.last_heard,
                    link_since: l.link_since,
                    last_video: l.last_video,
                    air: l.air,
                    foreign: l.foreign,
                    freq_hz: l.cfg.as_ref().map(|c| c.freq_hz),
                    lq_pct: last.as_ref().and_then(|s| s.get("lq_pct")),
                    snr_db: last.as_ref().and_then(|s| s.get("snr_db")),
                };
                drop(l);
                crate::diag::diagnose(&d)
            };
            let mut cur = sh.notice.lock().unwrap();
            if *cur != notice {
                *cur = notice;
                drop(cur);
                sh.bar_dirty.store(true, Ordering::Relaxed);
                sh.cv.notify_all();
            }
        }
    }

    fn stats_thread(sh: Arc<Shared>, win: WebviewWindow) {
        let mut next = unix_now();
        let mut alerter = Alerter::new();
        loop {
            next += 0.5;
            let wait = next - unix_now();
            if wait > 0.0 {
                std::thread::sleep(Duration::from_secs_f64(wait));
            } else {
                next = unix_now();
            }
            let sample = sh.live.lock().unwrap().stats.sample(unix_now());
            {
                let mut h = sh.history.lock().unwrap();
                h.push_back(sample.clone());
                while h.len() > HISTORY {
                    h.pop_front();
                }
            }
            if let Some(r) = sh.recorder.lock().unwrap().as_mut() {
                r.sample(&sample);
            }
            let st = app_state(Some(sample.clone()));
            let alert = alerter.next(&sample, !st.ground.port.is_empty(), unix_now());
            let mut alert = if sh.replaying.load(Ordering::Relaxed) {
                (0, String::new())
            } else {
                alert
            };
            // testing only (with --shot): HZ_TEST_ALERT=1|2 shows that warning level
            if let Some(l) = std::env::var("HZ_TEST_ALERT")
                .ok()
                .and_then(|v| v.parse::<u8>().ok())
                .filter(|l| (1..=2).contains(l))
            {
                alert = (l, format!("TEST: warning level {l}"));
            }
            *sh.alert.lock().unwrap() = alert;
            let bar = replay_bar(&sample).unwrap_or_else(|| live_bar(&sample, &st));
            *sh.bar.lock().unwrap() = bar;
            sh.bar_dirty.store(true, Ordering::Relaxed);
            let _ = win.emit("tick", st);
        }
    }

    #[derive(Serialize)]
    struct Field {
        key: &'static str,
        label: &'static str,
        unit: &'static str,
    }
    #[tauri::command]
    fn fields() -> Vec<Field> {
        STAT_FIELDS
            .iter()
            .map(|(key, label, unit)| Field { key, label, unit })
            .collect()
    }
    #[tauri::command]
    fn state() -> AppState {
        let s = sh().history.lock().unwrap().back().cloned();
        app_state(s)
    }
    #[tauri::command]
    fn history(since: f64) -> Vec<Sample> {
        sh().history
            .lock()
            .unwrap()
            .iter()
            .filter(|s| s.time > since)
            .cloned()
            .collect()
    }
    fn send(cmd: String) {
        sh().outbox.lock().unwrap().push(cmd);
    }
    #[tauri::command]
    fn set_frequency(hz: u64) -> Result<(), String> {
        if !(150_000_000..=960_000_000).contains(&hz) && !(2_400_000_000..=2_483_500_000).contains(&hz) {
            return Err("the frequency must be 150 to 960 MHz, or in the 2.4 GHz band (2400 to 2483.5 MHz)".into());
        }
        send(format!("F {hz}"));
        Ok(())
    }
    #[tauri::command]
    fn set_power(dbm: f64) -> Result<(), String> {
        let half = (dbm * 2.0).round() as i32;
        if !(-19..=44).contains(&half) {
            return Err("the power must be -9.5 to 22 dBm".into());
        }
        send(format!("P {half}"));
        Ok(())
    }
    #[tauri::command]
    fn set_hopping(centre_hz: u64, n: u32, spacing_hz: u64) -> Result<(), String> {
        if n == 0 {
            send(format!("H {centre_hz} 0"));
            return Ok(());
        }
        if !(2..=128).contains(&n) || spacing_hz == 0 || spacing_hz % 1000 != 0 {
            return Err("hopping: 2 to 128 channels, spacing a whole number of kHz".into());
        }
        send(format!("H {centre_hz} {n} {spacing_hz}"));
        Ok(())
    }
    #[tauri::command]
    fn record() {
        toggle_record();
    }
    #[tauri::command]
    fn sessions() -> Vec<session::Summary> {
        session::list(&session::out_dir())
    }
    #[tauri::command]
    async fn open_flight(dir: String) -> Result<session::Flight, String> {
        open_flight_dir(std::path::Path::new(&dir))
    }
    #[tauri::command]
    fn replay_control(action: String, value: Option<f64>) {
        match action.as_str() {
            "toggle" => replay_cmd(replay::Cmd::Toggle),
            "play" => replay_cmd(replay::Cmd::Play),
            "pause" => replay_cmd(replay::Cmd::Pause),
            "seek" => replay_cmd(replay::Cmd::Seek(value.unwrap_or(0.0))),
            "step" => replay_cmd(replay::Cmd::Step(value.unwrap_or(0.0))),
            "speed" => replay_cmd(replay::Cmd::Speed(value.unwrap_or(1.0))),
            "live" => back_to_live(),
            _ => {}
        }
    }
    #[tauri::command]
    fn export_video(dir: String, from: f64, to: f64) -> String {
        let d = std::path::PathBuf::from(&dir);
        let name = |t: f64| {
            chrono::DateTime::from_timestamp(t as i64, 0)
                .map(|x| x.with_timezone(&chrono::Local).format("%H%M%S").to_string())
                .unwrap_or_default()
        };
        let out = d.join(format!("video_{}-{}.mp4", name(from), name(to)));
        let out_s = out.to_string_lossy().into_owned();
        let o2 = out_s.clone();
        std::thread::spawn(move || {
            let win = WINDOW.get().cloned();
            let mut last = 0.0;
            let mut progress = |p: f64| {
                if p - last >= 0.05 || p >= 1.0 {
                    last = p;
                    if let Some(w) = &win {
                        let _ = w.emit("export", serde_json::json!({ "path": o2, "progress": p }));
                    }
                }
            };
            let r = crate::export::export(&d, from, to, &out, &mut progress);
            if let Some(w) = WINDOW.get() {
                let _ = w.emit(
                    "export",
                    match r {
                        Ok(p) => serde_json::json!({ "path": p.to_string_lossy(), "progress": 1.0, "done": true }),
                        Err(e) => serde_json::json!({ "path": o2, "error": e, "done": true }),
                    },
                );
            }
        });
        out_s
    }
    #[tauri::command]
    fn open_folder(path: Option<String>) {
        let p = path.unwrap_or_else(|| {
            let d = session::out_dir();
            let _ = std::fs::create_dir_all(&d);
            d.to_string_lossy().into_owned()
        });
        let target = std::path::Path::new(&p);
        let _ = if target.is_file() {
            std::process::Command::new("explorer")
                .arg(format!("/select,{p}"))
                .spawn()
        } else {
            std::process::Command::new("explorer").arg(&p).spawn()
        };
    }
    #[tauri::command]
    fn toggle_bar() {
        set_bar(!bar_shown());
    }
    /// Developer: injected packet loss, `percent` 0..90, `burst` average length in slots (1 = independent losses).
    #[tauri::command]
    fn set_injection(percent: f64, burst: f64) {
        *sh().inject.lock().unwrap() = ((percent / 100.0).clamp(0.0, 0.9), burst.clamp(1.0, 1000.0));
        sh().bar_dirty.store(true, Ordering::Relaxed);
    }
    #[tauri::command]
    fn get_prefs() -> Prefs {
        prefs()
    }
    #[tauri::command]
    fn set_prefs(mut p: Prefs) {
        p.monitor = prefs().monitor; // the window's own: not the page's (possibly older) copy
        save_prefs(p);
    }
    /// Recordings, Change…: a folder picker; None = cancelled.
    #[tauri::command]
    async fn choose_out_dir(app: tauri::AppHandle) -> Option<String> {
        use tauri_plugin_dialog::DialogExt;
        let dir = app
            .dialog()
            .file()
            .set_directory(session::out_dir())
            .blocking_pick_folder()?;
        let dir = dir.into_path().ok()?.to_string_lossy().into_owned();
        let mut p = prefs();
        p.out_dir = Some(dir.clone());
        save_prefs(p);
        Some(dir)
    }
    /// Recordings, Open other…: any session folder (packets.rxlog and/or stats.csv), e.g. one copied from another PC.
    #[tauri::command]
    async fn pick_flight(app: tauri::AppHandle) -> Option<String> {
        use tauri_plugin_dialog::DialogExt;
        let dir = app
            .dialog()
            .file()
            .set_directory(session::out_dir())
            .blocking_pick_folder()?;
        Some(dir.into_path().ok()?.to_string_lossy().into_owned())
    }
    #[tauri::command]
    fn close_settings() {
        set_panel(false);
    }
    #[tauri::command]
    fn quit() {
        quit_app();
    }

    #[cfg(test)]
    mod alert_tests {
        use super::*;
        fn sample(lq: f64, snr: f64) -> Sample {
            let mut values = vec![None; STAT_FIELDS.len()];
            values[STAT_FIELDS.iter().position(|f| f.0 == "lq_pct").unwrap()] = Some(lq);
            values[STAT_FIELDS.iter().position(|f| f.0 == "snr_db").unwrap()] = Some(snr);
            Sample {
                time: 0.0,
                values,
                totals: vec![0; crate::stats::TOTAL_FIELDS.len()],
            }
        }
        #[test]
        fn levels_and_hold() {
            let mut a = Alerter::new();
            assert_eq!(
                a.next(&sample(0.0, 0.0), true, 0.0).0,
                0,
                "no warning before the link was ever up"
            );
            assert_eq!(
                a.next(&sample(20.0, 2.0), true, 1.0).0,
                0,
                "link coming up: quality still filling"
            );
            assert_eq!(
                a.next(&sample(60.0, 9.0), true, 5.5).0,
                0,
                "within 5 s of the link coming up"
            );
            assert_eq!(a.next(&sample(100.0, 9.0), true, 11.0).0, 0, "good link");
            assert_eq!(a.next(&sample(90.0, 9.0), true, 12.0).0, 1, "under 95 %: weak");
            assert_eq!(a.next(&sample(100.0, 3.0), true, 13.0).0, 1, "4.5 dB margin: weak");
            assert_eq!(a.next(&sample(100.0, 0.0), true, 14.0).0, 2, "1.5 dB margin: critical");
            assert_eq!(
                a.next(&sample(100.0, 9.0), true, 15.0).0,
                2,
                "held for 2 s after it clears"
            );
            assert_eq!(a.next(&sample(100.0, 9.0), true, 16.1).0, 0, "then clear");
            assert_eq!(a.next(&sample(0.0, 0.0), true, 17.0).1, "SIGNAL LOST");
            assert_eq!(
                a.next(&sample(50.0, 9.0), true, 18.0).1,
                "LINK RECOVERING",
                "back after a loss: the lost alert winds down"
            );
            assert_eq!(
                a.next(&sample(50.0, 9.0), true, 20.5).0,
                0,
                "then the 5 s grace of the new link, not LINK CRITICAL"
            );
            assert_eq!(a.next(&sample(100.0, 9.0), false, 18.5).1, "RECEIVER LOST");
        }
    }

    pub fn run() {
        // Per-monitor DPI aware: a DPI-unaware window is scaled by Windows, so always composed and blurred.
        unsafe {
            let _ = windows::Win32::UI::HiDpi::SetProcessDpiAwarenessContext(
                windows::Win32::UI::HiDpi::DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2,
            );
        }
        let args: Vec<String> = std::env::args().collect();
        if let Some(i) = args.iter().position(|a| a == "--shot") {
            let after = args
                .iter()
                .position(|a| a == "--shot-after")
                .and_then(|j| args.get(j + 1))
                .and_then(|v| v.parse().ok())
                .unwrap_or(6.0);
            if let Some(p) = args.get(i + 1) {
                let _ = SHOT.set((std::path::PathBuf::from(p), after));
            }
        }
        // one receiver at a time (it owns the Ground's port): a second start brings the running one to the front.
        // (A --shot run is a test beside it: it does not take over.)
        if SHOT.get().is_none() {
            single_instance_or_exit();
        }
        fn single_instance_or_exit() {
            unsafe {
                let _guard = windows::Win32::System::Threading::CreateMutexW(None, true, w!("Local\\HorizonReceiver"));
                if windows::Win32::Foundation::GetLastError() == windows::Win32::Foundation::ERROR_ALREADY_EXISTS {
                    if let Ok(h) = FindWindowW(w!("HorizonReceiverVideo"), None) {
                        // this (just started) process may set the foreground; the running one may not, unless allowed
                        let mut pid = 0u32;
                        GetWindowThreadProcessId(h, Some(&mut pid));
                        let _ = AllowSetForegroundWindow(pid);
                        let _ = SetForegroundWindow(h);
                        let _ = PostMessageW(Some(h), WM_APP_FOCUS, WPARAM(0), LPARAM(0));
                    }
                    std::process::exit(0);
                }
                std::mem::forget(_guard);
            }
        }
        if let Some(i) = args.iter().position(|a| a == "--frame-log") {
            if let Some(p) = args.get(i + 1) {
                let _ = FRAME_LOG.set(p.clone());
            }
        }
        *session::OUT_DIR.lock().unwrap() = prefs().out_dir.map(Into::into);
        // --open DIR: open a recorded session at start (Settings on its Flight page, the replay in the viewer)
        let open_at_start = args
            .iter()
            .position(|a| a == "--open")
            .and_then(|i| args.get(i + 1))
            .cloned();
        tauri::Builder::default()
            .plugin(tauri_plugin_dialog::init())
            .invoke_handler(tauri::generate_handler![
                fields,
                state,
                history,
                set_frequency,
                set_power,
                set_hopping,
                record,
                sessions,
                open_flight,
                replay_control,
                export_video,
                open_folder,
                toggle_bar,
                close_settings,
                quit,
                get_prefs,
                set_prefs,
                set_injection,
                choose_out_dir,
                pick_flight
            ])
            .setup(move |app| {
                let win = app.get_webview_window("main").expect("settings window");
                // closing the Settings window hides it; the app ends with Q or the bar's Quit
                let w2 = win.clone();
                win.on_window_event(move |e| {
                    if let tauri::WindowEvent::CloseRequested { api, .. } = e {
                        api.prevent_close();
                        let _ = &w2;
                        set_panel(false);
                    }
                });
                let (dec_tx, dec_rx) = sync_channel(1024);
                let sh = Arc::new(Shared {
                    live: Mutex::new(Live::default()),
                    history: Mutex::new(VecDeque::new()),
                    latest: Mutex::new(Latest {
                        seq: 0,
                        w: 0,
                        h: 0,
                        px: Vec::new(),
                        arrival: 0,
                        decoded: 0,
                    }),
                    cv: Condvar::new(),
                    bar_shown: AtomicBool::new(!std::env::args().any(|a| a == "--hide-overlay" || a == "--no-bar")),
                    bar: Mutex::new(Bar::default()),
                    bar_dirty: AtomicBool::new(true),
                    epoch: AtomicU64::new(1),
                    replaying: AtomicBool::new(false),
                    replay: Mutex::new(None),
                    recorder: Mutex::new(None),
                    outbox: Mutex::new(Vec::new()),
                    render: Mutex::new(RenderInfo::default()),
                    dec_tx,
                    inject: Mutex::new((0.0, 1.0)),
                    alert: Mutex::new((0, String::new())),
                    last_picture: AtomicU64::new(0),
                    notice: Mutex::new(None),
                });
                let _ = SHARED.set(sh.clone());
                let _ = WINDOW.set(win.clone());
                let s = sh.clone();
                std::thread::spawn(move || serial_thread(s));
                let s = sh.clone();
                std::thread::spawn(move || decode_thread(s, dec_rx));
                let s = sh.clone();
                std::thread::spawn(move || render_thread(s));
                let s = sh.clone();
                std::thread::spawn(move || diag_thread(s));
                let s = sh.clone();
                let w3 = win.clone();
                std::thread::spawn(move || stats_thread(s, win));
                if let Some(dir) = open_at_start.clone() {
                    std::thread::spawn(move || {
                        std::thread::sleep(Duration::from_millis(1500)); // the video window and the page first
                        match open_flight_dir(std::path::Path::new(&dir)) {
                            Ok(f) => {
                                if SHOT.get().is_none() {
                                    show_settings(); // (a --shot run shows nothing)
                                }
                                let _ = w3.emit("flight-opened", serde_json::json!({ "dir": dir, "flight": f }));
                            }
                            Err(e) => sh_err(e),
                        }
                    });
                }
                Ok(())
            })
            .run(tauri::generate_context!())
            .expect("Horizon Receiver");
    }
}
