//! Sessions: one folder per recording, holding everything about a flight.
//!   packets.rxlog  every line from the Ground, "<unix time> <line>" (the video is in these packets: replay and export
//!                  decode it again, with the same decoder as live)
//!   stats.csv      the link statistics twice a second (stats.rs: the Python receiver's columns)
//! The Python receiver's log folders (log_*) are listed and opened too; without packets.rxlog they have graphs but no
//! video.
use crate::stats::{Sample, STAT_FIELDS};
use serde::Serialize;
use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};

pub fn unix_now() -> f64 {
    std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0)
}

/// The folder chosen in the Settings (Recordings, Change…), if any.
pub static OUT_DIR: std::sync::Mutex<Option<PathBuf>> = std::sync::Mutex::new(None);

/// Where sessions go: the chosen folder, else ~/HorizonReceiver, or ~/LoRaVideoRX when only that exists (the earlier Python
/// receiver's folder).
pub fn out_dir() -> PathBuf {
    if let Some(d) = OUT_DIR.lock().unwrap().clone() {
        return d;
    }
    let home = std::env::var_os("USERPROFILE").or_else(|| std::env::var_os("HOME")).map(PathBuf::from).unwrap_or_default();
    let new = home.join("HorizonReceiver");
    let old = home.join("LoRaVideoRX");
    if !new.is_dir() && old.is_dir() { old } else { new }
}

// ---- recording
pub struct Recorder {
    pub dir: PathBuf,
    pub started: f64,
    raw: BufWriter<File>,
    stats: BufWriter<File>,
    last_flush: f64,
    pub bytes: u64,
}

impl Recorder {
    pub fn start(base: &Path) -> std::io::Result<Recorder> {
        let name = chrono::Local::now().format("flight_%Y%m%d_%H%M%S").to_string();
        let dir = base.join(name);
        std::fs::create_dir_all(&dir)?;
        let raw = BufWriter::new(File::create(dir.join("packets.rxlog"))?);
        let mut stats = BufWriter::new(File::create(dir.join("stats.csv"))?);
        stats.write_all(Sample::csv_header().as_bytes())?;
        Ok(Recorder { dir, started: unix_now(), raw, stats, last_flush: 0.0, bytes: 0 })
    }
    pub fn line(&mut self, t: f64, line: &str) {
        let s = format!("{t:.4} {line}\n");
        self.bytes += s.len() as u64;
        let _ = self.raw.write_all(s.as_bytes());
        if t - self.last_flush > 1.0 {
            let _ = self.raw.flush();
            let _ = self.stats.flush();
            self.last_flush = t;
        }
    }
    pub fn sample(&mut self, s: &Sample) {
        let _ = self.stats.write_all(s.csv_row().as_bytes());
    }
    pub fn stop(mut self) -> PathBuf {
        let _ = self.raw.flush();
        let _ = self.stats.flush();
        self.dir
    }
}

// ---- reading a session's statistics
/// stats.csv: the column names and the rows (NaN where empty), sorted by time.
pub struct Table {
    pub cols: Vec<String>,
    pub rows: Vec<Vec<f64>>,
}
impl Table {
    pub fn load(path: &Path) -> Option<Table> {
        let f = BufReader::new(File::open(path).ok()?);
        let mut lines = f.lines();
        let cols: Vec<String> = lines.next()?.ok()?.trim().split(',').map(|s| s.to_string()).collect();
        let mut rows: Vec<Vec<f64>> = lines
            .filter_map(|l| l.ok())
            .map(|l| l.trim().split(',').map(|v| v.parse::<f64>().unwrap_or(f64::NAN)).collect::<Vec<f64>>())
            .filter(|r| r.len() == cols.len() && r[0].is_finite())
            .collect();
        rows.sort_by(|a, b| a[0].total_cmp(&b[0]));
        Some(Table { cols, rows })
    }
    pub fn col(&self, name: &str) -> Option<Vec<f64>> {
        let i = self.cols.iter().position(|c| c == name)?;
        Some(self.rows.iter().map(|r| r[i]).collect())
    }
}

/// Per-row increase of a running total that may restart (receiver reconnected). The first row is the baseline (a
/// recording can start long after the receiver did), then only rises count.
fn increases(v: &[f64]) -> Vec<f64> {
    let mut out = Vec::with_capacity(v.len());
    let mut prev = v.iter().copied().find(|x| x.is_finite()).unwrap_or(0.0);
    for &x in v {
        let x = if x.is_finite() { x } else { 0.0 };
        let d = x - prev;
        out.push(if d < 0.0 { x } else { d });
        prev = x;
    }
    out
}

#[derive(Clone, Serialize)]
pub struct Problem {
    pub start: f64,
    pub end: f64,
    pub text: String,
}

const EVENT_COUNTERS: &[(&str, &str)] = &[
    ("blocks_lost", "video packets lost"),
    ("decode_errors", "decoder errors"),
    ("resyncs", "stream resyncs"),
    ("concealed", "pictures concealed"),
];
const MERGE_S: f64 = 2.0; // problems closer than this are one
const GAP_S: f64 = 3.0; // no statistics for this long: the receiver was not running

/// Problems in a session (as the Python log viewer finds them): rows where a trouble counter rose, merged when close,
/// with the lowest link quality in each; and gaps in the statistics.
pub fn problems(t: &Table) -> Vec<Problem> {
    let Some(time) = t.col("time") else { return Vec::new() };
    let lq = t.col("lq_pct");
    let mut raw: Vec<(f64, &str, u64)> = Vec::new();
    for (key, what) in EVENT_COUNTERS {
        if let Some(v) = t.col(key) {
            for (i, d) in increases(&v).iter().enumerate() {
                if *d > 0.0 {
                    raw.push((time[i], what, *d as u64));
                }
            }
        }
    }
    raw.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut merged: Vec<(f64, f64, Vec<(String, u64)>)> = Vec::new();
    for (ti, what, n) in raw {
        if let Some(last) = merged.last_mut().filter(|m| ti - m.1 <= MERGE_S) {
            last.1 = ti;
            match last.2.iter_mut().find(|(w, _)| w == what) {
                Some(e) => e.1 += n,
                None => last.2.push((what.to_string(), n)),
            }
        } else {
            merged.push((ti, ti, vec![(what.to_string(), n)]));
        }
    }
    let mut out: Vec<Problem> = merged
        .into_iter()
        .map(|(a, b, c)| {
            let mut text = c.iter().map(|(w, n)| format!("{n} {w}")).collect::<Vec<_>>().join(", ");
            if let Some(lq) = &lq {
                let low = time
                    .iter()
                    .zip(lq)
                    .filter(|(tt, v)| **tt >= a - 1.0 && **tt <= b + 1.0 && v.is_finite())
                    .map(|(_, v)| *v)
                    .fold(f64::INFINITY, f64::min);
                if low.is_finite() && low < 99.5 {
                    text = format!("link down to {low:.0} %, {text}");
                }
            }
            Problem { start: a, end: b, text }
        })
        .collect();
    for w in time.windows(2) {
        if w[1] - w[0] > GAP_S {
            out.push(Problem { start: w[0], end: w[1], text: format!("no statistics for {:.0} s (receiver not running?)", w[1] - w[0]) });
        }
    }
    out.sort_by(|a, b| a.start.total_cmp(&b.start));
    out
}

#[derive(Clone, Serialize)]
pub struct Summary {
    pub dir: String,
    pub name: String,
    pub start: f64,
    pub duration: f64,
    pub link_pct: Option<f64>,
    pub problems: usize,
    pub bytes: u64,
    pub replayable: bool,
    pub rssi_median: Option<f64>,
    pub rssi_min: Option<f64>,
    pub frames: u64,
    pub lost: u64,
}

fn dir_bytes(d: &Path) -> u64 {
    std::fs::read_dir(d).map(|r| r.filter_map(|e| e.ok()).filter_map(|e| e.metadata().ok()).filter(|m| m.is_file()).map(|m| m.len()).sum()).unwrap_or(0)
}

/// A session's summary from its stats.csv (and its files).
pub fn summarize(dir: &Path, t: Option<&Table>) -> Summary {
    let name = dir.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
    let mut s = Summary {
        dir: dir.to_string_lossy().into_owned(),
        name,
        start: 0.0,
        duration: 0.0,
        link_pct: None,
        problems: 0,
        bytes: dir_bytes(dir),
        replayable: dir.join("packets.rxlog").is_file(),
        rssi_median: None,
        rssi_min: None,
        frames: 0,
        lost: 0,
    };
    let Some(t) = t else { return s };
    let Some(time) = t.col("time") else { return s };
    if let (Some(a), Some(b)) = (time.first(), time.last()) {
        s.start = *a;
        s.duration = b - a;
    }
    let tot = |k: &str| t.col(k).map(|v| increases(&v).iter().sum::<f64>() as u64).unwrap_or(0);
    let (pk, miss) = (tot("pkts"), tot("misses"));
    if pk + miss > 0 {
        s.link_pct = Some(100.0 * pk as f64 / (pk + miss) as f64);
    }
    s.frames = tot("frames");
    s.lost = tot("blocks_lost");
    s.problems = problems(t).len();
    if let Some(mut r) = t.col("rssi_dbm").map(|v| v.into_iter().filter(|x| x.is_finite()).collect::<Vec<_>>()) {
        if !r.is_empty() {
            r.sort_by(|a, b| a.total_cmp(b));
            s.rssi_median = Some(r[r.len() / 2]);
            s.rssi_min = Some(r[0]);
        }
    }
    s
}

/// The sessions in `base`, newest first: this app's flight_* and the Python receiver's log_* folders.
pub fn list(base: &Path) -> Vec<Summary> {
    let mut out: Vec<Summary> = std::fs::read_dir(base)
        .map(|r| {
            r.filter_map(|e| e.ok())
                .map(|e| e.path())
                .filter(|p| p.is_dir())
                .filter(|p| p.file_name().is_some_and(|n| {
                    let n = n.to_string_lossy();
                    n.starts_with("flight_") || n.starts_with("log_")
                }))
                .filter(|p| p.join("stats.csv").is_file() || p.join("packets.rxlog").is_file())
                .map(|p| {
                    let t = Table::load(&p.join("stats.csv"));
                    summarize(&p, t.as_ref())
                })
                .collect()
        })
        .unwrap_or_default();
    out.sort_by(|a, b| b.start.total_cmp(&a.start).then(b.name.cmp(&a.name)));
    out
}

/// An opened flight for the Flight page: its summary, problems, and every graphable column.
#[derive(Serialize)]
pub struct Flight {
    pub summary: Summary,
    pub problems: Vec<Problem>,
    pub time: Vec<f64>,
    /// key -> values (null where empty), only the STAT_FIELDS columns the file has
    pub series: Vec<(String, Vec<Option<f64>>)>,
}

pub fn load_flight(dir: &Path) -> Result<Flight, String> {
    let t = Table::load(&dir.join("stats.csv")).ok_or("no stats.csv in this session")?;
    let time = t.col("time").unwrap_or_default();
    let series = STAT_FIELDS
        .iter()
        .filter_map(|(k, _, _)| t.col(k).map(|v| (k.to_string(), v.into_iter().map(|x| if x.is_finite() { Some(x) } else { None }).collect())))
        .collect();
    Ok(Flight { summary: summarize(dir, Some(&t)), problems: problems(&t), time, series })
}

// ---- packets.rxlog
/// Capture time of one packets.rxlog line ("<unix time> <line>").
pub fn line_time(l: &str) -> Option<(f64, &str)> {
    let (t, rest) = l.split_once(' ')?;
    Some((t.parse().ok()?, rest))
}

/// Byte offsets of packets.rxlog lines about every half second, for seeking: (time, offset).
pub fn index_rxlog(path: &Path) -> std::io::Result<Vec<(f64, u64)>> {
    let mut f = BufReader::with_capacity(1 << 20, File::open(path)?);
    let mut idx = Vec::new();
    let mut off = 0u64;
    let mut line = String::new();
    let mut last = f64::NEG_INFINITY;
    loop {
        line.clear();
        let n = f.read_line(&mut line)?;
        if n == 0 {
            break;
        }
        if let Some((t, _)) = line_time(&line) {
            if t - last >= 0.5 {
                idx.push((t, off));
                last = t;
            }
        }
        off += n as u64;
    }
    Ok(idx)
}

/// Opens packets.rxlog at the last indexed line at or before `t`.
pub fn open_rxlog_at(path: &Path, idx: &[(f64, u64)], t: f64) -> std::io::Result<BufReader<File>> {
    let i = idx.partition_point(|e| e.0 <= t).saturating_sub(1);
    let mut f = File::open(path)?;
    f.seek(SeekFrom::Start(idx.get(i).map(|e| e.1).unwrap_or(0)))?;
    Ok(BufReader::with_capacity(1 << 20, f))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn problems_merge_and_gaps() {
        let mut t = Table { cols: vec!["time".into(), "lq_pct".into(), "blocks_lost".into()], rows: Vec::new() };
        for i in 0..20 {
            let lost = if i < 5 { 0.0 } else if i < 7 { 3.0 } else { 4.0 };
            let lq = if i == 5 { 60.0 } else { 100.0 };
            t.rows.push(vec![100.0 + i as f64 * 0.5, lq, lost]);
        }
        t.rows.push(vec![120.0, 100.0, 4.0]); // a 10.5 s gap
        let p = problems(&t);
        assert_eq!(p.len(), 2, "{:?}", p.iter().map(|x| &x.text).collect::<Vec<_>>());
        assert!(p[0].text.contains("4 video packets lost") && p[0].text.contains("60 %"));
        assert!(p[1].text.starts_with("no statistics"));
    }
}
