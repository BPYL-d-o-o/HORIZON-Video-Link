//! Link statistics over a sliding 1 s window, sampled twice a second.
use serde::Serialize;
use std::collections::VecDeque;

/// (key, label, unit): every graphable statistic, in stats.csv column order.
pub const STAT_FIELDS: &[(&str, &str, &str)] = &[
    ("rssi_dbm", "RSSI (packet)", "dBm"),
    ("rssi_sig_dbm", "RSSI (signal)", "dBm"),
    ("snr_db", "SNR", "dB"),
    ("noise_dbm", "Channel RSSI before packet", "dBm"),
    ("lq_pct", "Link quality", "%"),
    ("pps", "Packets received", "/s"),
    ("data_pps", "Video packets", "/s"),
    ("filler_pps", "Idle (filler) packets", "/s"),
    ("miss_ps", "Missed slots", "/s"),
    ("crc_ps", "CRC errors", "/s"),
    ("blocks_ok_ps", "Video packets received", "/s"),
    ("blocks_lost_ps", "Video packets lost", "/s"),
    ("block_loss_pct", "Video packet loss (1 s)", "%"),
    ("repaired_ps", "Damaged packets repaired", "/s"),
    ("fps", "Decoded frame rate", "fps"),
    ("video_kbps", "Video bitrate", "kbit/s"),
    ("latency_ms", "Link latency (needs --stamp)", "ms"),
    ("decode_ms", "Decode time", "ms"),
    ("display_ms", "Decode -> on screen", "ms"),
    ("foff_hz", "Frequency offset", "Hz"),
    ("timing_us", "Packet timing error", "us"),
    ("div_saved_ps", "Diversity: packets only another receiver got", "/s"),
];
/// Running totals, after the statistics in stats.csv.
pub const TOTAL_FIELDS: &[&str] = &[
    "pkts",
    "filler",
    "misses",
    "blocks_ok",
    "blocks_lost",
    "frames",
    "decode_errors",
    "resyncs",
    "out_dropped",
    "aus_dropped",
    "repaired",
    "crc32_bad",
    "idents",
    "held",
    "concealed",
];

/// One sample: the time (unix seconds) and a value per STAT_FIELDS entry (None: nothing to average), plus totals.
#[derive(Clone, Serialize)]
pub struct Sample {
    pub time: f64,
    pub values: Vec<Option<f64>>,
    pub totals: Vec<u64>,
}

impl Sample {
    pub fn get(&self, key: &str) -> Option<f64> {
        STAT_FIELDS.iter().position(|f| f.0 == key).and_then(|i| self.values[i])
    }
    pub fn total(&self, key: &str) -> u64 {
        TOTAL_FIELDS
            .iter()
            .position(|f| *f == key)
            .map(|i| self.totals[i])
            .unwrap_or(0)
    }
    pub fn csv_header() -> String {
        let mut h = vec!["time".to_string()];
        h.extend(STAT_FIELDS.iter().map(|f| f.0.to_string()));
        h.extend(TOTAL_FIELDS.iter().map(|f| f.to_string()));
        h.join(",") + "\n"
    }
    pub fn csv_row(&self) -> String {
        let mut r = vec![format!("{:.3}", self.time)];
        r.extend(self.values.iter().map(|v| v.map(|x| format_num(x)).unwrap_or_default()));
        r.extend(self.totals.iter().map(|v| v.to_string()));
        r.join(",") + "\n"
    }
}

fn format_num(x: f64) -> String {
    if x.fract() == 0.0 && x.abs() < 1e12 {
        format!("{}", x as i64)
    } else {
        format!("{:.2}", x)
    }
}

#[derive(Clone, Copy)]
struct Pk {
    t: f64,
    rssi: f32,
    rsig: f32,
    snr: f32,
    foff: f32,
    dt: f32,
    noise: f32,
    filler: bool,
}

/// Collects events as they happen; sample() gives the window's statistics.
pub struct Stats {
    pk: VecDeque<Pk>,
    miss: VecDeque<(f64, bool)>, // (t, crc failure)
    blk: VecDeque<(f64, bool)>,  // (t, lost)
    rep: VecDeque<f64>,
    frm: VecDeque<(f64, usize, f64)>, // (t, bytes, decode ms)
    tot: Vec<u64>,
}

const WINDOW: f64 = 1.0;

fn mean(it: impl Iterator<Item = f64>) -> Option<f64> {
    let (mut s, mut n) = (0.0, 0usize);
    for v in it {
        s += v;
        n += 1;
    }
    if n > 0 {
        Some((s / n as f64 * 100.0).round() / 100.0)
    } else {
        None
    }
}

impl Default for Stats {
    fn default() -> Self {
        Stats {
            pk: VecDeque::new(),
            miss: VecDeque::new(),
            blk: VecDeque::new(),
            rep: VecDeque::new(),
            frm: VecDeque::new(),
            tot: vec![0; TOTAL_FIELDS.len()],
        }
    }
}

impl Stats {
    fn inc(&mut self, key: &str, n: u64) {
        if let Some(i) = TOTAL_FIELDS.iter().position(|f| *f == key) {
            self.tot[i] += n;
        }
    }
    pub fn packet(&mut self, t: f64, r: &crate::engine::Radio, filler: bool) {
        self.inc("pkts", 1);
        if filler {
            self.inc("filler", 1);
        }
        self.pk.push_back(Pk {
            t,
            rssi: r.rssi,
            rsig: r.rssi_sig,
            snr: r.snr,
            foff: r.foff_hz,
            dt: r.timing_us,
            noise: r.noise,
            filler,
        });
    }
    pub fn missed(&mut self, t: f64, crc: bool) {
        self.inc("misses", 1);
        self.miss.push_back((t, crc));
    }
    pub fn block(&mut self, t: f64, lost: bool) {
        self.inc(if lost { "blocks_lost" } else { "blocks_ok" }, 1);
        self.blk.push_back((t, lost));
    }
    pub fn repaired(&mut self, t: f64) {
        self.inc("repaired", 1);
        self.rep.push_back(t);
    }
    pub fn frame(&mut self, t: f64, bytes: usize, decode_ms: f64) {
        self.inc("frames", 1);
        self.frm.push_back((t, bytes, decode_ms));
    }
    pub fn decode_error(&mut self) {
        self.inc("decode_errors", 1);
    }
    pub fn resync(&mut self) {
        self.inc("resyncs", 1);
    }
    /// a picture shown with lost areas held from the previous one
    pub fn held(&mut self) {
        self.inc("held", 1);
    }
    pub fn ident(&mut self) {
        self.inc("idents", 1);
    }
    pub fn sample(&mut self, now: f64) -> Sample {
        let lim = now - WINDOW;
        while self.pk.front().is_some_and(|p| p.t < lim) {
            self.pk.pop_front();
        }
        while self.miss.front().is_some_and(|m| m.0 < lim) {
            self.miss.pop_front();
        }
        while self.blk.front().is_some_and(|b| b.0 < lim) {
            self.blk.pop_front();
        }
        while self.rep.front().is_some_and(|t| *t < lim) {
            self.rep.pop_front();
        }
        while self.frm.front().is_some_and(|f| f.0 < lim) {
            self.frm.pop_front();
        }
        let n_slots = self.pk.len() + self.miss.len();
        let lost = self.blk.iter().filter(|b| b.1).count();
        let nb = self.blk.len();
        let mut v = vec![None; STAT_FIELDS.len()];
        let mut set = |k: &str, x: Option<f64>| {
            if let Some(i) = STAT_FIELDS.iter().position(|f| f.0 == k) {
                v[i] = x;
            }
        };
        let pk = &self.pk;
        set("rssi_dbm", mean(pk.iter().map(|p| p.rssi as f64)));
        set("rssi_sig_dbm", mean(pk.iter().map(|p| p.rsig as f64)));
        set("snr_db", mean(pk.iter().map(|p| p.snr as f64)));
        set(
            "noise_dbm",
            mean(pk.iter().filter(|p| p.noise.abs() < 4000.0).map(|p| p.noise as f64)),
        );
        set(
            "lq_pct",
            Some(if n_slots > 0 {
                (1000.0 * pk.len() as f64 / n_slots as f64).round() / 10.0
            } else {
                0.0
            }),
        );
        set("pps", Some(pk.len() as f64));
        set("data_pps", Some(pk.iter().filter(|p| !p.filler).count() as f64));
        set("filler_pps", Some(pk.iter().filter(|p| p.filler).count() as f64));
        set("miss_ps", Some(self.miss.len() as f64));
        set("crc_ps", Some(self.miss.iter().filter(|m| m.1).count() as f64));
        set("blocks_ok_ps", Some((nb - lost) as f64));
        set("blocks_lost_ps", Some(lost as f64));
        set(
            "block_loss_pct",
            Some(if nb > 0 {
                (1000.0 * lost as f64 / nb as f64).round() / 10.0
            } else {
                0.0
            }),
        );
        set("repaired_ps", Some(self.rep.len() as f64));
        set("fps", Some(self.frm.len() as f64));
        set(
            "video_kbps",
            Some((self.frm.iter().map(|f| f.1).sum::<usize>() as f64 * 8.0 / 100.0).round() / 10.0),
        );
        set("decode_ms", mean(self.frm.iter().map(|f| f.2)));
        set("foff_hz", mean(pk.iter().map(|p| p.foff as f64)));
        set("timing_us", mean(pk.iter().map(|p| p.dt as f64)));
        Sample {
            time: (now * 1000.0).round() / 1000.0,
            values: v,
            totals: self.tot.clone(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn header_matches_python_columns() {
        let h = Sample::csv_header();
        assert!(h.starts_with("time,rssi_dbm,rssi_sig_dbm,snr_db,noise_dbm,lq_pct,pps,"));
        assert!(h.trim_end().ends_with("idents,held,concealed"));
    }
    #[test]
    fn link_quality_counts_misses() {
        let mut s = Stats::default();
        let r = crate::engine::Radio {
            rssi: -80.0,
            ..Default::default()
        };
        for i in 0..9 {
            s.packet(10.0 + i as f64 * 0.05, &r, false);
        }
        s.missed(10.5, false);
        let x = s.sample(10.6);
        assert_eq!(x.get("lq_pct"), Some(90.0));
        assert_eq!(x.get("rssi_dbm"), Some(-80.0));
    }
}
