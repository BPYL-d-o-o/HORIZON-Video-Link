//! "Export video": a part of a session (or all of it) as an .mp4. The packets are run through the same receiver engine
//! as live, and the access units go into the file as received (no re-encoding), timed by the transmitter's own
//! timestamps. It starts 2 s before the chosen part, so the first pictures already have what they are predicted from.
use crate::decoder::{Decoder, Mux};
use crate::engine::{self, IdentAssembler, Stream, StreamSink};
use crate::session;
use std::io::BufRead;
use std::path::{Path, PathBuf};

const LEAD_S: f64 = 2.0;

struct Collect(Vec<Vec<u8>>);
impl StreamSink for Collect {
    fn au(&mut self, au: Vec<u8>, _after_gap: bool) {
        self.0.push(au);
    }
    fn source(&mut self, _lost: bool) {}
    fn event(&mut self, _text: String) {}
}

/// NAL unit types in an Annex-B access unit.
fn nal_types(au: &[u8]) -> Vec<(u8, usize, usize)> {
    let mut starts = Vec::new();
    let mut i = 0;
    while i + 3 <= au.len() {
        if au[i] == 0 && au[i + 1] == 0 && au[i + 2] == 1 {
            starts.push(i + 3);
            i += 3;
        } else {
            i += 1;
        }
    }
    let mut out = Vec::new();
    for (k, &s) in starts.iter().enumerate() {
        let mut end = if k + 1 < starts.len() {
            starts[k + 1] - 3
        } else {
            au.len()
        };
        while end > s && au[end - 1] == 0 {
            end -= 1;
        }
        if s < au.len() {
            out.push(((au[s] >> 1) & 0x3f, s, end));
        }
    }
    out
}

/// The VPS, SPS and PPS of an access unit, as Annex-B (the mp4 header needs them), if it carries all three.
fn parameter_sets(au: &[u8]) -> Option<Vec<u8>> {
    let mut out = Vec::new();
    let mut have = [false; 3];
    for (t, s, e) in nal_types(au) {
        if (32..=34).contains(&t) {
            have[(t - 32) as usize] = true;
            out.extend_from_slice(&[0, 0, 0, 1]);
            out.extend_from_slice(&au[s..e]);
        }
    }
    if have.iter().all(|h| *h) {
        Some(out)
    } else {
        None
    }
}

fn is_key(au: &[u8]) -> bool {
    nal_types(au).iter().any(|(t, _, _)| (16..=21).contains(t))
}

/// Writes `out` from the session in `dir`, `from`..`to` unix seconds. `progress(fraction)` is called as it goes.
pub fn export(dir: &Path, from: f64, to: f64, out: &Path, progress: &mut dyn FnMut(f64)) -> Result<PathBuf, String> {
    let rx = dir.join("packets.rxlog");
    if !rx.is_file() {
        return Err("this session has no packet capture (packets.rxlog): there is no video to export".into());
    }
    let idx = session::index_rxlog(&rx).map_err(|e| e.to_string())?;
    let reader = session::open_rxlog_at(&rx, &idx, from - LEAD_S - 1.0).map_err(|e| e.to_string())?;
    let mut stream = Stream::default();
    stream.link = crate::engine::recording_link(&rx).unwrap_or(0); // the flight's own link ID
    let mut idents = IdentAssembler::default();
    let mut sink = Collect(Vec::new());
    let mut dec = Decoder::load()?;
    let mut px = Vec::new();
    let mut size: Option<(u32, u32)> = None;
    let mut mux: Option<Mux> = None;
    let mut pending: Vec<(Vec<u8>, i64)> = Vec::new(); // until the first picture gives the size
    let mut extradata: Option<Vec<u8>> = None;
    let mut written = 0u64;
    for line in reader.lines() {
        let line = line.map_err(|e| e.to_string())?;
        let Some((t, text)) = session::line_time(&line) else {
            continue;
        };
        if t > to {
            break;
        }
        engine::line(text, &mut stream, &mut sink, &mut idents);
        for au in sink.0.drain(..) {
            if t < from - LEAD_S {
                continue;
            }
            let Some((data, sent)) = engine::prepare_au(&au) else {
                continue;
            };
            let time_us = sent.map(|s| s as i64).unwrap_or((t * 1e6) as i64);
            if extradata.is_none() {
                extradata = parameter_sets(&data);
                if extradata.is_none() {
                    continue; // the file starts at an access unit that carries the parameter sets
                }
            }
            if size.is_none() {
                dec.send(&data);
                size = dec.picture(&mut px);
                pending.push((data, time_us));
                if let Some((w, h)) = size {
                    let mut m = Mux::create(out, extradata.as_ref().unwrap(), w, h)?;
                    for (i, (d, tu)) in pending.drain(..).enumerate() {
                        if m.write(&d, tu, i == 0 || is_key(&d)) {
                            written += 1;
                        }
                    }
                    mux = Some(m);
                }
                continue;
            }
            if let Some(m) = mux.as_mut() {
                if m.write(&data, time_us, is_key(&data)) {
                    written += 1;
                }
            }
        }
        if to > from {
            progress(((t - from) / (to - from)).clamp(0.0, 1.0));
        }
    }
    let m = mux.ok_or("no decodable video in that part of the session")?;
    if !m.finish() || written == 0 {
        return Err(format!("could not finish {}", out.display()));
    }
    progress(1.0);
    Ok(out.to_path_buf())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn finds_parameter_sets() {
        // VPS (32), SPS (33), PPS (34), then a slice (1)
        let au = [
            0u8, 0, 0, 1, 64, 1, 9, 0, 0, 1, 66, 1, 8, 0, 0, 1, 68, 1, 7, 0, 0, 1, 2, 1, 5,
        ];
        let ps = parameter_sets(&au).unwrap();
        assert_eq!(
            ps,
            vec![0, 0, 0, 1, 64, 1, 9, 0, 0, 0, 1, 66, 1, 8, 0, 0, 0, 1, 68, 1, 7]
        );
        assert!(!is_key(&au));
    }
}
