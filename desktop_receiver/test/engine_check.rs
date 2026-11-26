//! Runs a receiver capture through the Rust engine and prints the same lines as setup_site/test/check_swfec.mjs
//! (the JavaScript receiver), so the two can be compared with diff:
//!   engine_check CAPTURE [MAX_LINES] > rust.txt
//!   node setup_site/test/check_swfec.mjs CAPTURE [MAX_LINES] > js.txt && diff js.txt rust.txt
use horizon_receiver::engine::{self, IdentAssembler, Stream, StreamSink};
use std::io::{BufRead, BufReader, Write};

struct Out {
    lines: Vec<String>,
    lost: u64,
    recovered: u64,
}
impl StreamSink for Out {
    fn au(&mut self, au: Vec<u8>, after_gap: bool) {
        if let Some((data, _)) = engine::prepare_au(&au) {
            self.lines.push(format!(
                "AU {} {}",
                sha1_smol::Sha1::from(&data).digest(),
                after_gap as u8
            ));
        }
    }
    fn source(&mut self, lost: bool) {
        if lost {
            self.lost += 1;
        }
    }
    fn event(&mut self, text: String) {
        self.lines
            .push(format!("EVENT {}", text.split(':').next().unwrap_or("")));
    }
    fn recovered(&mut self) {
        self.recovered += 1;
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).expect("usage: engine_check CAPTURE [MAX_LINES]");
    let limit: usize = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(usize::MAX);
    let f = BufReader::new(std::fs::File::open(path).expect("capture"));
    let mut stream = Stream::default();
    let mut idents = IdentAssembler::default();
    let mut out = Out {
        lines: Vec::new(),
        lost: 0,
        recovered: 0,
    };
    let stdout = std::io::stdout();
    let mut w = stdout.lock();
    for (n, raw) in f.split(b'\n').enumerate() {
        if n >= limit {
            break;
        }
        let raw = String::from_utf8_lossy(&raw.unwrap())
            .trim_end_matches('\r')
            .to_string();
        let text = match raw.find(' ') {
            Some(i) => &raw[i + 1..],
            None => &raw,
        };
        engine::line(text, &mut stream, &mut out, &mut idents);
        for l in out.lines.drain(..) {
            writeln!(w, "{l}").unwrap();
        }
    }
    writeln!(w, "LOST {}", out.lost).unwrap();
    writeln!(w, "RECOVERED {}", out.recovered).unwrap();
}
