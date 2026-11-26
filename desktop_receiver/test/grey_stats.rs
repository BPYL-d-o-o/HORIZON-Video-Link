//! Which 8x8 blocks are flat and colourless, by brightness, in clean pictures and in the 10 pictures after a loss:
//! to tune what the lost-area hold treats as the decoder's grey.   grey_stats SESSION_DIR
use horizon_receiver::decoder::Decoder;
use horizon_receiver::engine::{self, IdentAssembler, Stream, StreamSink};
use horizon_receiver::hold::flat_grey;
use std::io::BufRead;

struct S(Vec<(Vec<u8>, bool)>);
impl StreamSink for S {
    fn au(&mut self, au: Vec<u8>, g: bool) {
        self.0.push((au, g));
    }
    fn source(&mut self, _: bool) {}
    fn event(&mut self, _: String) {}
}
fn main() {
    let dir = std::env::args().nth(1).expect("usage: grey_stats SESSION_DIR");
    let f = std::io::BufReader::new(std::fs::File::open(std::path::Path::new(&dir).join("packets.rxlog")).unwrap());
    let (mut st, mut id, mut s) = (Stream::default(), IdentAssembler::default(), S(Vec::new()));
    let mut dec = Decoder::load().unwrap();
    let mut px = Vec::new();
    let mut since = 1000u32;
    // [clean, after loss] x 13 brightness buckets of 20
    let mut hist = [[0u64; 13]; 3];
    let mut blocks = [0u64; 3];
    let mut gap = false;
    for l in f.lines() {
        let l = l.unwrap();
        let Some((_, text)) = horizon_receiver::session::line_time(&l) else {
            continue;
        };
        engine::line(text, &mut st, &mut s, &mut id);
        for (au, g) in s.0.drain(..) {
            let Some((d, _)) = engine::prepare_au(&au) else {
                continue;
            };
            gap |= g;
            dec.send(&d);
            if let Some((w, h)) = dec.picture(&mut px) {
                if std::mem::take(&mut gap) {
                    since = 0;
                }
                let class = if since < 10 {
                    1
                } else if since > 150 {
                    0
                } else {
                    2
                };
                since += 1;
                let (bw, bh) = (w as usize / 8, h as usize / 8);
                blocks[class] += (bw * bh) as u64;
                for by in 0..bh {
                    for bx in 0..bw {
                        if let Some(g) = flat_grey(&px, w as usize, bx, by) {
                            hist[class][(g / 20).clamp(0, 12) as usize] += 1;
                        }
                    }
                }
            }
        }
    }
    println!("level     clean %   0-9 after %  10-150 after %");
    for b in 0..13 {
        let p = |c: usize| 100.0 * hist[c][b] as f64 / blocks[c].max(1) as f64;
        println!(
            "{:3}-{:3}   {:7.3}   {:9.3}   {:9.3}",
            b * 20,
            b * 20 + 19,
            p(0),
            p(1),
            p(2)
        );
    }
    println!(
        "blocks: clean {} after loss {} later {}",
        blocks[0], blocks[1], blocks[2]
    );
}
