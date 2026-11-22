//! Runs a session's packets.rxlog through the engine, the decoder and the lost-area hold, and prints per picture after
//! a loss how many 8x8 blocks were held (to check the hold catches the decoder's grey):  hold_check SESSION_DIR
use horizon_receiver::decoder::Decoder;
use horizon_receiver::engine::{self, IdentAssembler, Stream, StreamSink};
use horizon_receiver::hold::Hold;
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
    let dir = std::env::args().nth(1).expect("usage: hold_check SESSION_DIR");
    let f = std::io::BufReader::new(std::fs::File::open(std::path::Path::new(&dir).join("packets.rxlog")).unwrap());
    let (mut st, mut id, mut s) = (Stream::default(), IdentAssembler::default(), S(Vec::new()));
    let mut dec = Decoder::load().unwrap();
    let mut hold = Hold::default();
    let (mut px, mut gap, mut n, mut since) = (Vec::new(), false, 0u64, 999u64);
    for l in f.lines() {
        let l = l.unwrap();
        let Some((_, text)) = horizon_receiver::session::line_time(&l) else { continue };
        engine::line(text, &mut st, &mut s, &mut id);
        for (au, g) in s.0.drain(..) {
            let Some((d, _)) = engine::prepare_au(&au) else { continue };
            gap |= g;
            dec.send(&d);
            if let Some((w, h)) = dec.picture(&mut px) {
                n += 1;
                let g = std::mem::take(&mut gap);
                if g {
                    since = 0;
                }
                let held = hold.frame(&mut px, w, h, g);
                if since < 100 && (held > 0 || since < 3) {
                    println!("picture {n} ({w}x{h}) +{since}: held {held} of {} blocks", (w / 8) * (h / 8));
                }
                since += 1;
            }
        }
    }
    println!("RESULT {n} pictures");
}
