//! Display only ("Hold lost areas"): after a lost packet the decoder paints the parts
//! of the picture it has no reference for in flat grey, which the transmitter's row refresh repaints within one cycle
//! (2.7 s at 30 fps). Until then those blocks show the last picture that was on screen instead. The grey also spreads
//! for a while (later pictures predict from it), so for HOLD_MAX after the latest loss every block that is grey in the
//! new picture is held, not only those grey right after the loss; a block shows again as soon as it has real picture.
//! The decoder itself is untouched: this only changes what is shown.
use std::time::{Duration, Instant};

const HOLD_MAX: Duration = Duration::from_secs(4);
const B: usize = 8;
/// FFmpeg's grey stand-in (YUV 128,128,128) is about 130 in RGB; pictures predicted from it drift darker or lighter
/// (measured under injected loss: 83 % of the blocks right after a loss at 120-139, and later ones down to 60 and up
/// to 160). Flat colourless blocks anywhere in this range are taken for it: in clean pictures under 0.03 % are.
const GREY: std::ops::RangeInclusive<i32> = 60..=160;

#[derive(Default)]
pub struct Hold {
    prev: Vec<u8>,
    size: (u32, u32),
    /// the latest loss: holding while it is less than HOLD_MAX ago
    since: Option<Instant>,
}

/// The 8x8 block at (bx, by) if it is flat and colourless: its mean green level (for the analysis in hold_check).
pub fn flat_grey(px: &[u8], w: usize, bx: usize, by: usize) -> Option<i32> {
    let (mut sb, mut sg, mut sr, mut sg2) = (0i32, 0i32, 0i32, 0i32);
    for y in by * B..by * B + B {
        let row = &px[(y * w + bx * B) * 4..(y * w + bx * B + B) * 4];
        for p in row.chunks_exact(4) {
            let (b, g, r) = (p[0] as i32, p[1] as i32, p[2] as i32);
            sb += b;
            sg += g;
            sr += r;
            sg2 += g * g;
        }
    }
    let n = (B * B) as i32;
    let (mb, mg, mr) = (sb / n, sg / n, sr / n);
    let var = sg2 / n - mg * mg;
    (var <= 36 && (mb - mg).abs() <= 5 && (mr - mg).abs() <= 5).then_some(mg)
}

/// Is the 8x8 block at (bx, by) the decoder's grey stand-in: flat, mid grey, no colour?
/// (Measured in the Python app: 0.2 % of the blocks of clean pictures, 44-77 % right after a loss.)
fn greyish(px: &[u8], w: usize, bx: usize, by: usize) -> bool {
    flat_grey(px, w, bx, by).is_some_and(|g| GREY.contains(&g))
}

impl Hold {
    /// `px`: the new picture (BGRA, w x h), changed in place where held. Returns the number of blocks held.
    pub fn frame(&mut self, px: &mut [u8], w: u32, h: u32, after_gap: bool) -> usize {
        let (wu, hu) = (w as usize, h as usize);
        if self.size != (w, h) || self.prev.len() != px.len() {
            self.size = (w, h);
            self.prev = px.to_vec();
            self.since = None;
            return 0;
        }
        if after_gap {
            self.since = Some(Instant::now());
        }
        let mut held = 0;
        if self.since.is_some_and(|t| t.elapsed() <= HOLD_MAX) {
            for by in 0..hu / B {
                for bx in 0..wu / B {
                    if greyish(px, wu, bx, by) {
                        held += 1;
                        for y in by * B..by * B + B {
                            let a = (y * wu + bx * B) * 4;
                            px[a..a + B * 4].copy_from_slice(&self.prev[a..a + B * 4]);
                        }
                    }
                }
            }
        } else {
            self.since = None;
        }
        self.prev.copy_from_slice(px);
        held
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn holds_grey_until_refreshed() {
        let (w, h) = (16u32, 8u32);
        let pic = |v: u8| vec![v; (w * h * 4) as usize];
        let mut hold = Hold::default();
        let mut a = pic(40);
        assert_eq!(hold.frame(&mut a, w, h, false), 0);
        // a loss: both blocks grey, both held from the last picture
        let mut b = pic(130);
        assert_eq!(hold.frame(&mut b, w, h, true), 2);
        assert_eq!(b[0], 40);
        // the left block refreshed: only the right one is still held
        let mut c = pic(130);
        for y in 0..8 {
            for x in 0..8 {
                let i = ((y * w as usize) + x) * 4;
                c[i..i + 4].copy_from_slice(&[200, 30, 90, 255]);
            }
        }
        assert_eq!(hold.frame(&mut c, w, h, false), 1);
        assert_eq!(c[0], 200);
        assert_eq!(c[8 * 4], 40);
        // the grey spreads to the left block a picture later: held as well (from what was last shown there)
        let mut d = pic(130);
        assert_eq!(hold.frame(&mut d, w, h, false), 2);
        assert_eq!(d[0], 200);
    }
}
