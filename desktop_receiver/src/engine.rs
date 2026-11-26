//! HORIZON over-the-air format, receiver side (format/README.md): the Rust twin of setup_site/js/lib/swfec.js,
//! checked against format/packet.py. `engine_check` compares it with the JavaScript one on a capture.

use std::collections::{BTreeMap, BTreeSet};

pub const PAYLOAD: usize = 255;
pub const ENTRY: usize = 253;
pub const HDR: usize = 4;
pub const RS_PAR: usize = 16;
pub const SYM: usize = ENTRY - HDR - 2 - RS_PAR; // 231
const CRC_AT: usize = HDR + SYM;
pub const FILLER: u8 = 0xff;
pub const PARITY: u8 = 0x02; // low nibble of a block FEC parity entry's type

// GF(256), polynomial 0x11d
struct Gf {
    exp: [u8; 512],
    log: [u8; 256],
}
static GF: std::sync::LazyLock<Gf> = std::sync::LazyLock::new(|| {
    let mut g = Gf {
        exp: [0; 512],
        log: [0; 256],
    };
    let mut x: u32 = 1;
    for i in 0..255 {
        g.exp[i] = x as u8;
        g.log[x as usize] = i as u8;
        x <<= 1;
        if x & 0x100 != 0 {
            x ^= 0x11d;
        }
    }
    for i in 255..512 {
        g.exp[i] = g.exp[i - 255];
    }
    g
});
fn gmul(a: u8, b: u8) -> u8 {
    if a == 0 || b == 0 {
        0
    } else {
        GF.exp[GF.log[a as usize] as usize + GF.log[b as usize] as usize]
    }
}
fn ginv(a: u8) -> u8 {
    GF.exp[255 - GF.log[a as usize] as usize]
}
fn gpow(x: u8, p: usize) -> u8 {
    if p == 0 {
        1
    } else if x == 0 {
        0
    } else {
        GF.exp[(GF.log[x as usize] as usize * p) % 255]
    }
}

static CRC16: std::sync::LazyLock<[u16; 256]> = std::sync::LazyLock::new(|| {
    let mut t = [0u16; 256];
    for (i, v) in t.iter_mut().enumerate() {
        let mut c = i as u16;
        for _ in 0..8 {
            c = if c & 1 != 0 { 0xa001 ^ (c >> 1) } else { c >> 1 };
        }
        *v = c;
    }
    t
});
pub fn crc16(d: &[u8]) -> u16 {
    let mut c: u16 = 0xffff;
    for &b in d {
        c = CRC16[((c ^ b as u16) & 0xff) as usize] ^ (c >> 8);
    }
    c
}
/// The entry's CRC-16, keyed with the link ID (format/README.md "Link ID"; 0 = unkeyed).
fn crc_ok(e: &[u8], link: u16) -> bool {
    crc16(&e[..CRC_AT]) ^ link == (e[CRC_AT] as u16 | (e[CRC_AT + 1] as u16) << 8)
}
/// The link ID a recording was made with (its first "C cfg ... link=N" line), so a replay or export starts right.
pub fn recording_link(path: &std::path::Path) -> Option<u16> {
    use std::io::BufRead;
    let f = std::fs::File::open(path).ok()?;
    for line in std::io::BufReader::new(f).lines().take(20000) {
        let line = line.ok()?;
        if let Some(i) = line.find("C cfg ") {
            return line[i..]
                .split_whitespace()
                .find_map(|kv| kv.strip_prefix("link="))
                .and_then(|v| v.parse().ok());
        }
    }
    None
}

// RS(253,237): errors and erasures
fn poly_mul(p: &[u8], q: &[u8]) -> Vec<u8> {
    let mut r = vec![0u8; p.len() + q.len() - 1];
    for (j, &qj) in q.iter().enumerate() {
        if qj == 0 {
            continue;
        }
        for (i, &pi) in p.iter().enumerate() {
            if pi != 0 {
                r[i + j] ^= GF.exp[GF.log[pi as usize] as usize + GF.log[qj as usize] as usize];
            }
        }
    }
    r
}
fn poly_eval(p: &[u8], x: u8) -> u8 {
    let mut y = p[0];
    for &c in &p[1..] {
        y = gmul(y, x) ^ c;
    }
    y
}
fn poly_scale(p: &[u8], x: u8) -> Vec<u8> {
    p.iter().map(|&c| gmul(c, x)).collect()
}
fn poly_add(p: &[u8], q: &[u8]) -> Vec<u8> {
    let n = p.len().max(q.len());
    let mut r = vec![0u8; n];
    for (i, &c) in p.iter().enumerate() {
        r[i + n - p.len()] = c;
    }
    for (i, &c) in q.iter().enumerate() {
        r[i + n - q.len()] ^= c;
    }
    r
}
fn syndromes(cw: &[u8]) -> Vec<u8> {
    let mut s = vec![0u8];
    for i in 0..RS_PAR {
        s.push(poly_eval(cw, GF.exp[i]));
    }
    s
}
fn forney_syndromes(synd: &[u8], pos: &[usize], n: usize) -> Vec<u8> {
    let mut fs = synd[1..].to_vec();
    for &p in pos {
        let x = gpow(2, n - 1 - p);
        for j in 0..fs.len() - 1 {
            fs[j] = gmul(fs[j], x) ^ fs[j + 1];
        }
    }
    fs
}
fn error_locator(synd: &[u8], nsym: usize, erase_count: usize) -> Option<Vec<u8>> {
    let mut err_loc = vec![1u8];
    let mut old_loc = vec![1u8];
    let shift = synd.len().saturating_sub(nsym);
    for i in 0..nsym.saturating_sub(erase_count) {
        let k = i + shift;
        let mut delta = synd[k];
        for j in 1..err_loc.len() {
            delta ^= gmul(err_loc[err_loc.len() - 1 - j], synd[k - j]);
        }
        old_loc.push(0);
        if delta != 0 {
            if old_loc.len() > err_loc.len() {
                let new_loc = poly_scale(&old_loc, delta);
                old_loc = poly_scale(&err_loc, ginv(delta));
                err_loc = new_loc;
            }
            err_loc = poly_add(&err_loc, &poly_scale(&old_loc, delta));
        }
    }
    while !err_loc.is_empty() && err_loc[0] == 0 {
        err_loc.remove(0);
    }
    if err_loc.is_empty() || (err_loc.len() - 1).saturating_sub(erase_count) * 2 + erase_count > nsym {
        return None;
    }
    Some(err_loc)
}
fn find_errors(err_loc: &[u8], n: usize) -> Option<Vec<usize>> {
    let errs = err_loc.len() - 1;
    let pos: Vec<usize> = (0..n)
        .filter(|&i| poly_eval(err_loc, gpow(2, i)) == 0)
        .map(|i| n - 1 - i)
        .collect();
    if pos.len() == errs {
        Some(pos)
    } else {
        None
    }
}
fn correct_errata(cw: &[u8], synd: &[u8], err_pos: &[usize]) -> Option<Vec<u8>> {
    let n = cw.len();
    let coef_pos: Vec<usize> = err_pos.iter().map(|&p| n - 1 - p).collect();
    let mut loc = vec![1u8];
    for &p in &coef_pos {
        loc = poly_mul(&loc, &poly_add(&[1], &[gpow(2, p), 0]));
    }
    let rev: Vec<u8> = synd.iter().rev().copied().collect();
    let prod = poly_mul(&rev, &loc);
    let mut ev: Vec<u8> = prod[prod.len() - loc.len()..].to_vec();
    ev.reverse();
    let xs: Vec<u8> = coef_pos.iter().map(|&p| GF.exp[p % 255]).collect();
    let ev_rev: Vec<u8> = ev.iter().rev().copied().collect();
    let mut out = cw.to_vec();
    for i in 0..xs.len() {
        let xi_inv = ginv(xs[i]);
        let mut prime = 1u8;
        for (j, &xj) in xs.iter().enumerate() {
            if j != i {
                prime = gmul(prime, 1 ^ gmul(xi_inv, xj));
            }
        }
        if prime == 0 {
            return None;
        }
        let y = gmul(xs[i], poly_eval(&ev_rev, xi_inv));
        out[err_pos[i]] ^= gmul(y, ginv(prime));
    }
    Some(out)
}

/// cw: 253 bytes. The corrected codeword or None.
pub fn rs_decode(cw: &[u8], erasures: &[usize]) -> Option<Vec<u8>> {
    let mut cw = cw.to_vec();
    let mut er: Vec<usize> = erasures.to_vec();
    er.sort_unstable();
    er.dedup();
    if er.len() > RS_PAR {
        return None;
    }
    for &p in &er {
        cw[p] = 0;
    }
    let synd = syndromes(&cw);
    if synd.iter().all(|&v| v == 0) {
        return Some(cw);
    }
    let fs = forney_syndromes(&synd, &er, cw.len());
    let loc = error_locator(&fs, RS_PAR, er.len())?;
    let rev: Vec<u8> = loc.iter().rev().copied().collect();
    let errs = find_errors(&rev, cw.len())?;
    let mut all = er.clone();
    all.extend(errs);
    let out = correct_errata(&cw, &synd, &all)?;
    if !syndromes(&out).iter().all(|&v| v == 0) {
        return None;
    }
    Some(out)
}

/// Block FEC Cauchy matrix entry, parity row j, source column i: 1 / ((k + j) xor i) (never 0: i < k).
fn cauchy(k: usize, j: usize, i: usize) -> u8 {
    ginv(((k + j) ^ i) as u8)
}
fn mul_acc(acc: &mut [u8], c: u8, v: &[u8]) {
    if c == 0 {
        return;
    }
    let lc = GF.log[c as usize] as usize;
    for (a, &b) in acc.iter_mut().zip(v) {
        if b != 0 {
            *a ^= GF.exp[GF.log[b as usize] as usize + lc];
        }
    }
}
/// k equal-length source symbols -> m parity symbols (blockfec.encode).
pub fn bfec_encode(symbols: &[&[u8]], m: usize) -> Vec<Vec<u8>> {
    let k = symbols.len();
    (0..m)
        .map(|j| {
            let mut acc = vec![0u8; symbols[0].len()];
            for (i, s) in symbols.iter().enumerate() {
                mul_acc(&mut acc, cauchy(k, j, i), s);
            }
            acc
        })
        .collect()
}
/// have: received sources (index 0..k-1, symbol); parity: received parities (index j, symbol), any order.
/// -> (index, symbol) of every missing source, or None when more are missing than parities arrived (blockfec.decode).
pub fn bfec_decode(k: usize, have: &[(usize, &[u8])], parity: &[(usize, &[u8])]) -> Option<Vec<(usize, Vec<u8>)>> {
    let miss: Vec<usize> = (0..k).filter(|i| !have.iter().any(|h| h.0 == *i)).collect();
    if miss.is_empty() {
        return Some(Vec::new());
    }
    let mut rows: Vec<&(usize, &[u8])> = parity.iter().collect();
    rows.sort_by_key(|r| r.0);
    rows.truncate(miss.len());
    if rows.len() < miss.len() {
        return None;
    }
    let n = miss.len();
    // right-hand side: parity j minus the known sources' contribution
    let mut rhs: Vec<Vec<u8>> = rows
        .iter()
        .map(|&&(j, p)| {
            let mut acc = p.to_vec();
            for &(i, s) in have {
                mul_acc(&mut acc, cauchy(k, j, i), s);
            }
            acc
        })
        .collect();
    let mut a: Vec<Vec<u8>> = rows
        .iter()
        .map(|r| miss.iter().map(|&i| cauchy(k, r.0, i)).collect())
        .collect();
    // Gauss-Jordan over GF(256)
    for col in 0..n {
        let piv = (col..n).find(|&r| a[r][col] != 0)?;
        a.swap(col, piv);
        rhs.swap(col, piv);
        let inv = ginv(a[col][col]);
        for v in a[col].iter_mut() {
            *v = gmul(*v, inv);
        }
        for v in rhs[col].iter_mut() {
            *v = gmul(*v, inv);
        }
        for r in 0..n {
            let f = a[r][col];
            if r != col && f != 0 {
                let (pr, pc) = (a[col].clone(), rhs[col].clone());
                for (x, y) in a[r].iter_mut().zip(&pr) {
                    *x ^= gmul(f, *y);
                }
                mul_acc(&mut rhs[r], f, &pc);
            }
        }
    }
    Some(miss.into_iter().zip(rhs).collect())
}

/// The TX RP2350's own packets: idle filler (FF FF FF), command ACK / power announcement (FF FF FE).
pub fn is_filler(e: &[u8]) -> bool {
    e[0] == FILLER && e[1] == 0xff && (e[2] == 0xff || e[2] == 0xfe)
}
/// The Air status in the TX's idle packets (radio_firmware/src/common.h "Air status"): why there is no video.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirStatus {
    /// AIR_* below
    pub state: u8,
    /// seconds since the Air's camera computer last sent video (None: not since the radio started)
    pub since_video: Option<u16>,
    /// seconds since the camera computer last talked to the radio at all (None: never)
    pub since_host: Option<u16>,
    /// what the camera computer reported: 0 nothing, 1 no camera, 2 camera does not start, 3 encoder does not
    /// start, 4 other start-up error
    pub error: u8,
    /// seconds since the Air's radio started
    pub up_s: u32,
}
pub const AIR_VIDEO: u8 = 1;
pub const AIR_NO_VIDEO: u8 = 2;
pub const AIR_SILENT: u8 = 3;
pub const AIR_NO_HOST: u8 = 4;
pub const AIR_STOPPING: u8 = 5;
pub const AIR_ERROR: u8 = 6;
/// The Air status in an idle entry, if it carries one.
pub fn air_status(e: &[u8]) -> Option<AirStatus> {
    if e.len() < 21 || e[0..3] != [0xff, 0xff, 0xff] || e[3..6] != [b'H', b'S', 1] {
        return None;
    }
    if e[3..20].iter().fold(0x5au8, |a, b| a ^ b) != e[20] {
        return None;
    }
    let u16_at = |i: usize| u16::from_le_bytes([e[i], e[i + 1]]);
    let age = |v: u16| if v == 0xffff { None } else { Some(v) };
    Some(AirStatus {
        state: e[6],
        since_video: age(u16_at(7)),
        since_host: age(u16_at(9)),
        error: e[11],
        up_s: u32::from_le_bytes([e[12], e[13], e[14], e[15]]),
    })
}
pub fn check_entry(e: &[u8], link: u16) -> bool {
    e.len() == ENTRY && crc_ok(e, link)
}
/// An entry from a packet that FAILED the radio CRC: corrected bytes or None.
pub fn repair_entry(e: &[u8], link: u16) -> Option<Vec<u8>> {
    if e.len() != ENTRY {
        return None;
    }
    if crc_ok(e, link) {
        return Some(e.to_vec());
    }
    let out = rs_decode(e, &[])?;
    if crc_ok(&out, link) {
        Some(out)
    } else {
        None
    }
}

// byte stream -> access units
const AUD: [u8; 7] = [0, 0, 0, 1, 0x46, 0x01, 0x50];

fn au_start(d: &[u8]) -> bool {
    let mut i = 0;
    while i < d.len() && d[i] == 0 {
        i += 1;
    }
    if i < 2 || i + 3 > d.len() || d[i] != 1 {
        return false;
    }
    let t = (d[i + 1] >> 1) & 0x3f;
    matches!(t, 32 | 33 | 34 | 35 | 39) || (t <= 31 && i + 3 < d.len() && d[i + 3] & 0x80 != 0)
}
fn non_zero(d: &[u8]) -> bool {
    d.iter().any(|&v| v != 0)
}
fn find_aud(buf: &[u8], from: usize) -> Option<usize> {
    let mut i = from;
    while i + AUD.len() <= buf.len() {
        if buf[i] == 0 && buf[i + 3] == 1 && buf[i + 4] == 0x46 && buf[i..i + AUD.len()] == AUD {
            return Some(i);
        }
        i += 1;
    }
    None
}

/// What the stream reports: complete access units, source packets (received or lost), resyncs.
pub trait StreamSink {
    fn au(&mut self, au: Vec<u8>, after_gap: bool);
    fn source(&mut self, lost: bool);
    fn event(&mut self, text: String);
    /// a lost video packet rebuilt by the block FEC (also reported as source(false))
    fn recovered(&mut self) {}
}

struct PathSplitter {
    buf: Vec<u8>,
    skip: bool,
    after_gap: bool,
    fresh: bool,
    pub dropped: u64,
}
impl PathSplitter {
    fn new() -> Self {
        PathSplitter {
            buf: Vec::with_capacity(65536),
            skip: true,
            after_gap: true,
            fresh: false,
            dropped: 0,
        }
    }
    fn start_at_boundary(&mut self) {
        self.buf.clear();
        self.skip = false;
    }
    fn gap(&mut self) {
        if non_zero(&self.buf) && !self.skip {
            self.dropped += 1;
        }
        self.buf.clear();
        self.skip = true;
        self.after_gap = true;
        self.fresh = true;
    }
    fn data(&mut self, b: &[u8], sink: &mut dyn StreamSink) {
        if self.fresh && non_zero(b) {
            self.fresh = false; // the packet after a lost one may start exactly at a new access unit: keep that one
            if self.skip && au_start(b) {
                self.skip = false;
            }
        }
        let from = self.buf.len().saturating_sub(AUD.len() - 1);
        self.buf.extend_from_slice(b);
        let mut start = 0;
        let mut found = find_aud(&self.buf, from);
        while let Some(i) = found {
            if self.skip {
                // the AU the stream resumed in lost its start: drop it
                if non_zero(&self.buf[start..i]) {
                    self.dropped += 1;
                }
                self.skip = false;
            } else if non_zero(&self.buf[start..i]) {
                sink.au(self.buf[start..i].to_vec(), self.after_gap);
                self.after_gap = false;
            }
            start = i + AUD.len();
            found = find_aud(&self.buf, start);
        }
        if start > 0 {
            self.buf.drain(..start);
        }
        if self.buf.len() > 200000 {
            self.gap(); // no delimiter for far too long: resynchronise
        }
    }
}

/// Checked entries in, access units out.
///
/// While block FEC parity is seen, the packets after a lost video packet are held until the parity rebuilds it, or
/// until it can no longer be rebuilt (a later block's parity arrived, or FEC_GIVEUP packets went by): then it is
/// delivered as a gap. Without parity packets nothing is held.
pub struct Stream {
    next: Option<i64>,
    split: PathSplitter,
    fec_last: Option<i64>,                 // source id when the last parity packet arrived
    src: BTreeMap<i64, Vec<u8>>,           // recent video symbols (for rebuilding)
    par: BTreeMap<(i64, usize), ParBlock>, // (first id of a block, d) -> its parity
    missing: BTreeSet<i64>,                // lost video packets that may still be rebuilt
    held: BTreeMap<i64, Vec<u8>>,          // received or rebuilt, waiting behind a missing one
    giveup_below: Option<i64>,             // missing ids below this can no longer be rebuilt
    giveup_n: i64,
    /// video packets rebuilt by the block FEC (total, kept across resyncs)
    pub recovered: u64,
    /// the link ID packets are checked with (from the receiver's "C cfg ... link=N"; 0 = unkeyed)
    pub link: u16,
}
struct ParBlock {
    k: usize,
    d: usize,
    p: BTreeMap<usize, Vec<u8>>,
}
const FEC_GIVEUP: i64 = 40; // video packets after a lost one at most before it is given up
const FEC_ACTIVE: i64 = 400; // FEC counts as in use while a parity packet came within this many video packets
impl Default for Stream {
    fn default() -> Self {
        Stream {
            next: None,
            split: PathSplitter::new(),
            fec_last: None,
            src: BTreeMap::new(),
            par: BTreeMap::new(),
            missing: BTreeSet::new(),
            held: BTreeMap::new(),
            giveup_below: None,
            giveup_n: FEC_GIVEUP,
            recovered: 0,
            link: 0,
        }
    }
}
impl Stream {
    fn reset(&mut self) {
        let recovered = self.recovered;
        let link = self.link;
        *self = Stream {
            recovered,
            link,
            ..Stream::default()
        };
    }
    fn unwrap_id(&self, id16: i64) -> i64 {
        let next = self.next.unwrap_or(0);
        let d = (id16 - next) & 0xffff;
        next + if d >= 0x8000 { d - 0x10000 } else { d }
    }
    fn fec_on(&self, sid: i64) -> bool {
        self.fec_last.is_some_and(|f| sid - f < FEC_ACTIVE)
    }
    fn parity(&mut self, e: &[u8], sink: &mut dyn StreamSink) {
        let Some(next) = self.next else { return };
        let first = self.unwrap_id(e[1] as i64 | (e[2] as i64) << 8);
        let (k, j, d) = ((e[3] >> 4) as usize + 1, (e[3] & 15) as usize, (e[0] >> 4) as usize + 1);
        self.fec_last = Some(next);
        let b = self.par.entry((first, d)).or_insert_with(|| ParBlock {
            k,
            d,
            p: BTreeMap::new(),
        });
        b.p.insert(j, e[HDR..HDR + SYM].to_vec());
        let jmax = *b.p.keys().next_back().unwrap() as i64;
        // how long a lost packet may wait for its parity: about one group (d interleaved blocks of k + m packets)
        self.giveup_n = FEC_GIVEUP.max(3 * d as i64 * (k as i64 + jmax + 1) / 2);
        // parity comes block after block: once a later block's parity arrives, what earlier blocks miss is lost
        if self.missing.first().is_some_and(|&m| m < first) {
            self.giveup_below = Some(self.giveup_below.map_or(first, |g| g.max(first)));
        }
        self.rebuild((first, d), sink);
        self.release(sink);
    }
    fn rebuild(&mut self, key: (i64, usize), sink: &mut dyn StreamSink) {
        let Some(b) = self.par.get(&key) else { return };
        let ids: Vec<i64> = (0..b.k).map(|n| key.0 + (n * b.d) as i64).collect();
        let nmiss = ids.iter().filter(|s| !self.src.contains_key(s)).count();
        if nmiss == 0 || nmiss > b.p.len() {
            return;
        }
        let have: Vec<(usize, &[u8])> = ids
            .iter()
            .enumerate()
            .filter_map(|(n, s)| self.src.get(s).map(|v| (n, v.as_slice())))
            .collect();
        let parity: Vec<(usize, &[u8])> = b.p.iter().map(|(j, v)| (*j, v.as_slice())).collect();
        let Some(r) = bfec_decode(b.k, &have, &parity) else {
            return;
        };
        for (i, sym) in r {
            let s = ids[i];
            self.src.insert(s, sym.clone());
            if self.missing.remove(&s) {
                self.held.insert(s, sym);
                self.recovered += 1;
                sink.recovered();
                sink.source(false);
            }
        }
    }
    /// Delivers held packets in order up to the first one still missing; gives up missing ones that cannot come.
    fn release(&mut self, sink: &mut dyn StreamSink) {
        let next = self.next.unwrap_or(0);
        while let Some(&m0) = self.missing.first() {
            if !(self.giveup_below.is_some_and(|g| m0 < g) || next - 1 - m0 >= self.giveup_n) {
                break;
            }
            let mut end = m0;
            while self.missing.contains(&(end + 1)) {
                end += 1;
            }
            let rest = self.held.split_off(&m0);
            for (_, v) in std::mem::replace(&mut self.held, rest) {
                self.split.data(&v, sink);
            }
            for s in m0..=end {
                self.missing.remove(&s);
                sink.source(true);
            }
            self.split.gap();
        }
        let keep = match self.missing.first() {
            Some(&limit) => self.held.split_off(&limit),
            None => BTreeMap::new(),
        };
        for (_, v) in std::mem::replace(&mut self.held, keep) {
            self.split.data(&v, sink);
        }
    }
    pub fn entry(&mut self, e: &[u8], sink: &mut dyn StreamSink) {
        if e[0] & 0x0f == PARITY {
            return self.parity(e, sink);
        }
        if e[0] != 0 {
            return; // filler and anything newer: not used by this receiver
        }
        let id16 = e[1] as i64 | (e[2] as i64) << 8;
        let sid;
        match self.next {
            None => {
                sid = id16;
                if sid == 0 {
                    self.split.start_at_boundary(); // the transmitter's very first packet starts at a boundary
                }
            }
            Some(next) => {
                let d = (id16 - next) & 0xffff;
                if d >= 0x8000 {
                    // older than expected: a duplicate, or the transmitter restarted
                    if 0x10000 - d > 3000 {
                        sink.event(format!("resync: packet {id16}, expected {}", next & 0xffff));
                        self.reset();
                        return self.entry(e, sink);
                    }
                    return;
                }
                if d > 3000 {
                    sink.event(format!("resync: packet {id16}, expected {}", next & 0xffff));
                    self.reset();
                    return self.entry(e, sink);
                }
                sid = next + d;
                if d > 0 {
                    if self.fec_on(sid) {
                        for s in sid - d..sid {
                            if let Some(v) = self.src.get(&s) {
                                // already rebuilt (the block's last packets: parity came first)
                                self.held.insert(s, v.clone());
                                self.recovered += 1;
                                sink.recovered();
                                sink.source(false);
                            } else {
                                self.missing.insert(s); // may still be rebuilt from the block's parity
                            }
                        }
                    } else {
                        for _ in 0..d {
                            sink.source(true);
                        }
                        self.split.gap();
                    }
                }
            }
        }
        sink.source(false);
        self.next = Some(sid + 1);
        let sym = &e[HDR..HDR + SYM];
        if self.fec_on(sid) {
            self.src.insert(sid, sym.to_vec());
            if self.src.len() > 1024 {
                self.src = self.src.split_off(&(sid - 512));
                self.par = self.par.split_off(&(sid - 512, 0));
            }
            if !self.missing.is_empty() || !self.held.is_empty() {
                self.held.insert(sid, sym.to_vec());
                self.release(sink);
                return;
            }
        }
        self.split.data(sym, sink);
    }
}

// access unit -> what the decoder gets
const STAMP: [u8; 6] = [0, 0, 0, 1, 0x60, 0x01];
/// Annex-B bytes for the decoder (the transmitter's timestamp NAL removed) and that timestamp (µs, TX clock).
pub fn prepare_au(au: &[u8]) -> Option<(Vec<u8>, Option<u64>)> {
    let mut au = au.to_vec();
    let mut sent_us = None;
    if au.len() >= STAMP.len() + 8 {
        let mut j = au.len() - STAMP.len() - 8;
        loop {
            if au[j..j + STAMP.len()] == STAMP {
                let mut b = [0u8; 8];
                b.copy_from_slice(&au[j + STAMP.len()..j + STAMP.len() + 8]);
                sent_us = Some(u64::from_le_bytes(b));
                au.drain(j..j + STAMP.len() + 8);
                break;
            }
            if j == 0 {
                break;
            }
            j -= 1;
        }
    }
    let i = (0..au.len().saturating_sub(2)).find(|&k| au[k] == 0 && au[k + 1] == 0 && au[k + 2] == 1)?;
    let mut end = au.len();
    while end > i && au[end - 1] == 0 {
        end -= 1;
    }
    let mut data = Vec::with_capacity(1 + end - i);
    data.push(0);
    data.extend_from_slice(&au[i..end]);
    Some((data, sent_us))
}

// receiver serial lines (radio_firmware/src/rx.c) -> stream + statistics
fn hex(s: &str) -> Vec<u8> {
    (0..s.len() / 2)
        .map(|i| u8::from_str_radix(&s[2 * i..2 * i + 2], 16).unwrap_or(0))
        .collect()
}

/// A received packet's radio measurements (the receiver's P / E line).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Radio {
    pub ch: u32,
    pub rssi: f32,
    pub rssi_sig: f32,
    pub snr: f32,
    pub foff_hz: f32,
    pub timing_us: f32,
    pub noise: f32,
}

/// What one line was, for statistics.
#[derive(Debug, Clone, PartialEq)]
pub enum Line {
    /// a packet: ok (CRC good, or repaired), filler (the transmitter's idle packets), repaired (by Reed-Solomon),
    /// crc_failed (the radio's CRC failed: an E line), video (a video entry went to the stream)
    /// air: the Air status an idle packet carries; foreign: the link ID of another system's packet (received intact,
    /// keyed with another link: dropped)
    Packet {
        ok: bool,
        filler: bool,
        repaired: bool,
        crc_failed: bool,
        video: bool,
        radio: Radio,
        air: Option<AirStatus>,
        foreign: Option<u16>,
    },
    /// a slot without a packet; noise = channel RSSI measured in it (dBm)
    Miss {
        noise: Option<f32>,
    },
    /// the receiver's configuration (once a second)
    /// hop: (channels, spacing in Hz), one packet per channel
    Cfg {
        freq_hz: u64,
        pwr_dbm: f32,
        locked: bool,
        hop: Option<(u32, u64)>,
    },
    /// "W": who the receiver is (key=value pairs)
    Who(Vec<(String, String)>),
    /// "K": the pairing profile it keeps (the raw line)
    Profile(String),
    /// the transmitter's identification, complete
    Ident(String),
    /// "S": the receiver's status counters (the raw text after "S ")
    Status(String),
    Other,
}

/// Parses one receiver line and drives the stream. `idents` collects the callsign's parts ("I" lines).
pub fn line(text: &str, stream: &mut Stream, sink: &mut dyn StreamSink, idents: &mut IdentAssembler) -> Line {
    let b = text.as_bytes();
    if b.len() < 2 || b[1] != b' ' {
        return Line::Other;
    }
    let c = b[0];
    if c == b'P' || c == b'E' {
        let p: Vec<&str> = text.split(' ').collect();
        if p.len() != 11 {
            return Line::Other;
        }
        let payload = hex(p[10]);
        if payload.len() != PAYLOAD {
            return Line::Other;
        }
        let num = |i: usize| p[i].parse::<f32>().unwrap_or(0.0);
        let radio = Radio {
            ch: p[2].parse().unwrap_or(0),
            rssi: num(4) / 2.0,
            rssi_sig: num(5) / 2.0,
            snr: num(6) / 4.0,
            foff_hz: num(7),
            timing_us: num(8),
            noise: num(9) / 2.0,
        };
        let raw = &payload[2..];
        let crc_failed = c == b'E';
        if c == b'P' && is_filler(raw) {
            let air = air_status(raw).filter(|_| {
                let l = u16::from_le_bytes([raw[21], raw[22]]);
                l == 0 || l == stream.link // (0: firmware without the link in it)
            });
            return Line::Packet {
                ok: true,
                filler: true,
                repaired: false,
                crc_failed,
                video: false,
                radio,
                air,
                foreign: None,
            };
        }
        let (e, repaired) = if c == b'P' && check_entry(raw, stream.link) {
            (Some(raw.to_vec()), false)
        } else {
            let r = repair_entry(raw, stream.link);
            let ok = r.is_some();
            (r, ok)
        };
        let Some(e) = e else {
            // radio CRC passed, RS finds no error, keyed CRC fails: another system's intact packet
            let foreign = (c == b'P' && rs_decode(raw, &[]).is_some_and(|o| o.as_slice() == raw))
                .then(|| crc16(&raw[..CRC_AT]) ^ (raw[CRC_AT] as u16 | (raw[CRC_AT + 1] as u16) << 8));
            return Line::Packet {
                ok: false,
                filler: false,
                repaired: false,
                crc_failed,
                video: false,
                radio,
                air: None,
                foreign,
            };
        };
        let filler = e[0] == FILLER;
        let video = e[0] == 0;
        if !filler {
            stream.entry(&e, sink);
        }
        return Line::Packet {
            ok: true,
            filler,
            repaired,
            crc_failed,
            video,
            radio,
            air: None,
            foreign: None,
        };
    }
    if c == b'M' {
        // "M seq ch reason noise_half_dbm"
        let p: Vec<&str> = text.split(' ').collect();
        let noise = p
            .get(4)
            .and_then(|v| v.parse::<f32>().ok())
            .filter(|v| *v != 9999.0)
            .map(|v| v / 2.0);
        return Line::Miss { noise };
    }
    if let Some(rest) = text.strip_prefix("C cfg ") {
        let mut freq_hz = 0;
        let mut pwr = 0.0;
        let mut locked = false;
        let mut hop = None;
        for kv in rest.split_whitespace() {
            match kv.split_once('=') {
                Some(("freq_hz", v)) => freq_hz = v.parse().unwrap_or(0),
                Some(("pwr", v)) => pwr = v.parse::<f32>().unwrap_or(0.0) / 2.0,
                Some(("locked", v)) => locked = v == "1",
                Some(("link", v)) => stream.link = v.parse().unwrap_or(stream.link),
                Some(("hop", v)) => {
                    let n: Vec<u64> = v.split(',').filter_map(|x| x.parse().ok()).collect();
                    if n.len() >= 2 && n[0] > 0 {
                        hop = Some((n[0] as u32, n[1])); // (older firmware adds slots per channel: ignored)
                    }
                }
                _ => {}
            }
        }
        return Line::Cfg {
            freq_hz,
            pwr_dbm: pwr,
            locked,
            hop,
        };
    }
    if c == b'W' {
        let kv = text[2..]
            .split_whitespace()
            .filter_map(|x| x.split_once('=').map(|(k, v)| (k.to_string(), v.to_string())))
            .collect();
        return Line::Who(kv);
    }
    if c == b'K' && (text.starts_with("K profile ") || text.starts_with("K saved ")) {
        return Line::Profile(text.to_string());
    }
    if c == b'I' {
        // "I seq part parts hex"
        let p: Vec<&str> = text.split(' ').collect();
        if p.len() >= 4 {
            let num = |i: usize| p[i].parse::<u32>().unwrap_or(0);
            if let Some(t) = idents.add(num(1), num(2), num(3), &hex(p.get(4).copied().unwrap_or(""))) {
                return Line::Ident(t);
            }
        }
        return Line::Other;
    }
    if c == b'S' {
        return Line::Status(text[2..].to_string());
    }
    Line::Other
}

// callsign blob, as format/packet.py encode_call / decode_call
const CALL_ALPHA: &str = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789/- ";
pub const CALL_TEXT_MAX: usize = 32;

/// The callsign as sent over the air (the receiver's "I <hex>" command).
pub fn encode_call(text: &str) -> Result<Vec<u8>, String> {
    if text.is_empty() {
        return Err("empty callsign".into());
    }
    if text.len() <= 127 && text.chars().all(|c| CALL_ALPHA.contains(c)) {
        let mut out = vec![0x80 | text.len() as u8];
        let mut codes: Vec<u32> = text.chars().map(|c| CALL_ALPHA.find(c).unwrap() as u32 + 1).collect();
        while codes.len() % 3 != 0 {
            codes.push(0);
        }
        for w in codes.chunks(3) {
            let v = w[0] * 1600 + w[1] * 40 + w[2];
            out.push((v & 0xff) as u8);
            out.push((v >> 8) as u8);
        }
        return Ok(out);
    }
    let raw = text.as_bytes();
    if raw.len() > 111 {
        return Err("callsign too long".into());
    }
    let mut out = vec![raw.len() as u8];
    out.extend_from_slice(raw);
    Ok(out)
}

fn decode_call(blob: &[u8]) -> Option<String> {
    let first = *blob.first()?;
    let n = (first & 0x7f) as usize;
    let len = if first & 0x80 != 0 {
        1 + 2 * ((n + 2) / 3)
    } else {
        1 + n
    };
    if len > blob.len() {
        return None;
    }
    if first & 0x80 == 0 {
        return Some(String::from_utf8_lossy(&blob[1..1 + n]).into_owned());
    }
    let alpha: Vec<char> = CALL_ALPHA.chars().collect();
    let mut out = String::new();
    let mut i = 1;
    while i < len {
        let w = blob[i] as u32 | (blob[i + 1] as u32) << 8;
        for c in [w / 1600, (w / 40) % 40, w % 40] {
            if c > 0 {
                out.push(if (c as usize) <= alpha.len() {
                    alpha[c as usize - 1]
                } else {
                    '?'
                });
            }
        }
        i += 2;
    }
    out.truncate(n);
    Some(out)
}

/// "I seq part parts hex" lines -> the callsign once all parts are in.
#[derive(Default)]
pub struct IdentAssembler {
    parts: std::collections::BTreeMap<u32, Vec<u8>>,
    n: Option<u32>,
    last_seq: Option<u32>,
}
impl IdentAssembler {
    pub fn add(&mut self, seq: u32, part: u32, parts: u32, data: &[u8]) -> Option<String> {
        let stale = self
            .last_seq
            .is_some_and(|l| (seq.wrapping_sub(l) & 0xffff) > 64 * (parts + 1));
        if self.n != Some(parts) || stale || part == 0 {
            self.parts.clear();
        }
        self.n = Some(parts);
        self.last_seq = Some(seq);
        self.parts.insert(part, data.to_vec());
        if self.parts.len() as u32 != parts {
            return None;
        }
        let blob: Vec<u8> = (0..parts)
            .flat_map(|i| self.parts.get(&i).cloned().unwrap_or_default())
            .collect();
        self.parts.clear();
        decode_call(&blob)
    }
}

#[cfg(test)]
#[path = "../test/engine.rs"]
mod tests;
