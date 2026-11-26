//! Unit tests of src/engine.rs (the receiver engine: callsign blobs, Reed-Solomon repair, the access unit stream): compiled into that module by `cargo test`.
use super::*;
#[test]
fn callsign_round_trip() {
    for call in ["S59ABC/P", "N0CALL", "AB1"] {
        let blob = encode_call(call).unwrap();
        let mut ia = IdentAssembler::default();
        let (a, b) = blob.split_at(blob.len() / 2);
        assert_eq!(ia.add(10, 0, 2, a), None);
        assert_eq!(ia.add(11, 1, 2, b).as_deref(), Some(call));
    }
}

#[test]
fn rs_repairs_eight_errors() {
    // a valid entry: CRC, then parity from a systematic encoder (as check_swfec.mjs builds them)
    let mut g = vec![1u8];
    for i in 0..RS_PAR {
        g = poly_mul(&g, &[1, GF.exp[i]]);
    }
    let mut e = vec![0u8; ENTRY];
    for (i, v) in e.iter_mut().enumerate().take(HDR + SYM).skip(HDR) {
        *v = (i * 37 % 251) as u8;
    }
    let c = crc16(&e[..CRC_AT]);
    e[CRC_AT] = c as u8;
    e[CRC_AT + 1] = (c >> 8) as u8;
    let mut r = vec![0u8; RS_PAR];
    for &b in &e[..CRC_AT + 2] {
        let f = b ^ r[0];
        r.remove(0);
        r.push(0);
        for j in 0..RS_PAR {
            r[j] ^= gmul(g[j + 1], f);
        }
    }
    e[CRC_AT + 2..].copy_from_slice(&r);
    assert!(check_entry(&e, 0));
    let mut d = e.clone();
    for k in 0..8 {
        d[k * 29 + 3] ^= 0x5a;
    }
    assert_eq!(repair_entry(&d, 0).unwrap(), e);
}

// block FEC
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn below(&mut self, n: usize) -> usize {
        (self.next() % n as u64) as usize
    }
    fn bytes(&mut self, n: usize) -> Vec<u8> {
        (0..n).map(|_| self.next() as u8).collect()
    }
    /// n distinct values below `of`
    fn pick(&mut self, n: usize, of: usize) -> Vec<usize> {
        let mut v: Vec<usize> = (0..of).collect();
        for i in 0..n {
            let j = i + self.below(of - i);
            v.swap(i, j);
        }
        v.truncate(n);
        v
    }
}

#[test]
fn block_fec_round_trip_with_erasures() {
    let mut rng = Rng(0x9e3779b97f4a7c15);
    for &(k, m) in &[(16, 2), (12, 3), (8, 2), (1, 1), (16, 16), (5, 4), (3, 16)] {
        for _ in 0..30 {
            let src: Vec<Vec<u8>> = (0..k).map(|_| rng.bytes(SYM)).collect();
            let refs: Vec<&[u8]> = src.iter().map(|v| v.as_slice()).collect();
            let par = bfec_encode(&refs, m);
            // lose up to m of the k + m packets, any mix of video and parity
            let nlose = 1 + rng.below(m);
            let lose = rng.pick(nlose, k + m);
            let have: Vec<(usize, &[u8])> = (0..k)
                .filter(|i| !lose.contains(i))
                .map(|i| (i, src[i].as_slice()))
                .collect();
            let pk: Vec<(usize, &[u8])> = (0..m)
                .filter(|j| !lose.contains(&(k + j)))
                .map(|j| (j, par[j].as_slice()))
                .collect();
            let got = bfec_decode(k, &have, &pk).expect("decodable");
            let want: Vec<usize> = (0..k).filter(|i| lose.contains(i)).collect();
            assert_eq!(got.iter().map(|g| g.0).collect::<Vec<_>>(), want);
            for (i, v) in got {
                assert_eq!(v, src[i], "k={k} m={m} lost {lose:?}");
            }
        }
        // one more loss than parity: not decodable
        let src: Vec<Vec<u8>> = (0..k).map(|_| rng.bytes(SYM)).collect();
        let refs: Vec<&[u8]> = src.iter().map(|v| v.as_slice()).collect();
        let par = bfec_encode(&refs, m);
        if m < k {
            let have: Vec<(usize, &[u8])> = (m + 1..k).map(|i| (i, src[i].as_slice())).collect();
            let pk: Vec<(usize, &[u8])> = (0..m).map(|j| (j, par[j].as_slice())).collect();
            assert!(bfec_decode(k, &have, &pk).is_none());
        }
    }
    // C[j][i] = 1 / ((k + j) xor i); k = 2, m = 1, sources [1], [2]: parity = ginv(2) ^ gmul(ginv(3), 2)
    let p = bfec_encode(&[&[1u8][..], &[2u8][..]], 1);
    assert_eq!(p[0], vec![ginv(2) ^ gmul(ginv(3), 2)]);
}

fn entry_of(typ: u8, id: u32, aux: u8, sym: &[u8]) -> Vec<u8> {
    let mut e = vec![0u8; ENTRY];
    e[0] = typ;
    e[1] = id as u8;
    e[2] = (id >> 8) as u8;
    e[3] = aux;
    e[HDR..HDR + sym.len()].copy_from_slice(sym);
    let c = crc16(&e[..CRC_AT]);
    e[CRC_AT] = c as u8;
    e[CRC_AT + 1] = (c >> 8) as u8;
    e
}

/// Transmitter as scheduler.h: the byte stream in 231-byte symbols; groups of `sizes[g]` video packets
/// (k * d, or fewer: a group closed early), the parity of its d interleaved blocks after it.
fn tx(stream: &[u8], m: usize, d: usize, sizes: &mut dyn FnMut() -> usize) -> Vec<Vec<u8>> {
    let syms: Vec<Vec<u8>> = stream
        .chunks(SYM)
        .map(|c| {
            let mut v = c.to_vec();
            v.resize(SYM, 0);
            v
        })
        .collect();
    let mut out = Vec::new();
    let mut id = 0usize;
    while id < syms.len() {
        let n = sizes().min(syms.len() - id);
        for (i, s) in syms[id..id + n].iter().enumerate() {
            out.push(entry_of(0, (id + i) as u32, 0, s));
        }
        for b in 0..d.min(n) {
            let blk: Vec<&[u8]> = (b..n).step_by(d).map(|i| syms[id + i].as_slice()).collect();
            for (j, p) in bfec_encode(&blk, m).iter().enumerate() {
                out.push(entry_of(
                    PARITY | ((d - 1) << 4) as u8,
                    (id + b) as u32,
                    ((blk.len() - 1) << 4 | j) as u8,
                    p,
                ));
            }
        }
        id += n;
    }
    out
}

#[derive(Default)]
struct Got {
    aus: Vec<Vec<u8>>,
    gaps: usize,
    lost: u64,
    recovered: u64,
}
impl StreamSink for Got {
    fn au(&mut self, au: Vec<u8>, after_gap: bool) {
        self.gaps += after_gap as usize;
        self.aus.push(au);
    }
    fn source(&mut self, lost: bool) {
        self.lost += lost as u64;
    }
    fn event(&mut self, _: String) {}
    fn recovered(&mut self) {
        self.recovered += 1;
    }
}

fn make_aus(n: usize, rng: &mut Rng) -> Vec<Vec<u8>> {
    (0..n)
        .map(|i| {
            let mut a = vec![0, 0, 0, 1, 2, 1, 1 + (i / 255 % 255) as u8, 1 + (i % 255) as u8];
            let len = 60 + rng.below(640);
            a.extend((0..len).map(|_| 1 + rng.below(255) as u8)); // no zeros: no false start codes
            a
        })
        .collect()
}

fn run(entries: &[Vec<u8>], drop: &mut dyn FnMut(usize, &[u8]) -> bool) -> (Got, u64) {
    let mut st = Stream::default();
    let mut g = Got::default();
    for (n, e) in entries.iter().enumerate() {
        if !drop(n, e) {
            st.entry(e, &mut g);
        }
    }
    st.entry(&entry_of(FILLER, 0xffff, 0xff, &[]), &mut g);
    (g, st.recovered)
}

#[test]
fn block_fec_stream_recovers_losses() {
    let mut rng = Rng(12345);
    let aus = make_aus(400, &mut rng);
    let mut stream = Vec::new();
    for a in &aus {
        stream.extend_from_slice(a);
        stream.extend_from_slice(&AUD);
    }
    // the AUD that ends the last AU is in the stream: every AU comes out
    let ents = tx(&stream, 2, 1, &mut || 16);
    let (g, rec) = run(&ents, &mut |_, _| false);
    assert_eq!(g.aus, aus);
    assert_eq!((g.lost, rec), (0, 0));

    // groups closed early (random sizes 1..16) and two losses in every block after the first two (until the first
    // parity arrives the receiver cannot know FEC is on)
    let mut r2 = Rng(77);
    let ents = tx(&stream, 2, 1, &mut || 1 + r2.below(16));
    let mut r3 = Rng(99);
    let mut block_of = Vec::new(); // block number per entry
    let mut b = 0usize;
    for (n, e) in ents.iter().enumerate() {
        if n > 0 && e[0] == 0 && ents[n - 1][0] != 0 {
            b += 1;
        }
        block_of.push(b);
    }
    let mut lost_in: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for (n, &blk) in block_of.iter().enumerate() {
        lost_in.entry(blk).or_default().push(n);
    }
    let mut drop_set = BTreeSet::new();
    for (blk, idx) in &lost_in {
        if *blk >= 2 && idx.len() >= 3 {
            for p in r3.pick(2, idx.len()) {
                drop_set.insert(idx[p]);
            }
        }
    }
    let (g, rec) = run(&ents, &mut |n, _| drop_set.contains(&n));
    assert_eq!(g.aus, aus, "two losses per block, early-closed blocks");
    assert_eq!(g.gaps, 1, "only the very first AU is after a gap");
    assert_eq!(g.lost, 0);
    assert!(rec > 50, "rebuilt {rec}");
    assert_eq!(g.recovered, rec);
    // the same with ids that wrap past 65535 (the receiver starts inside the stream: the first AU is not complete)
    let wrapped: Vec<Vec<u8>> = ents
        .iter()
        .map(|e| {
            let id = (e[1] as u32 | (e[2] as u32) << 8) + 65000;
            let mut w = e.clone();
            w[1] = id as u8;
            w[2] = (id >> 8) as u8;
            w
        })
        .collect();
    let (g, rec2) = run(&wrapped, &mut |n, _| drop_set.contains(&n));
    assert_eq!(g.aus[..], aus[1..], "ids wrapping past 65535");
    assert_eq!(rec2, rec);

    // interleaved 8+2 x3: a fade of 6 packets in a row in every group (24 video + 6 parity) is rebuilt
    let ents = tx(&stream, 2, 3, &mut || 24);
    let mut r4 = Rng(5);
    let starts: Vec<usize> = (0..ents.len() / 30 + 1).map(|_| r4.below(19)).collect();
    let fade = |n: usize| n / 30 >= 2 && (starts[n / 30]..starts[n / 30] + 6).contains(&(n % 30));
    let (g, rec) = run(&ents, &mut |n, _| fade(n));
    assert_eq!(g.aus, aus, "interleaved fades");
    assert!(rec > 0 && g.lost == 0);

    // more losses than the code corrects: only the AUs touched are lost, none is corrupted, the stream goes on
    let ents = tx(&stream, 2, 1, &mut || 8);
    let mut r5 = Rng(31337);
    let (g, rec) = run(&ents, &mut |_, _| r5.below(10) == 0);
    assert!(g.aus.iter().all(|a| aus.contains(a)), "a corrupted AU was delivered");
    assert!(
        g.aus.len() > aus.len() * 8 / 10 && g.lost > 0 && rec > 0,
        "{} AUs, lost {}, rebuilt {rec}",
        g.aus.len(),
        g.lost
    );

    // without parity: a lost packet drops what it touched at once, nothing rebuilt
    let plain: Vec<Vec<u8>> = ents.iter().filter(|e| e[0] == 0).cloned().collect();
    let mut r6 = Rng(31337);
    let (g, rec) = run(&plain, &mut |_, _| r6.below(10) == 0);
    assert!(g.aus.iter().all(|a| aus.contains(a)) && rec == 0 && g.lost > 0);
}
