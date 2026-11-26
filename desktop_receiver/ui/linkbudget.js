// Link budget: a planning calculator. It only calculates: nothing here
// is sent to the Ground or the Air. Inputs are kept in this window (localStorage).
//   time on air     Semtech LoRa formula (SX126x/LR11xx/LR20xx; SF5/6 preamble + 6.25 symbols). Long interleaver:
//                   payload symbols = ceil(bits (4 + CR) / 4 / SF): 17.03 ms for SF5 / 1000 kHz / LI 4/5 / 255 B.
//   sensitivity     -174 dBm/Hz + 10 log10(BW) + noise figure + noise rise + required SNR(SF) + implementation loss
//   free space      20 log10(4 pi d / lambda)
//   two-ray ground  free space up to 4 pi ht hr / lambda, then 40 log10(d) - 20 log10(ht hr)
//   log-distance    free space at 1 m + 10 n log10(d)
//   radio horizon   4.12 (sqrt(ht) + sqrt(hr)) km (4/3 earth radius), caps the line-of-sight ranges
'use strict';
const LB = (() => {
  const C0 = 299792458;
  const SNR_REQ = { 5: -2.5, 6: -5, 7: -7.5, 8: -10, 9: -12.5, 10: -15, 11: -17.5, 12: -20 };
  const BWS = [62.5, 125, 203.125, 250, 406.25, 500, 812.5, 1000];
  const CRS = [['4/5', 1, false], ['4/6', 2, false], ['4/7', 3, false], ['4/8', 4, false],
    ['4/5, long interleaver', 1, true], ['4/6, long interleaver', 2, true], ['4/8, long interleaver', 4, true]];
  const MODELS = [['Free space', 'fs', 2], ['Two-ray ground', 'tworay', 2], ['Log-distance', 'logd', 3]];
  const SCEN = [
    ['Clear line of sight', 'fs', 2, true, 'Air to ground, hilltop to hilltop'],
    ['Line of sight over flat ground', 'tworay', 2, true, 'both ends low, open field or water'],
    ['Rural, light obstruction', 'logd', 2.7, false, 'scattered trees, low buildings (n = 2.7)'],
    ['Suburban', 'logd', 3.0, false, 'houses and trees in the path (n = 3.0)'],
    ['Urban', 'logd', 3.5, false, 'streets, no line of sight (n = 3.5)'],
    ['Dense urban, indoors', 'logd', 4.0, false, 'through buildings and walls (n = 4.0)'],
  ];
  // hz_vtx defaults: 231 bytes of video per 255-byte packet, block FEC 8+4, 576x320 at 30 fps.
  const DEFAULTS = {
    freq_mhz: 868, sf: 5, bw_khz: 1000, cr: 4, preamble: 12, payload: 255, overhead: 24, crc: true, explicit: true,
    guard_ms: 1.67, fec_k: 8, fec_m: 4, util: 1.0,
    width: 576, height: 320, fps: 30, bpp_good: 0.06,
    tx_dbm: 22, pa_db: 0, tx_cable: 0.5, tx_ant: 2, rx_ant: 2, rx_cable: 0.5, nf_db: 7, impl_db: 1, noise_rise: 0,
    fade_db: 10, extra_loss: 0, ht_m: 50, hr_m: 1.5, model: 0, exponent: 3, dist_km: 1,
  };

  function airtime(sf, bw, parity, li, preamble, payload, crc, explicit, ldro) {
    const tsym = 2 ** sf / bw;
    const pre = preamble + (sf <= 6 ? 6.25 : 4.25);
    const bits = Math.max(0, 8 * payload + (crc ? 16 : 0) - 4 * sf + (sf <= 6 ? 0 : 8) + (explicit ? 20 : 0));
    const sfe = ldro ? sf - 2 : sf;
    const n = li ? Math.ceil(bits * (4 + parity) / 4 / sfe) : 8 + Math.ceil(bits / (4 * sfe)) * (parity + 4);
    return [(pre + n) * tsym, tsym, pre + n];
  }
  const fspl = (d, f) => 20 * Math.log10(4 * Math.PI * Math.max(d, 1e-3) * f / C0);
  function pathLoss(d, f, kind, n, ht, hr) {
    d = Math.max(d, 0.01);
    if (kind === 'fs') return fspl(d, f);
    if (kind === 'tworay') {
      const dc = 4 * Math.PI * ht * hr / (C0 / f);
      return d <= dc ? fspl(d, f) : 40 * Math.log10(d) - 20 * Math.log10(ht * hr);
    }
    return fspl(1, f) + 10 * n * Math.log10(Math.max(d, 1));
  }
  function range(maxLoss, f, kind, n, ht, hr) {
    let lo = 0.01, hi = 1e9;
    if (pathLoss(lo, f, kind, n, ht, hr) > maxLoss) return 0;
    if (pathLoss(hi, f, kind, n, ht, hr) <= maxLoss) return hi;
    for (let i = 0; i < 200; i++) { const m = Math.sqrt(lo * hi); if (pathLoss(m, f, kind, n, ht, hr) <= maxLoss) lo = m; else hi = m; }
    return lo;
  }
  const horizon = (ht, hr) => 4120 * (Math.sqrt(Math.max(ht, 0)) + Math.sqrt(Math.max(hr, 0)));
  function quality(bpp, good) {
    const r = good > 0 ? bpp / good : 0;
    if (r >= 1) return ['good', 'holds up in motion', 'ok'];
    if (r >= 0.6) return ['usable', 'clean when still, soft in motion', 'ok'];
    if (r >= 0.3) return ['poor', 'blocky whenever the scene moves', 'warn'];
    return ['very poor', 'blocky most of the time', 'bad'];
  }

  function calculate(p) {
    const r = {}, f = p.freq_mhz * 1e6, bw = p.bw_khz * 1e3, sf = +p.sf;
    const [crl, parity, li] = CRS[+p.cr];
    const tsym0 = 2 ** sf / bw, ldro = tsym0 >= 0.016;
    const [toa, tsym, nsym] = airtime(sf, bw, parity, li, p.preamble, p.payload, p.crc, p.explicit, ldro);
    Object.assign(r, { toa_ms: toa * 1e3, tsym_ms: tsym * 1e3, nsym, ldro, cr_label: crl });
    r.phy_bps = sf * bw / 2 ** sf * 4 / (4 + parity);
    const slot = toa + p.guard_ms / 1e3;
    r.slot_ms = slot * 1e3; r.pkt_rate = 1 / slot; r.duty = toa / slot;
    r.video_bytes = Math.max(0, p.payload - p.overhead);
    r.payload_bps = r.video_bytes * 8 / slot;
    const k = Math.max(1, p.fec_k), m = Math.max(0, p.fec_m);
    r.fec_rate = k / (k + m);
    r.capacity_bps = r.payload_bps * r.fec_rate;
    r.video_bps = r.capacity_bps * p.util;
    r.loss_tolerated = m / (k + m);
    const px = p.width * p.height;
    r.frame_bytes = p.fps ? r.video_bps / 8 / p.fps : 0;
    r.bpp = px && p.fps ? r.video_bps / (px * p.fps) : 0;
    [r.quality, r.quality_note, r.quality_cls] = quality(r.bpp, p.bpp_good);
    r.packets_per_frame = r.video_bytes ? r.frame_bytes / r.video_bytes : 0;
    const pix = p.fps && p.bpp_good ? r.video_bps / (p.fps * p.bpp_good) : 0;
    const gh = Math.floor(Math.sqrt(pix * 9 / 16) / 2) * 2;
    r.good_res = [Math.floor(gh * 16 / 9 / 2) * 2, gh];

    r.eirp_dbm = p.tx_dbm + p.pa_db - p.tx_cable + p.tx_ant;
    r.noise_floor_dbm = -174 + 10 * Math.log10(bw) + p.nf_db + p.noise_rise;
    r.snr_req_db = SNR_REQ[sf];
    r.sens_dbm = r.noise_floor_dbm + SNR_REQ[sf] + p.impl_db;
    r.rx_gain = p.rx_ant - p.rx_cable;
    r.max_loss_db = r.eirp_dbm + r.rx_gain - r.sens_dbm;
    r.usable_loss_db = r.max_loss_db - p.fade_db - p.extra_loss;
    const hor = horizon(p.ht_m, p.hr_m);
    r.horizon_m = hor;
    r.theory_m = range(r.max_loss_db, f, 'fs', 2, p.ht_m, p.hr_m);
    r.scenarios = SCEN.map(([name, kind, n, los, note]) => {
      const d = range(r.usable_loss_db, f, kind, n, p.ht_m, p.hr_m);
      return { name, note, d: los ? Math.min(d, hor) : d, capped: los && d > hor };
    });
    const [, mk] = MODELS[+p.model];
    r.sel_kind = mk;
    const dsel = range(r.usable_loss_db, f, mk, p.exponent, p.ht_m, p.hr_m);
    r.sel_capped = mk !== 'logd' && dsel > hor;
    r.sel_range_m = mk !== 'logd' ? Math.min(dsel, hor) : dsel;
    const d = p.dist_km * 1e3;
    r.d_loss_db = pathLoss(d, f, mk, p.exponent, p.ht_m, p.hr_m) + p.extra_loss;
    r.d_rx_dbm = r.eirp_dbm + r.rx_gain - r.d_loss_db;
    r.d_snr_db = r.d_rx_dbm - r.noise_floor_dbm;
    r.d_margin_db = r.d_rx_dbm - r.sens_dbm;
    r.d_fresnel_m = 0.5 * Math.sqrt(C0 / f * d);
    r.d_beyond_horizon = d > hor;
    r.rxAt = (dm) => r.eirp_dbm + r.rx_gain - pathLoss(dm, f, mk, p.exponent, p.ht_m, p.hr_m) - p.extra_loss;
    r.rxFs = (dm) => r.eirp_dbm + r.rx_gain - fspl(dm, f);
    return r;
  }
  function dist(m) {
    if (m >= 1e8) return 'unlimited';
    if (m >= 10000) return (m / 1000).toFixed(0) + ' km';
    if (m >= 1000) return (m / 1000).toFixed(1) + ' km';
    if (m >= 100) return m.toFixed(0) + ' m';
    return m.toFixed(1) + ' m';
  }
  return { DEFAULTS, BWS, CRS, MODELS, calculate, dist };
})();
