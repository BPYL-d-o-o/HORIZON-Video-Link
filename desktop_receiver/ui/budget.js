// Link budget page: inputs, results, the received-power chart and the range bars (the maths: linkbudget.js).
'use strict';
const LBF = [
  ['Path', true, [
    ['dist_km', 'Check a distance', 'km'], ['ht_m', 'Air unit antenna height', 'm'], ['hr_m', 'Ground antenna height', 'm'],
    ['fade_db', 'Fade margin', 'dB', 'Reserve for fading, pointing and polarisation: 10 dB is usual, 15-20 dB for a moving aircraft.'],
    ['extra_loss', 'Obstruction loss', 'dB', 'Fixed losses in the path: a wall 5-15 dB, foliage a few dB.'],
    ['model', 'Model for the graph', '', '', LB.MODELS.map((m) => m[0])],
    ['exponent', 'Log-distance exponent n', '', '2 free space, 3 suburban, 4 indoors (log-distance model only).'],
  ]],
  ['Transmitter and receiver', true, [
    ['tx_dbm', 'TX power', 'dBm', 'The radio chip: -9.5 to +22 dBm.'], ['pa_db', 'External amplifier', 'dB', '0 without one.'],
    ['tx_ant', 'Air unit antenna gain', 'dBi'], ['tx_cable', 'Air unit cable loss', 'dB'],
    ['rx_ant', 'Ground antenna gain', 'dBi'], ['rx_cable', 'Ground cable loss', 'dB'],
    ['nf_db', 'Receiver noise figure', 'dB', 'Whole receive chain; 6-8 dB is typical.'],
    ['impl_db', 'Implementation loss', 'dB', 'How far a real receiver falls short of the ideal demodulator.'],
    ['noise_rise', 'Noise above thermal', 'dB', 'Man-made noise on the channel: 0 in a quiet place, 5-15 dB in town.'],
  ]],
  ['Radio', false, [
    ['freq_mhz', 'Frequency', 'MHz'],
    ['sf', 'Spreading factor', '', 'Each step up halves the bitrate and gains 2.5 dB.', [5, 6, 7, 8, 9, 10, 11, 12].map((x) => 'SF' + x)],
    ['bw_khz', 'Bandwidth', 'kHz', 'Halving it halves the bitrate and gains 3 dB.', LB.BWS.map(String)],
    ['cr', 'Coding rate', '', 'This system uses 4/5 with the long interleaver.', LB.CRS.map((c) => c[0])],
    ['preamble', 'Preamble', 'symbols'], ['crc', 'Radio CRC', 'check'], ['explicit', 'Explicit header', 'check'],
  ]],
  ['Packets and video', false, [
    ['payload', 'Radio packet', 'B'],
    ['overhead', 'Overhead in a packet', 'B', 'Format v4: sequence 2 + header 4 + CRC 2 + Reed-Solomon 16 = 24 bytes.'],
    ['guard_ms', 'Gap after a packet', 'ms', 'Slot = time on air + this gap (18.7 ms slots today).'],
    ['fec_k', 'Block FEC: video packets', '', 'Video packets per block (only with parity packets).'],
    ['fec_m', 'Block FEC: parity packets', '', '0 = no block FEC (the transmitters send 4 per 8 video packets unless set otherwise).'],
    ['util', 'Video share of the link', '', '0 to 1.'],
    ['width', 'Width', 'px'], ['height', 'Height', 'px'], ['fps', 'Frame rate', 'fps'],
    ['bpp_good', 'Good picture needs', 'bit/px', 'Rule of thumb for low-rate HEVC.'],
  ]],
];
let lbP = null, lbR = null, lbHover = null;
function lbSave() { try { localStorage.setItem('linkbudget', JSON.stringify(lbP)); } catch (_) {} }
function lbLoad() {
  let saved = {};
  try { saved = JSON.parse(localStorage.getItem('linkbudget') || '{}') || {}; } catch (_) {}
  lbP = { ...LB.DEFAULTS, ...saved };
}
function lbBuildInputs() {
  const box = $('lb-inputs'); box.innerHTML = '';
  for (const [title, open, fields] of LBF) {
    const d = document.createElement('details'); d.className = 'card'; d.open = open;
    d.innerHTML = '<summary><h2></h2></summary><div class="fields"></div>';
    d.querySelector('h2').textContent = title;
    const grid = d.querySelector('.fields');
    for (const [key, label, unit, tip, options] of fields) {
      const l = document.createElement('label');
      if (tip) l.title = tip;
      let el;
      if (unit === 'check') {
        l.className = 'check'; el = document.createElement('input'); el.type = 'checkbox'; el.checked = !!lbP[key];
        l.append(el, document.createTextNode(label));
      } else {
        const head = document.createElement('span'); head.textContent = label + ' ';
        if (unit) { const u = document.createElement('span'); u.className = 'u'; u.textContent = unit; head.appendChild(u); }
        l.appendChild(head);
        if (options) {
          el = document.createElement('select');
          options.forEach((o, i) => {
            const op = document.createElement('option');
            op.value = key === 'sf' ? String(5 + i) : key === 'bw_khz' ? o : String(i);
            op.textContent = key === 'bw_khz' ? o + ' kHz' : o; el.appendChild(op);
          });
          el.value = String(lbP[key]);
        } else { el = document.createElement('input'); el.inputMode = 'decimal'; el.value = String(lbP[key]); }
        l.appendChild(el);
      }
      el.addEventListener(unit === 'check' || options ? 'change' : 'input', () => {
        const v = unit === 'check' ? el.checked : parseFloat(String(el.value).replace(',', '.').replace('−', '-'));
        if (unit !== 'check' && !isFinite(v)) return;
        lbP[key] = v; lbSave(); drawBudget();
      });
      grid.appendChild(l);
    }
    box.appendChild(d);
  }
}
$('lb-reset').addEventListener('click', () => {
  lbP = { ...LB.DEFAULTS };
  try { localStorage.removeItem('linkbudget'); } catch (_) {}
  lbBuildInputs(); drawBudget();
});
$('lb-current').addEventListener('click', () => {
  const c = st && st.ground.cfg;
  if (!c) { alert('No receiver: plug in the Ground to read its frequency and power.'); return; }
  lbP.freq_mhz = +(c.freq_hz / 1e6).toFixed(3); lbP.tx_dbm = c.pwr_dbm;
  lbSave(); lbBuildInputs(); drawBudget();
});
$('lb-copy').addEventListener('click', () => {
  const p = lbP, r = lbR;
  const lines = [
    'Horizon video link budget (planning estimate)',
    `Radio: ${p.freq_mhz} MHz, SF${p.sf}, ${p.bw_khz} kHz, CR ${r.cr_label}, ${p.payload} B packets: ${r.toa_ms.toFixed(2)} ms on air, ${r.slot_ms.toFixed(1)} ms slots`,
    `Video: ${(r.video_bps / 1e3).toFixed(1)} kbit/s, ${p.width}x${p.height} @ ${p.fps} fps, ${r.bpp.toFixed(3)} bit/px: ${r.quality}`,
    `EIRP ${r.eirp_dbm.toFixed(1)} dBm, sensitivity ${r.sens_dbm.toFixed(1)} dBm, max path loss ${r.max_loss_db.toFixed(1)} dB (${r.usable_loss_db.toFixed(1)} dB after the ${p.fade_db} dB margin)`,
    `Radio horizon ${LB.dist(r.horizon_m)} (antennas at ${p.ht_m} m and ${p.hr_m} m)`,
    ...r.scenarios.map((x) => `  ${x.name.padEnd(32)} ${LB.dist(x.d)}${x.capped ? ' (radio horizon)' : ''}`),
    `At ${p.dist_km} km: received ${r.d_rx_dbm.toFixed(1)} dBm, SNR ${r.d_snr_db.toFixed(1)} dB, margin ${r.d_margin_db.toFixed(1)} dB`,
  ];
  navigator.clipboard.writeText(lines.join('\n')).then(() => {
    $('lb-copy').textContent = 'Copied ✓';
    setTimeout(() => { $('lb-copy').textContent = 'Copy results'; }, 1500);
  });
});
const cvB = $('lb-canvas');
cvB.addEventListener('mousemove', (e) => { lbHover = cvB._x ? cvB._x.inv(e.offsetX) : null; drawBudgetChart(); });
cvB.addEventListener('mouseleave', () => { lbHover = null; drawBudgetChart(); });

function drawBudget() {
  if ($('page-budget').hidden) return;
  if (!lbP) { lbLoad(); lbBuildInputs(); }
  const p = lbP, r = lbR = LB.calculate(p);
  const mc = r.d_margin_db >= p.fade_db ? 'ok' : r.d_margin_db >= 0 ? 'warn' : 'bad';
  const los = r.scenarios[0];
  const tiles = [
    ['RANGE, LINE OF SIGHT', LB.dist(los.d), '', los.capped ? 'limited by the radio horizon' : `with ${p.fade_db} dB fade margin`],
    ['VIDEO BITRATE', `${(r.video_bps / 1e3).toFixed(0)} kbit/s`, '', `${r.slot_ms.toFixed(1)} ms slots`],
    ['PICTURE', r.quality, r.quality_cls, `${p.width}×${p.height} at ${p.fps} fps`],
    [`MARGIN AT ${p.dist_km} KM`, `${fmtNum(r.d_margin_db, 1)} dB`, mc, mc === 'ok' ? 'the link closes' : mc === 'warn' ? 'less than the fade margin' : 'no link'],
  ];
  $('lb-tiles').innerHTML = tiles.map(([k, v, c, sub]) =>
    `<div class="tile"><span class="k">${k}</span><span class="v ${c}">${v}</span><span class="small faint">${sub}</span></div>`).join('');
  // practical range: bars on a log scale from 10 m, striped where the radio horizon limits them
  const finite = r.scenarios.map((x) => x.d).filter((d) => d < 1e8);
  const hi = Math.max(...finite, 1000) * 1.15, lo = 10;
  const frac = (d) => Math.max(0.01, Math.min(1, Math.log10(Math.max(d, lo) / lo) / Math.log10(hi / lo)));
  $('lb-range-sub').textContent = `with ${p.fade_db} dB fade margin${p.extra_loss ? ` and ${p.extra_loss} dB obstruction` : ''} · log scale · striped: stopped by the radio horizon (${LB.dist(r.horizon_m)})`;
  $('lb-ranges').innerHTML = r.scenarios.map((x) =>
    `<div class="rg"><div class="name">${x.name}<span>${x.note}</span></div>` +
    `<div class="track"><div class="bar${x.capped ? ' capped' : ''}" style="width:${(frac(x.d) * 100).toFixed(1)}%"></div></div>` +
    `<div class="val">${LB.dist(x.d)}</div></div>`).join('');
  const row = (a, b, c = '', cls = '') => `<tr><td>${a}</td><td class="${cls}">${b}</td><td>${c}</td></tr>`;
  $('lb-tables').innerHTML = `
    <section class="card"><h2>At ${p.dist_km} km</h2><table class="kv">
      ${row('Received power', `${fmtNum(r.d_rx_dbm, 1)} dBm`, `path loss ${r.d_loss_db.toFixed(1)} dB, ${LB.MODELS[p.model][0].toLowerCase()}`)}
      ${row('SNR', `${fmtNum(r.d_snr_db, 1)} dB`, `needs ${fmtNum(r.snr_req_db + p.impl_db, 1)} dB at SF${p.sf}`)}
      ${row('Margin over sensitivity', `${fmtNum(r.d_margin_db, 1)} dB`, '', mc + '-t')}
      ${row('Fresnel zone, mid-path', `${r.d_fresnel_m.toFixed(1)} m`, `keep ${(0.6 * r.d_fresnel_m).toFixed(1)} m clear${r.d_beyond_horizon ? '; beyond the radio horizon' : ''}`)}
    </table></section>
    <section class="card"><h2>Budget</h2><table class="kv">
      ${row('Radiated power (EIRP)', `${fmtNum(r.eirp_dbm, 1)} dBm`, `${(10 ** (r.eirp_dbm / 10)).toPrecision(2)} mW`)}
      ${row('Noise floor', `${fmtNum(r.noise_floor_dbm, 1)} dBm`, `in ${p.bw_khz} kHz`)}
      ${row('Sensitivity', `${fmtNum(r.sens_dbm, 1)} dBm`)}
      ${row('Maximum path loss', `${r.max_loss_db.toFixed(1)} dB`, `${r.usable_loss_db.toFixed(1)} dB usable`)}
      ${row('Theoretical range', LB.dist(r.theory_m), 'free space, no margin, no horizon')}
    </table></section>
    <section class="card"><h2>Packets and bitrate</h2><table class="kv">
      ${row('Time on air', `${r.toa_ms.toFixed(2)} ms`, `${r.nsym.toFixed(2)} symbols of ${r.tsym_ms.toFixed(3)} ms${r.ldro ? ', LDRO' : ''}`)}
      ${row('Slot', `${r.slot_ms.toFixed(2)} ms`, `${r.pkt_rate.toFixed(1)} packets/s, on air ${(r.duty * 100).toFixed(0)} %`)}
      ${row('LoRa bitrate', `${(r.phy_bps / 1e3).toFixed(1)} kbit/s`, 'modulation only')}
      ${row('Video in a packet', `${r.video_bytes} B`, `${(r.payload_bps / 1e3).toFixed(1)} kbit/s`)}
      ${row('Block FEC', p.fec_m > 0 ? `${p.fec_k} + ${p.fec_m}` : 'none', p.fec_m > 0 ? `survives ${(r.loss_tolerated * 100).toFixed(0)} % loss` : 'a lost packet is lost')}
    </table></section>
    <section class="card"><h2>Video</h2><table class="kv">
      ${row('Per picture', `${r.frame_bytes.toFixed(0)} B`, `${r.packets_per_frame.toFixed(2)} packets`)}
      ${row('Bits per pixel', r.bpp.toFixed(3), `good from ${p.bpp_good}`)}
      ${row('Expected picture', r.quality, r.quality_note, r.quality_cls + '-t')}
      ${row('Largest good picture', `${r.good_res[0]}×${r.good_res[1]}`, `at ${p.fps} fps`)}
    </table></section>`;
  drawBudgetChart();
}

function drawBudgetChart() {
  const r = lbR, p = lbP;
  if (!r || $('page-budget').hidden) return;
  const { g, W, H } = canvasCtx(cvB);
  const L = 54, R = W - 12, Tp = 12, B = H - 26;
  const reach = Math.max(r.sel_range_m < 1e8 ? r.sel_range_m : 1e5, p.dist_km * 1e3, 100);
  const d0 = 10, d1 = Math.min(reach * 4, 1e7);
  const lx = (d) => L + Math.log10(d / d0) / Math.log10(d1 / d0) * (R - L);
  lx.inv = (px) => d0 * 10 ** ((Math.max(L, Math.min(R, px)) - L) / (R - L) * Math.log10(d1 / d0));
  cvB._x = lx;
  const yTop = Math.ceil((r.rxAt(d0) + 5) / 20) * 20, yBot = Math.floor((r.sens_dbm - 20) / 20) * 20;
  const y = (v) => Tp + (yTop - v) / (yTop - yBot) * (B - Tp);
  g.font = MONO; g.textBaseline = 'middle';
  for (let v = yBot; v <= yTop; v += 20) {
    g.strokeStyle = C.grid; g.lineWidth = 1; g.beginPath(); g.moveTo(L, Math.round(y(v)) + .5); g.lineTo(R, Math.round(y(v)) + .5); g.stroke();
    g.fillStyle = C.axis; g.textAlign = 'right'; g.fillText(fmtNum(v), L - 6, y(v));
  }
  g.textAlign = 'center';
  for (let e = 1; 10 ** e <= d1; e++) { const d = 10 ** e; if (d >= d0) g.fillText(LB.dist(d), lx(d), H - 9); }
  g.save(); g.beginPath(); g.rect(L, Tp, R - L, B - Tp); g.clip();
  // below the sensitivity: no link; the fade margin above it
  g.fillStyle = 'rgba(255,138,126,0.07)'; g.fillRect(L, y(r.sens_dbm), R - L, B - y(r.sens_dbm));
  g.strokeStyle = C.bad; g.lineWidth = 1.5; g.beginPath(); g.moveTo(L, y(r.sens_dbm)); g.lineTo(R, y(r.sens_dbm)); g.stroke();
  g.strokeStyle = C.warn; g.setLineDash([5, 4]); g.beginPath(); g.moveTo(L, y(r.sens_dbm + p.fade_db)); g.lineTo(R, y(r.sens_dbm + p.fade_db)); g.stroke(); g.setLineDash([]);
  if (r.horizon_m < d1) { g.strokeStyle = C.axis; g.lineWidth = 1; g.beginPath(); g.moveTo(lx(r.horizon_m), Tp); g.lineTo(lx(r.horizon_m), B); g.stroke(); }
  const dk = p.dist_km * 1e3;
  if (dk >= d0 && dk <= d1) { g.strokeStyle = 'rgba(247,249,248,0.5)'; g.setLineDash([3, 3]); g.beginPath(); g.moveTo(lx(dk), Tp); g.lineTo(lx(dk), B); g.stroke(); g.setLineDash([]); }
  const N = 240, ds = Array.from({ length: N }, (_, i) => d0 * (d1 / d0) ** (i / (N - 1)));
  const curve = (fn, color, w, dash) => {
    g.strokeStyle = color; g.lineWidth = w; g.setLineDash(dash || []); g.beginPath();
    ds.forEach((d, i) => { const v = fn(d); if (i) g.lineTo(lx(d), y(v)); else g.moveTo(lx(d), y(v)); });
    g.stroke(); g.setLineDash([]);
  };
  const showFs = r.sel_kind !== 'fs' || p.extra_loss;
  if (showFs) curve(r.rxFs, C.axis, 1.2, [5, 4]);
  curve(r.rxAt, C.accent, 2);
  g.restore();
  g.font = '11px Helvetica, Arial, sans-serif'; g.textAlign = 'left';
  g.fillStyle = C.bad; g.fillText(`sensitivity ${fmtNum(r.sens_dbm)} dBm`, L + 6, y(r.sens_dbm) + 10);
  g.fillStyle = C.warn; g.fillText(`with ${p.fade_db} dB fade margin`, L + 6, y(r.sens_dbm + p.fade_db) - 10);
  if (r.horizon_m < d1) { g.fillStyle = C.axis; g.fillText('radio horizon', lx(r.horizon_m) + 5, Tp + 8); }
  $('lb-legend').innerHTML = `<span class="legend-i"><i style="border-color:${C.accent}"></i>${LB.MODELS[p.model][0]}</span>` +
    (showFs ? `<span class="legend-i"><i style="border-color:${C.axis};border-top-style:dashed"></i>free space</span>` : '') +
    '<span class="legend-i">dBm</span>';
  const tip = $('lb-tip');
  if (lbHover != null) {
    const d = lbHover, v = r.rxAt(d), px = lx(d), m = v - r.sens_dbm;
    g.strokeStyle = C.ink; g.lineWidth = 1; g.beginPath(); g.moveTo(px + .5, Tp); g.lineTo(px + .5, B); g.stroke();
    g.fillStyle = C.accent; g.strokeStyle = '#121f2a'; g.lineWidth = 2; g.beginPath(); g.arc(px, y(v), 4.5, 0, 7); g.fill(); g.stroke();
    tipAt(tip, cvB.parentElement, px + cvB.offsetLeft, y(v) + cvB.offsetTop,
      `<b>${LB.dist(d)}</b><div class="tr"><span>Received</span><span>${fmtNum(v, 1)} dBm</span></div>` +
      `<div class="tr"><span>Margin</span><span class="${m >= p.fade_db ? 'ok-t' : m >= 0 ? 'warn-t' : 'bad-t'}">${fmtNum(m, 1)} dB</span></div>`);
  } else tip.hidden = true;
}
