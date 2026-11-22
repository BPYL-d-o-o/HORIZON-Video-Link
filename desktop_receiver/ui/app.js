// Horizon Receiver: the Settings window. The video and the bar are drawn natively (lib.rs, render.rs); this page is
// for what is not on the video: graphs, the link's settings, recordings and opened flights.
'use strict';
const T = window.__TAURI__;
const invoke = (cmd, args) => T.core.invoke(cmd, args);
const $ = (id) => document.getElementById(id);
const C = { accent: '#8fc6e5', grid: '#2a3f4d', axis: '#7f95a2', ink: '#f7f9f8', warn: '#f2b84b', bad: '#ff8a7e', sel: 'rgba(143,198,229,0.12)', probBand: 'rgba(242,184,75,0.10)' };
const MONO = '11px "Cascadia Mono", Consolas, monospace';
// not offered as graphs: the delay to the screen means nothing to a pilot, and the link latency needs a stamped stream
const HIDDEN = new Set(['display_ms', 'latency_ms']);
const DEFAULTS = {
  live: ['rssi_dbm', 'snr_db', 'lq_pct', 'fps', 'video_kbps', 'blocks_lost_ps'],
  flight: ['rssi_dbm', 'snr_db', 'lq_pct', 'blocks_lost_ps', 'fps', 'video_kbps'],
};

let FIELDS = [];             // [{key,label,unit}] in Sample.values order
let FIDX = {};               // key -> index
let hist = [];               // live samples (oldest first)
let st = null;               // the last AppState
let liveRange = 120;
let flight = null;           // the opened flight (session::Flight)
let flightDir = null;
let sel = null;              // [t0, t1] selection on the timeline
let tickAt = 0;              // performance.now() of the last tick, for the replay cursor between ticks

// ---- small helpers
const fmtNum = (v, d = 0) => (v == null || !isFinite(v)) ? '–' : v.toFixed(d).replace('-', '−');
const mmss = (s) => { s = Math.max(0, Math.floor(s)); const h = Math.floor(s / 3600), m = Math.floor(s / 60) % 60, x = s % 60;
  return h ? `${h}:${String(m).padStart(2, '0')}:${String(x).padStart(2, '0')}` : `${m}:${String(x).padStart(2, '0')}`; };
const clock = (t, sec = false) => new Date(t * 1000).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', ...(sec ? { second: '2-digit' } : {}) });
const day = (t) => new Date(t * 1000).toLocaleDateString([], { day: 'numeric', month: 'short' });
const bytes = (b) => b >= 1e9 ? (b / 1e9).toFixed(1) + ' GB' : b >= 1e6 ? (b / 1e6).toFixed(1) + ' MB' : Math.max(1, Math.round(b / 1e3)) + ' kB';
const decimals = (unit) => ({ dB: 1, '%': 0, dBm: 0, ms: 1, '/s': 1 }[unit] ?? 0);
const fmtField = (f, v) => v == null ? '–' : `${fmtNum(v, decimals(f.unit))} ${f.unit}`;
function chosen(page) {
  try { const v = JSON.parse(localStorage.getItem('graphs.' + page)); if (Array.isArray(v)) return v.filter((k) => k in FIDX); } catch (_) {}
  return DEFAULTS[page].filter((k) => k in FIDX);
}
function setChosen(page, keys) { try { localStorage.setItem('graphs.' + page, JSON.stringify(keys)); } catch (_) {} }

/** A nice axis for [lo, hi]: three ticks on round numbers. */
function niceAxis(lo, hi) {
  if (!isFinite(lo) || !isFinite(hi)) return { lo: 0, hi: 1, ticks: [0, 1] };
  if (hi - lo < 1e-9) { const p = Math.max(1, Math.abs(hi) * 0.1); lo = lo >= 0 ? Math.max(0, lo - p) : lo - p; hi += p; }
  const raw = (hi - lo) / 2, mag = Math.pow(10, Math.floor(Math.log10(raw))), n = raw / mag;
  const step = (n <= 1 ? 1 : n <= 2 ? 2 : n <= 2.5 ? 2.5 : n <= 5 ? 5 : 10) * mag;
  const nonneg = lo >= 0;
  lo = Math.floor(lo / step) * step; hi = Math.ceil(hi / step) * step;
  if (nonneg && lo < 0) lo = 0; // counts and rates never go below zero
  const ticks = []; for (let v = lo; v <= hi + step / 2; v += step) ticks.push(+v.toFixed(6));
  return { lo, hi, ticks };
}
function canvasCtx(cv) {
  const r = cv.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
  const w = Math.max(1, Math.round(r.width * dpr)), h = Math.max(1, Math.round(r.height * dpr));
  if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
  const g = cv.getContext('2d'); g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, r.width, r.height);
  return { g, W: r.width, H: r.height };
}
/** One series as a line, broken where there is no value or a gap of more than `gap` seconds. With more points than
 *  pixels it is drawn per pixel column: the average as the line, the column's lowest to highest as a faint band (so a
 *  short drop still shows). */
function line(g, times, vals, x, y, gap, color = C.accent, width = 2) {
  const x0 = times.length ? x(times[0]) : 0, x1 = times.length ? x(times[times.length - 1]) : 0;
  if (times.length > 2 * Math.max(1, x1 - x0)) {
    const cols = new Map(); // pixel column -> [sum, n, min, max, last time]
    for (let i = 0; i < times.length; i++) {
      const v = vals[i];
      if (v == null || !isFinite(v)) continue;
      const c = Math.round(x(times[i]));
      const e = cols.get(c);
      if (e) { e[0] += v; e[1]++; e[2] = Math.min(e[2], v); e[3] = Math.max(e[3], v); e[4] = times[i]; }
      else cols.set(c, [v, 1, v, v, times[i]]);
    }
    const keys = [...cols.keys()].sort((p, q) => p - q);
    g.fillStyle = color; g.globalAlpha = 0.28;
    for (const c of keys) { const e = cols.get(c); const t = y(e[3]), b = y(e[2]); g.fillRect(c - 0.5, t, 1, Math.max(1, b - t)); }
    g.globalAlpha = 1;
    const tt = [], vv = [];
    for (const c of keys) { const e = cols.get(c); tt.push(x.inv ? x.inv(c) : e[4]); vv.push(e[0] / e[1]); }
    const perPx = x.inv ? Math.abs(x.inv(1) - x.inv(0)) : gap;
    return line(g, tt, vv, x, y, Math.max(gap * 4, perPx * 3), color, Math.min(width, 1.5));
  }
  g.strokeStyle = color; g.lineWidth = width; g.lineJoin = 'round'; g.beginPath();
  let pen = false, lastT = -Infinity;
  for (let i = 0; i < times.length; i++) {
    const v = vals[i];
    if (v == null || !isFinite(v)) { pen = false; continue; }
    const px = x(times[i]), py = y(v);
    if (pen && times[i] - lastT <= gap) g.lineTo(px, py); else g.moveTo(px, py);
    pen = true; lastT = times[i];
  }
  g.stroke();
}
function nearest(times, t) {
  let lo = 0, hi = times.length - 1;
  if (hi < 0) return -1;
  while (lo < hi) { const m = (lo + hi) >> 1; if (times[m] < t) lo = m + 1; else hi = m; }
  return lo > 0 && Math.abs(times[lo - 1] - t) < Math.abs(times[lo] - t) ? lo - 1 : lo;
}
function tipAt(tip, box, px, py, html) {
  tip.innerHTML = html; tip.hidden = false;
  const tw = tip.offsetWidth, th = tip.offsetHeight;
  tip.style.left = Math.min(box.clientWidth - tw - 4, px + 14) + 'px';
  tip.style.top = Math.max(4, Math.min(box.clientHeight - th - 4, py - th / 2)) + 'px';
}

// ---- pages
const pages = ['live', 'link', 'recordings', 'flight', 'budget'];
function show() {
  let p = location.hash.slice(1) || 'live';
  if (!pages.includes(p) || (p === 'flight' && !flight)) p = 'live';
  for (const q of pages) $('page-' + q).hidden = q !== p;
  document.querySelectorAll('nav a').forEach((a) => a.classList.toggle('on', a.dataset.page === p));
  if (p === 'recordings') loadSessions();
  if (p === 'link') fillLinkForm();
  redraw();
}
window.addEventListener('hashchange', show);
window.addEventListener('resize', () => redraw());
function current() { return pages.find((q) => !$('page-' + q).hidden); }
function redraw() { const p = current(); if (p === 'live') drawLive(); if (p === 'flight') drawFlight(); if (p === 'budget') drawBudget(); }

// ---- the graph checklist (like the main branch's tick list): which statistics are graphed, per page
let graphsFor = null;
// Graphs… opens the checklist and, clicked again, closes it
document.querySelectorAll('[data-graphs]').forEach((b) => b.addEventListener('click', () => {
  if (!$('graphs-pop').hidden && graphsFor === b.dataset.graphs) $('graphs-pop').hidden = true;
  else openGraphs(b.dataset.graphs);
}));
$('graphs-close').addEventListener('click', () => { $('graphs-pop').hidden = true; });
// the viewer's keys also work here (outside text fields): S / Esc close the panel, R records
document.addEventListener('keydown', (e) => {
  const typing = /^(INPUT|SELECT|TEXTAREA)$/.test(e.target.tagName);
  if (e.key === 'Escape') { if (!$('graphs-pop').hidden) $('graphs-pop').hidden = true; else invoke('close_settings'); return; }
  if (typing || e.ctrlKey || e.altKey || e.metaKey) return;
  if (e.key === 's' || e.key === 'S') invoke('close_settings');
  else if (e.key === 'r' || e.key === 'R') invoke('record');
});
$('close').addEventListener('click', () => invoke('close_settings'));
function openGraphs(page) {
  graphsFor = page;
  const on = new Set(chosen(page));
  const list = $('graphs-list'); list.innerHTML = '';
  for (const f of FIELDS) {
    if (HIDDEN.has(f.key)) continue;
    const l = document.createElement('label');
    l.innerHTML = `<input type="checkbox"${on.has(f.key) ? ' checked' : ''}><span></span><span class="u"></span>`;
    l.children[1].textContent = f.label; l.children[2].textContent = f.unit;
    l.firstChild.addEventListener('change', () => {
      const keys = [...list.querySelectorAll('input')].map((c, i) => c.checked ? c.dataset.k : null).filter(Boolean);
      setChosen(graphsFor, keys); buildLiveCharts(); redraw();
    });
    l.firstChild.dataset.k = f.key;
    list.appendChild(l);
  }
  $('graphs-pop').hidden = false;
}

// ---- Live
document.querySelectorAll('#live-range button').forEach((b) => b.addEventListener('click', () => {
  liveRange = +b.dataset.s;
  document.querySelectorAll('#live-range button').forEach((x) => x.classList.toggle('on', x === b));
  drawLive();
}));
let liveHover = null; // unix time under the pointer, shared by all the charts (one crosshair across them)
function buildLiveCharts() {
  const box = $('live-charts'); box.innerHTML = '';
  const keys = chosen('live');
  if (!keys.length) { box.innerHTML = '<p class="empty">No graphs chosen: Graphs… picks them.</p>'; return; }
  for (const k of keys) {
    const f = FIELDS[FIDX[k]];
    const fig = document.createElement('figure'); fig.className = 'card chart'; fig.dataset.k = k;
    fig.innerHTML = '<figcaption><b></b><span></span></figcaption><canvas></canvas><div class="tip" hidden></div>';
    fig.querySelector('b').textContent = f.label; fig.querySelector('figcaption span').textContent = f.unit;
    const cv = fig.querySelector('canvas');
    cv.setAttribute('role', 'img'); cv.setAttribute('aria-label', `${f.label} over time`);
    cv.addEventListener('mousemove', (e) => { liveHover = cv._x ? cv._x.inv(e.offsetX) : null; drawLive(); });
    cv.addEventListener('mouseleave', () => { liveHover = null; drawLive(); });
    box.appendChild(fig);
  }
}
function drawLive() {
  if ($('page-live').hidden) return;
  const s = st && st.sample;
  const v = (k) => s && FIDX[k] != null ? s.values[FIDX[k]] : null;
  const linked = (v('lq_pct') || 0) > 0;
  const lq = v('lq_pct');
  const tot = (k) => s ? s.totals[TOTALS.indexOf(k)] : null;
  const tiles = [
    ['RSSI', linked ? fmtNum(v('rssi_dbm')) + ' dBm' : '–'],
    ['SNR', linked ? fmtNum(v('snr_db'), 1) + ' dB' : '–'],
    ['LINK', lq == null ? '–' : fmtNum(lq) + ' %', lq == null ? '' : lq >= 95 ? 'ok' : lq >= 70 ? 'warn' : 'bad'],
    ['PICTURES/S', fmtNum(v('fps'))],
    ['FIXED/LOST', s ? `${tot('repaired')} · ${tot('blocks_lost')}` : '–'],
  ];
  // since the app started, one line under the tiles (repaired and lost are in a tile, pictures in the charts)
  const COUNTERS = [['pkts', 'packets'], ['misses', 'missed'], ['held', 'held pictures'],
    ['decode_errors', 'decode errors'], ['idents', 'IDs heard']];
  $('live-counters').innerHTML = 'Since start: ' + COUNTERS.map(([k, l]) => `<span><b>${s ? (tot(k) ?? 0).toLocaleString() : '–'}</b> ${l}</span>`).join('');
  $('live-tiles').innerHTML = tiles.map(([k, val, cls]) => `<div class="tile"><span class="k">${k}</span><span class="v ${cls || ''}">${val}</span></div>`).join('');
  const now = s ? s.time : Date.now() / 1000;
  // 0 = all: everything since the app started (the history keeps the last 6 hours)
  const span = liveRange || Math.max(60, now - (hist.length ? hist[0].time : now));
  const t0 = now - span;
  const i0 = Math.max(0, nearest(hist.map((h) => h.time), t0) - 1);
  const part = hist.slice(i0);
  const times = part.map((h) => h.time);
  const label = liveRange ? { 120: '−2 min', 600: '−10 min', 3600: '−1 h' }[liveRange] : clock(t0);
  for (const fig of document.querySelectorAll('#live-charts figure')) {
    const k = fig.dataset.k, f = FIELDS[FIDX[k]], idx = FIDX[k];
    const vals = part.map((h) => h.values[idx]);
    const cv = fig.querySelector('canvas'); const { g, W, H } = canvasCtx(cv);
    const L = 44, R = W - 8, Tp = 8, B = H - 20;
    const finite = vals.filter((x) => x != null && isFinite(x));
    const ax = niceAxis(Math.min(...finite), Math.max(...finite));
    if (f.unit === '%') { ax.lo = 0; ax.hi = 100; ax.ticks = [0, 50, 100]; }
    const x = (t) => L + (t - t0) / span * (R - L); x.inv = (px) => t0 + (px - L) / (R - L) * span;
    const y = (val) => B - (val - ax.lo) / (ax.hi - ax.lo || 1) * (B - Tp);
    cv._x = x;
    g.font = MONO; g.textBaseline = 'middle';
    for (const tk of ax.ticks) {
      g.strokeStyle = C.grid; g.lineWidth = 1; g.beginPath(); g.moveTo(L, Math.round(y(tk)) + .5); g.lineTo(R, Math.round(y(tk)) + .5); g.stroke();
      g.fillStyle = C.axis; g.textAlign = 'right'; g.fillText(fmtNum(tk, ax.ticks.length > 1 && ax.ticks[1] - ax.ticks[0] < 1 ? 1 : 0), L - 6, y(tk));
    }
    g.textAlign = 'left'; g.fillText(label, L, H - 7); g.textAlign = 'right'; g.fillText('now', R, H - 7);
    if (!finite.length) { g.textAlign = 'center'; g.fillText('no data yet', (L + R) / 2, (Tp + B) / 2); fig.querySelector('.tip').hidden = true; continue; }
    g.save(); g.beginPath(); g.rect(L, 0, R - L, H); g.clip();
    line(g, times, vals, x, y, Math.max(2, span / 200));
    g.restore();
    const tip = fig.querySelector('.tip');
    if (liveHover != null && liveHover >= t0 && liveHover <= now) {
      const i = nearest(times, liveHover);
      const px = x(times[i]);
      g.strokeStyle = C.ink; g.lineWidth = 1; g.beginPath(); g.moveTo(px + .5, Tp); g.lineTo(px + .5, B); g.stroke();
      if (vals[i] != null) {
        g.fillStyle = C.accent; g.strokeStyle = '#121f2a'; g.lineWidth = 2; g.beginPath(); g.arc(px, y(vals[i]), 4.5, 0, 7); g.fill(); g.stroke();
      }
      tipAt(tip, fig, px + cv.offsetLeft, y(vals[i] ?? ax.lo) + cv.offsetTop, `<b>${clock(times[i], true)}</b><div class="tr"><span>${f.label}</span><span>${fmtField(f, vals[i])}</span></div>`);
    } else tip.hidden = true;
  }
}

// ---- Link
// Hopping: the firmware puts channel k (0..n-1) at centre + (k - floor(n/2)) x spacing. Here the plan is a band and
// a channel count: the outer channels sit half the signal width (1 MHz at 1000 kHz bandwidth) inside the band edges.
const SIGNAL_MHZ = 1;
function bandToPlan(lo, hi, n) {
  if (!(n >= 2 && n <= 128)) throw '2 to 128 channels';
  const f0 = lo + SIGNAL_MHZ / 2, f1 = hi - SIGNAL_MHZ / 2;
  if (!(f1 > f0)) throw `the band must be wider than the ${SIGNAL_MHZ} MHz signal`;
  const stepKhz = Math.floor((f1 - f0) * 1000 / (n - 1));
  if (stepKhz < 1) throw 'too many channels for this band';
  const span = (n - 1) * stepKhz / 1000;
  const first = f0 + ((f1 - f0) - span) / 2;
  const centreHz = Math.round((first + Math.floor(n / 2) * stepKhz / 1000) * 1e6);
  return { centreHz, stepHz: stepKhz * 1000, first, last: first + span, overlap: stepKhz / 1000 < SIGNAL_MHZ };
}
function planToBand(centreHz, n, stepHz) {
  const first = (centreHz - Math.floor(n / 2) * stepHz) / 1e6, last = first + (n - 1) * stepHz / 1e6;
  return { lo: first - SIGNAL_MHZ / 2, hi: last + SIGNAL_MHZ / 2, first, last };
}
const num = (id) => parseFloat($(id).value.replace(',', '.').replace('−', '-'));
function showHopPlan() {
  const on = $('f-hop').checked;
  $('f-hop-box').hidden = !on; $('f-freq-l').hidden = on;
  if (!on) { $('lk-msg').textContent = ''; return; }
  const el = $('f-hop-plan');
  try {
    const n = parseInt($('f-hopn').value, 10);  // one packet per channel
    const p = bandToPlan(num('f-lo'), num('f-hi'), n);
    el.className = 'small ' + (p.overlap ? 'warn-t' : 'muted');
    el.textContent = `${n} channels ${(p.stepHz / 1e6).toFixed(3)} MHz apart, centred ${p.first.toFixed(3)} to ${p.last.toFixed(3)} MHz; ` +
      `one packet (${(17.54).toFixed(0)} ms) on each, the whole band every ${(n * 17.54 / 1000).toFixed(2)} s.` +
      (p.overlap ? ` The channels overlap (closer than the ${SIGNAL_MHZ} MHz signal): fewer channels or a wider band avoids that.` : '');
  } catch (e) { el.className = 'small msg-bad'; el.textContent = String(e); }
}
['f-hop', 'f-lo', 'f-hi', 'f-hopn'].forEach((id) => $(id).addEventListener(id === 'f-hop' ? 'change' : 'input', () => {
  if (id === 'f-hop' && $('f-hop').checked && !$('f-lo').value) {
    // a starting band around the current frequency (the EU 863-870 MHz band when the frequency is in it)
    const f = num('f-freq') || 868;
    const [lo, hi] = f >= 863 && f <= 870 ? [863, 870] : f >= 2400 ? [2400, 2483.5] : [f - 4, f + 4];
    $('f-lo').value = lo.toFixed(3); $('f-hi').value = hi.toFixed(3);
    // as many channels as fit side by side without overlapping
    $('f-hopn').value = String(f >= 2400 ? 100 : Math.max(2, Math.min(128, Math.floor((hi - lo) / SIGNAL_MHZ))));
  }
  showHopPlan();
}));
let linkFormFilled = false;
function fillLinkForm(force) {
  const c = st && st.ground.cfg;
  if (!c || (linkFormFilled && !force)) return;
  linkFormFilled = true;
  $('f-pwr').value = String(c.pwr_dbm);
  $('f-hop').checked = !!c.hop;
  if (c.hop) {
    const b = planToBand(c.freq_hz, c.hop[0], c.hop[1]);
    $('f-lo').value = b.lo.toFixed(3); $('f-hi').value = b.hi.toFixed(3);
    $('f-hopn').value = c.hop[0];
    $('f-freq').value = (c.freq_hz / 1e6).toFixed(3);
  } else $('f-freq').value = (c.freq_hz / 1e6).toFixed(3);
  showHopPlan();
}
/** The Ground's "K profile freq_hz=… hop=n,step pwr=<half dBm> saved=1" in words. */
function savedPairing(line) {
  if (!line) return '';
  const kv = Object.fromEntries(line.split(/\s+/).filter((x) => x.includes('=')).map((x) => x.split('=')));
  if (kv.saved === '0') return 'Nothing saved on the Ground yet: Pair both boards on the setup site.';
  const parts = [];
  if (kv.freq_hz) parts.push((kv.freq_hz / 1e6).toFixed(3) + ' MHz');
  if (kv.pwr != null) parts.push(fmtNum(kv.pwr / 2, kv.pwr % 2 ? 1 : 0) + ' dBm');
  const hop = (kv.hop || '0').split(',');
  parts.push(+hop[0] > 0 ? `hopping over ${hop[0]} channels` : 'no hopping');
  return 'Saved by pairing (used after a restart): ' + parts.join(' · ');
}
function drawLink() {
  const gr = st.ground, c = gr.cfg;
  const lq = st.sample && FIDX.lq_pct != null ? st.sample.values[FIDX.lq_pct] : null;
  let dot = 'bad', title = gr.error || 'No receiver';
  if (gr.port && !gr.error) {
    if (lq > 0) { dot = 'ok'; title = 'Receiving the Air unit'; }
    else if (c && !c.locked) { dot = 'warn'; title = 'Searching for the Air unit'; }
    else { dot = 'warn'; title = 'Receiver ready, nothing heard from the Air unit'; }
  }
  $('lk-dot').className = 'dot ' + dot; $('lk-title').textContent = title;
  $('lk-freq').textContent = c ? (c.freq_hz / 1e6).toFixed(3) + ' MHz' : '–';
  $('lk-pwr').textContent = c ? fmtNum(c.pwr_dbm, c.pwr_dbm % 1 ? 1 : 0) + ' dBm' : '–';
  if (c && c.hop) { const b = planToBand(c.freq_hz, c.hop[0], c.hop[1]); $('lk-hop').textContent = `${c.hop[0]} channels, ${b.lo.toFixed(1)}–${b.hi.toFixed(1)} MHz`; }
  else $('lk-hop').textContent = c ? 'off' : '–';
  $('lk-freq-k').textContent = c && c.hop ? 'CENTRE CHANNEL' : 'FREQUENCY';
  fillLinkForm();
  $('lk-port').textContent = gr.port || '–';
  $('lk-profile').textContent = savedPairing(gr.profile);
  $('lk-problems').innerHTML = '';
  for (const t of gr.problems || []) { const d = document.createElement('div'); d.className = 'problem-row'; d.textContent = t; $('lk-problems').appendChild(d); }
  if ((gr.problems || []).length) { $('lk-dot').className = 'dot bad'; }
  if (gr.ident) {
    $('cs-heard').textContent = gr.ident[1];
    $('cs-when').textContent = `heard ${clock(gr.ident[0], true)}` + (gr.ident[1] === 'N0CALL' ? ' · debugging default' : '');
    $('cs-when').className = 'small ' + (gr.ident[1] === 'N0CALL' ? 'warn-t' : 'muted');
  } else { $('cs-heard').textContent = 'not heard yet'; $('cs-when').textContent = ''; }
}
function msg(el, text, ok) { el.textContent = text; el.className = 'small ' + (ok ? 'msg-ok' : 'msg-bad'); }
$('lk-apply').addEventListener('click', async () => {
  const m = $('lk-msg'), pwr = num('f-pwr');
  try {
    if ($('f-hop').checked) {
      const n = parseInt($('f-hopn').value, 10);
      if (!(d >= 1 && d <= 255)) throw 'slots per channel: 1 to 255';
      const p = bandToPlan(num('f-lo'), num('f-hi'), n);
      await invoke('set_hopping', { centreHz: p.centreHz, n, spacingHz: p.stepHz });
    } else {
      const freq = num('f-freq');
      if (!isFinite(freq)) throw 'enter the frequency in MHz';
      const hz = Math.round(freq * 1e6);
      await invoke('set_hopping', { centreHz: hz, n: 0, spacingHz: 0 });
      await invoke('set_frequency', { hz });
    }
    if (isFinite(pwr)) await invoke('set_power', { dbm: pwr });
    msg(m, 'Sent. The Air unit follows within a second; the values above confirm it.', true);
  } catch (e) { msg(m, String(e), false); }
});

// ---- Picture options (saved by the app)
let prefs = null;
async function loadPrefs() {
  prefs = await invoke('get_prefs');
  $('pic-flip').value = String(prefs.flip);
}
function savePrefs() {
  prefs = { ...prefs, flip: +$('pic-flip').value };
  invoke('set_prefs', { p: prefs });
}
['pic-flip'].forEach((id) => $(id).addEventListener('change', savePrefs));

// ---- Developer: injected packet loss (not saved: off whenever the app starts)
async function setInjection(pct, burst) {
  await invoke('set_injection', { percent: pct, burst });
  const m = $('inj-msg');
  if (pct > 0) msg(m, `On: ${pct} % of the packets dropped, bursts of about ${Math.round(burst * 18.7)} ms on average.`, false);
  else { m.textContent = 'Off.'; m.className = 'small muted'; }
}
$('inj-apply').addEventListener('click', () => {
  const pct = parseFloat($('inj-pct').value.replace(',', '.')) || 0, burst = parseFloat($('inj-burst').value.replace(',', '.')) || 1;
  if (pct < 0 || pct > 90) return msg($('inj-msg'), 'loss: 0 to 90 %', false);
  setInjection(pct, Math.max(1, burst));
});
$('inj-off').addEventListener('click', () => { $('inj-pct').value = '0'; setInjection(0, 1); });

// ---- Recordings
$('rec-btn').addEventListener('click', () => invoke('record').then(() => setTimeout(loadSessions, 600)));
$('open-dir').addEventListener('click', () => invoke('open_folder', { path: null }));
$('change-dir').addEventListener('click', async () => { if (await invoke('choose_out_dir')) { await loadPrefs(); loadSessions(); } });
$('open-other').addEventListener('click', async () => { const d = await invoke('pick_flight'); if (d) openFlight(d); });
function drawRec() {
  const r = st.recording;
  $('rec-chip').hidden = !r;
  if (r) $('rec-text').textContent = `REC ${mmss(Date.now() / 1000 - r.started)} · ${bytes(r.bytes)}`;
  const b = $('rec-btn');
  b.textContent = r ? 'Stop recording (R)' : 'Record (R)'; b.className = 'btn' + (r ? ' recording' : ' primary');
  $('out-dir').textContent = st.out_dir;
  if (!!r !== drawRec.was) { drawRec.was = !!r; if (!r && current() === 'recordings') setTimeout(loadSessions, 300); }
}
const exporting = {}; // dir -> text
async function loadSessions() {
  const list = await invoke('sessions');
  const tb = $('sessions');
  if (!list.length) { tb.innerHTML = '<tr><td colspan="5" class="faint">Nothing recorded yet. Record (R) in the viewer or above.</td></tr>'; return; }
  tb.innerHTML = '';
  for (const s of list) {
    const tr = document.createElement('tr');
    const link = s.link_pct == null ? '–' : `${fmtNum(s.link_pct, s.link_pct > 99 && s.link_pct < 100 ? 2 : 1)} % · ${s.problems} problem${s.problems === 1 ? '' : 's'}`;
    tr.innerHTML = `<td>${day(s.start)} ${clock(s.start)}</td><td>${mmss(s.duration)}</td><td class="${s.problems ? 'warn-t' : ''}">${link}</td><td>${bytes(s.bytes)}</td><td class="acts"></td>`;
    const acts = tr.lastChild;
    const open = document.createElement('button'); open.className = 'btn small primary'; open.textContent = 'Open';
    open.addEventListener('click', () => openFlight(s.dir));
    acts.appendChild(open);
    if (s.replayable) {
      const ex = document.createElement('button'); ex.className = 'btn small'; ex.textContent = exporting[s.dir] || 'Export video…';
      ex.dataset.dir = s.dir;
      ex.addEventListener('click', () => exportVideo(s.dir, s.start, s.start + s.duration));
      acts.appendChild(ex);
    } else {
      const n = document.createElement('span'); n.className = 'faint small'; n.textContent = ' statistics only'; acts.appendChild(n);
    }
    tb.appendChild(tr);
  }
}
async function exportVideo(dir, from, to) {
  const out = await invoke('export_video', { dir, from, to });
  exporting[dir] = 'Exporting…';
  $('fl-export-msg').textContent = 'Writing ' + out.split(/[\\/]/).pop() + '…'; $('fl-export-msg').className = 'small muted';
  document.querySelectorAll(`button[data-dir]`).forEach((b) => { if (b.dataset.dir === dir) b.textContent = 'Exporting…'; });
}
T.event.listen('export', (e) => {
  const p = e.payload, name = p.path.split(/[\\/]/).pop();
  const dir = p.path.replace(/[\\/][^\\/]*$/, '');
  const fm = $('fl-export-msg');
  if (p.error) { fm.textContent = p.error; fm.className = 'small msg-bad'; delete exporting[dir]; }
  else if (p.done) {
    delete exporting[dir];
    fm.innerHTML = ''; const a = document.createElement('a'); a.href = '#'; a.textContent = 'Saved ' + name + ' (show)';
    a.addEventListener('click', (ev) => { ev.preventDefault(); invoke('open_folder', { path: p.path }); });
    fm.appendChild(a); fm.className = 'small';
  } else { fm.textContent = `Writing ${name}: ${Math.round(p.progress * 100)} %`; fm.className = 'small muted'; exporting[dir] = `Exporting ${Math.round(p.progress * 100)} %`; }
  document.querySelectorAll('button[data-dir]').forEach((b) => { if (b.dataset.dir === dir) b.textContent = exporting[dir] || (p.error ? 'Export failed' : 'Exported ✓'); });
});

// ---- Flight: the opened session's statistics, one lane per chosen graph, the replay's cursor across all of them
async function openFlight(dir, loaded) {
  try { flight = loaded || await invoke('open_flight', { dir }); } catch (e) { alert(e); return; }
  flightDir = dir; sel = null;
  $('nav-flight').hidden = false;
  const s = flight.summary;
  $('fl-title').textContent = `Flight ${day(s.start)} ${clock(s.start)}`;
  const tiles = [
    ['LINK', s.link_pct == null ? '–' : fmtNum(s.link_pct, 1) + ' %'],
    ['PROBLEMS', String(flight.problems.length), flight.problems.length ? 'warn' : 'ok'],
    ['RSSI MEDIAN · MIN', `${fmtNum(s.rssi_median)} · ${fmtNum(s.rssi_min)} dBm`],
    ['PICTURES', s.frames.toLocaleString()],
    ['PACKETS LOST', String(s.lost)],
  ];
  $('fl-tiles').innerHTML = tiles.map(([k, v, c]) => `<div class="tile"><span class="k">${k}</span><span class="v ${c || ''}">${v}</span></div>`).join('');
  const pb = $('fl-problems'); pb.innerHTML = '';
  if (!flight.problems.length) pb.innerHTML = '<p class="muted small">None: the link held for the whole flight.</p>';
  for (const p of flight.problems) {
    const b = document.createElement('button');
    b.innerHTML = `<b>${mmss(p.start - s.start)} · ${Math.max(1, Math.round(p.end - p.start))} s</b><span></span>`;
    b.lastChild.textContent = p.text;
    b.addEventListener('click', () => seek(Math.max(s.start, p.start - 3)));
    pb.appendChild(b);
  }
  $('fl-export-all').disabled = !s.replayable; $('fl-export-sel').disabled = true;
  $('fl-sel').textContent = s.replayable ? 'Drag on the timeline to pick a part.' : 'This session has statistics only, no packets: nothing to replay or export.';
  $('fl-export-msg').textContent = '';
  location.hash = '#flight'; show();
}
function flightRange() { const s = flight.summary; return [s.start, s.start + Math.max(1, s.duration)]; }
function replayPos() {
  const r = st && st.replay;
  if (!r || r.dir !== flightDir) return null;
  let p = r.pos;
  if (r.playing) p += (performance.now() - tickAt) / 1000 * r.speed;
  return Math.min(p, r.end);
}
async function seek(t) {
  if (!st || !st.replay || st.replay.dir !== flightDir) { await invoke('open_flight', { dir: flightDir }); }
  await invoke('replay_control', { action: 'seek', value: t });
  if (st && st.replay) { st.replay.pos = t; tickAt = performance.now(); }
}
let flHover = null, drag = null;
const cvF = $('fl-canvas');
function flX() { return cvF._x; }
cvF.addEventListener('mousedown', (e) => { if (flX()) drag = { x0: e.offsetX, t0: flX().inv(e.offsetX) }; });
cvF.addEventListener('mousemove', (e) => {
  if (!flX()) return;
  const [a, b] = flightRange();
  flHover = Math.max(a, Math.min(b, flX().inv(e.offsetX)));
  if (drag && Math.abs(e.offsetX - drag.x0) > 4) { const t = flHover; sel = [Math.min(drag.t0, t), Math.max(drag.t0, t)]; showSel(); }
  drawFlight();
});
cvF.addEventListener('mouseleave', () => { flHover = null; drawFlight(); });
window.addEventListener('mouseup', (e) => {
  if (!drag) return;
  const moved = Math.abs(e.offsetX - drag.x0) > 4 && e.target === cvF;
  if (!moved && e.target === cvF && flight.summary.replayable) { sel = null; showSel(); seek(Math.max(flightRange()[0], drag.t0)); }
  drag = null; drawFlight();
});
function showSel() {
  const s = flight.summary;
  if (!sel || sel[1] - sel[0] < 1) { $('fl-sel').textContent = s.replayable ? 'Drag on the timeline to pick a part.' : $('fl-sel').textContent; $('fl-export-sel').disabled = true; return; }
  $('fl-sel').innerHTML = `Selected: <span class="mono">${mmss(sel[0] - s.start)} – ${mmss(sel[1] - s.start)}</span> (${Math.round(sel[1] - sel[0])} s)`;
  $('fl-export-sel').disabled = !s.replayable;
}
$('fl-export-sel').addEventListener('click', () => sel && exportVideo(flightDir, sel[0], sel[1]));
$('fl-export-all').addEventListener('click', () => { const [a, b] = flightRange(); exportVideo(flightDir, a, b); });

function drawFlight() {
  if ($('page-flight').hidden || !flight) return;
  const s = flight.summary, [a, b] = flightRange();
  const r = st && st.replay && st.replay.dir === flightDir ? st.replay : null;
  $('fl-sub').textContent = `${mmss(s.duration)} · ` + (r ? (r.error ? r.error : r.ended ? 'ended in the viewer' : r.playing ? `playing in the viewer${r.speed !== 1 ? ' at ' + r.speed + '×' : ''}` : 'paused in the viewer')
    : s.replayable ? 'not in the viewer: click the timeline to play from there' : 'statistics only');
  const { g, W, H } = canvasCtx(cvF);
  const keys = chosen('flight').filter((k) => flight.series.some((x) => x[0] === k));
  const L = 52, R = W - 10, Tp = 12, B = H - 22;
  const x = (t) => L + (t - a) / (b - a) * (R - L); x.inv = (px) => a + (px - L) / (R - L) * (b - a);
  cvF._x = x;
  g.font = MONO; g.textBaseline = 'middle';
  if (!keys.length) { g.fillStyle = C.axis; g.textAlign = 'center'; g.fillText('No graphs chosen: Graphs… picks them.', W / 2, H / 2); return; }
  // problems: shaded spans with a mark on top; the selection
  for (const p of flight.problems) {
    g.fillStyle = C.probBand; g.fillRect(x(p.start), Tp, Math.max(3, x(p.end) - x(p.start)), B - Tp);
    g.fillStyle = C.warn; g.fillRect(x(p.start) - 2, 0, 4, 9);
  }
  if (sel) { g.fillStyle = C.sel; g.fillRect(x(sel[0]), Tp, x(sel[1]) - x(sel[0]), B - Tp); }
  const laneH = (B - Tp) / keys.length;
  const tipRows = [];
  const hi = flHover != null ? nearest(flight.time, flHover) : -1;
  keys.forEach((k, li) => {
    const f = FIELDS[FIDX[k]], vals = flight.series.find((x) => x[0] === k)[1];
    const top = Tp + li * laneH + 16, bot = Tp + (li + 1) * laneH - 6;
    const finite = vals.filter((v) => v != null);
    const ax = f.unit === '%' ? { lo: 0, hi: 100, ticks: [0, 100] } : niceAxis(Math.min(...finite), Math.max(...finite));
    const y = (v) => bot - (v - ax.lo) / (ax.hi - ax.lo || 1) * (bot - top);
    g.strokeStyle = C.grid; g.lineWidth = 1; g.beginPath(); g.moveTo(L, Math.round(bot) + .5); g.lineTo(R, Math.round(bot) + .5); g.stroke();
    g.fillStyle = '#b5c5ce'; g.textAlign = 'left'; g.font = '12px Helvetica, Arial, sans-serif'; g.fillText(`${f.label} (${f.unit})`, L + 4, top - 8);
    g.font = MONO; g.fillStyle = C.axis; g.textAlign = 'right';
    if (finite.length) { const d = ax.hi - ax.lo < 4 ? 1 : 0; g.fillText(fmtNum(ax.hi, d), L - 6, top + 4); g.fillText(fmtNum(ax.lo, d), L - 6, bot - 4); }
    g.save(); g.beginPath(); g.rect(L, top - 2, R - L, bot - top + 4); g.clip();
    line(g, flight.time, vals, x, y, 3, C.accent, 1.8);
    if (hi >= 0 && vals[hi] != null) {
      g.fillStyle = C.accent; g.strokeStyle = '#121f2a'; g.lineWidth = 2; g.beginPath(); g.arc(x(flight.time[hi]), y(vals[hi]), 4, 0, 7); g.fill(); g.stroke();
    }
    g.restore();
    if (hi >= 0) tipRows.push(`<div class="tr"><span>${f.label}</span><span>${fmtField(f, vals[hi])}</span></div>`);
  });
  // time axis
  g.fillStyle = C.axis; g.font = MONO;
  const dur = b - a, step = [10, 30, 60, 120, 300, 600, 900, 1800, 3600].find((s2) => dur / s2 <= 8) || 7200;
  for (let t = 0; t <= dur; t += step) { g.textAlign = t === 0 ? 'left' : 'center'; g.fillText(mmss(t), x(a + t), H - 8); }
  // the replay's cursor
  const pos = replayPos();
  if (pos != null) {
    const px = Math.round(x(pos)) + .5;
    g.strokeStyle = C.ink; g.lineWidth = 1.5; g.beginPath(); g.moveTo(px, Tp - 4); g.lineTo(px, B); g.stroke();
    const lbl = mmss(pos - a); g.font = '600 11px "Cascadia Mono", Consolas, monospace';
    const tw = g.measureText(lbl).width + 12, lx = Math.max(L, Math.min(R - tw, px - tw / 2));
    g.fillStyle = C.ink; g.beginPath(); g.roundRect(lx, B + 2, tw, 16, 3); g.fill();
    g.fillStyle = '#0c1620'; g.textAlign = 'center'; g.fillText(lbl, lx + tw / 2, B + 10.5);
  }
  // hover: crosshair and every lane's value
  const tip = $('fl-tip');
  if (hi >= 0 && !drag) {
    const px = x(flight.time[hi]);
    g.strokeStyle = 'rgba(247,249,248,0.35)'; g.lineWidth = 1; g.setLineDash([3, 3]); g.beginPath(); g.moveTo(px + .5, Tp); g.lineTo(px + .5, B); g.stroke(); g.setLineDash([]);
    const prob = flight.problems.find((p) => flight.time[hi] >= p.start - 1 && flight.time[hi] <= p.end + 1);
    tipAt(tip, cvF.parentElement, px + cvF.offsetLeft, Tp + 60, `<b>${mmss(flight.time[hi] - a)}</b> <span class="faint">${clock(flight.time[hi], true)}</span>${tipRows.join('')}` +
      (prob ? `<div class="warn-t" style="margin-top:4px"></div>` : '') + (s.replayable ? '<div class="faint" style="margin-top:4px">click: play from here · drag: select</div>' : ''));
    if (prob) tip.querySelector('.warn-t').textContent = prob.text;
  } else tip.hidden = true;
}
function animate() { if (current() === 'flight' && st && st.replay && st.replay.playing) drawFlight(); requestAnimationFrame(animate); }

// ---- state from the app
let TOTALS = ['pkts', 'filler', 'misses', 'blocks_ok', 'blocks_lost', 'frames', 'decode_errors', 'resyncs', 'out_dropped', 'aus_dropped', 'repaired', 'crc32_bad', 'idents', 'held', 'concealed'];
function onState(s) {
  st = s; tickAt = performance.now();
  if (s.sample && (!hist.length || s.sample.time > hist[hist.length - 1].time)) {
    hist.push(s.sample);
    if (hist.length > 6 * 3600 * 2) hist.splice(0, hist.length - 6 * 3600 * 2);
  }
  const g = s.ground;
  const who = Object.fromEntries(g.who), board = who.git || who.fw;
  $('who').textContent = g.port ? `Ground · ${g.port}` + (board ? `\nfirmware ${board.slice(0, 8)}` : '') : (g.error || 'no receiver');
  drawRec(); drawLink();
  if (current() === 'live') drawLive();
  if (current() === 'flight') drawFlight();
}
(async function start() {
  FIELDS = await invoke('fields');
  FIELDS.forEach((f, i) => { FIDX[f.key] = i; });
  hist = await invoke('history', { since: 0 });
  await loadPrefs();
  buildLiveCharts();
  onState(await invoke('state'));
  T.event.listen('tick', (e) => onState(e.payload));
  T.event.listen('flight-opened', (e) => openFlight(e.payload.dir, e.payload.flight));
  T.event.listen('replay-closed', () => { if (st) st.replay = null; drawFlight(); });
  show(); animate();
})().catch((e) => { document.body.insertAdjacentHTML('afterbegin', `<p style="color:#ff8a7e;padding:20px">${e}</p>`); });
