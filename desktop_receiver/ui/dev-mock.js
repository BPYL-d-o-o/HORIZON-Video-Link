// Testing only: when this page is opened in an ordinary browser (no Tauri), a fake backend with made-up statistics,
// recordings and a flight, so the Settings pages can be checked without the app or a Ground:
//   cd desktop_receiver/ui && python -m http.server 8792   ->   http://127.0.0.1:8792/
// Inside the app window.__TAURI__ exists and this file does nothing.
'use strict';
(() => {
  if (window.__TAURI__) return;
  const FIELDS = [
    ['rssi_dbm', 'RSSI (packet)', 'dBm'], ['rssi_sig_dbm', 'RSSI (signal)', 'dBm'], ['snr_db', 'SNR', 'dB'],
    ['noise_dbm', 'Channel RSSI before packet', 'dBm'], ['lq_pct', 'Link quality', '%'], ['pps', 'Packets received', '/s'],
    ['data_pps', 'Video packets', '/s'], ['filler_pps', 'Idle (filler) packets', '/s'], ['miss_ps', 'Missed slots', '/s'],
    ['crc_ps', 'CRC errors', '/s'], ['blocks_ok_ps', 'Video packets received', '/s'], ['blocks_lost_ps', 'Video packets lost', '/s'],
    ['block_loss_pct', 'Video packet loss (1 s)', '%'], ['repaired_ps', 'Damaged packets repaired', '/s'], ['fps', 'Decoded frame rate', 'fps'],
    ['video_kbps', 'Video bitrate', 'kbit/s'], ['latency_ms', 'Link latency (needs --stamp)', 'ms'], ['decode_ms', 'Decode time', 'ms'],
    ['display_ms', 'Decode -> on screen', 'ms'], ['foff_hz', 'Frequency offset', 'Hz'], ['timing_us', 'Packet timing error', 'us'],
    ['div_saved_ps', 'Diversity: packets only another receiver got', '/s'],
  ].map(([key, label, unit]) => ({ key, label, unit }));
  const TOTALS = 15;
  // a deterministic wobble, and a dip every few minutes
  const wave = (t, a, p) => Math.sin(t / p) * a + Math.sin(t / (p * 0.37)) * a * 0.4;
  function sampleAt(t) {
    const dip = Math.max(0, 1 - Math.abs(((t % 300) - 150)) / 6); // a 12 s problem every 5 minutes
    const lq = 100 - dip * 30, lost = dip > 0.3 ? 3 * dip : 0;
    const v = {
      rssi_dbm: -84 + wave(t, 3, 40) - dip * 8, rssi_sig_dbm: -85 + wave(t, 3, 40), snr_db: 9 + wave(t, 1.2, 25) - dip * 5,
      noise_dbm: -110 + wave(t, 1, 60), lq_pct: lq, pps: 53 * lq / 100, data_pps: 50 * lq / 100, filler_pps: 3, miss_ps: 53 - 53 * lq / 100,
      crc_ps: 0, blocks_ok_ps: 50 * lq / 100, blocks_lost_ps: lost, block_loss_pct: lost * 2, repaired_ps: 0,
      fps: 29 + wave(t, 1.5, 10) - dip * 12, video_kbps: 90 + wave(t, 6, 15) - dip * 40, latency_ms: null, decode_ms: 1.2,
      display_ms: 2.5, foff_hz: 400 + wave(t, 80, 120), timing_us: wave(t, 20, 30), div_saved_ps: null,
    };
    return { time: t, values: FIELDS.map((f) => v[f.key] ?? null), totals: Array.from({ length: TOTALS }, (_, i) => Math.floor((t % 10000) / (i + 3))) };
  }
  const now = () => Date.now() / 1000;
  const start = now() - 3600;
  let prefs = { flip: 0, out_dir: null };
  let replay = null;
  const sessions = [0, 1, 2].map((i) => {
    const s = now() - 86400 * i - 7200 - i * 1800, d = [295, 3250, 752][i];
    return { dir: `C:\\Users\\pilot\\HorizonReceiver\\flight_${i}`, name: `flight_${i}`, start: s, duration: d, link_pct: [99.9, 99.96, 100][i],
      problems: [1, 2, 0][i], bytes: [2.1e6, 22e6, 5e6][i], replayable: i !== 2, rssi_median: -83, rssi_min: -91, frames: d * 30, lost: [28, 61, 0][i] };
  });
  function flight(dir) {
    const s = sessions.find((x) => x.dir === dir) || sessions[0];
    const time = [], series = FIELDS.map((f) => [f.key, []]);
    for (let t = s.start; t <= s.start + s.duration; t += 0.5) {
      const smp = sampleAt(t - s.start + 140);
      time.push(t); smp.values.forEach((v, i) => series[i][1].push(v));
    }
    const problems = [{ start: s.start + 4, end: s.start + 14, text: 'link down to 70 %, 27 video packets lost' }];
    return { summary: s, problems: problems.slice(0, s.problems), time, series: series.filter((x) => x[1].some((v) => v != null)) };
  }
  function state() {
    const t = now();
    return {
      ground: { port: 'COM8', error: '', who: [['board', 'hat'], ['git', 'e7e9218e'], ['fmt', '4']], cfg: { freq_hz: 868000000, pwr_dbm: -9.5, locked: true, hop: null },
        profile: 'K profile freq_hz=868000000 hop=0,0,0 pwr=-19 saved=1', ident: [t - 95, 'N0CALL'], status: '',
        problems: location.hash.includes('problems') ? ['COM9: Ground firmware 1234abcd is not up to date (this app: e7e9218e): update it on the setup site, Ground page'] : [] },
      recording: null, replay, out_dir: 'C:\\Users\\pilot\\HorizonReceiver', bar: true,
      render: { gpu: 'test', mode: 'Hardware: Independent Flip', screen: '2560x1440', tearing: true, error: '' }, sample: sampleAt(t),
    };
  }
  const handlers = {
    fields: () => FIELDS, state, history: ({ since }) => { const out = []; for (let t = Math.max(start, since + 0.5); t < now(); t += 0.5) out.push(sampleAt(t)); return out; },
    sessions: () => sessions, open_flight: ({ dir }) => { const f = flight(dir); replay = { dir, name: f.summary.name, start: f.summary.start, end: f.summary.start + f.summary.duration, pos: f.summary.start + 30, playing: true, speed: 1, ended: false, error: '' }; return f; },
    replay_control: ({ action, value }) => { if (replay && action === 'seek') replay.pos = value; if (action === 'live') replay = null; },
    get_prefs: () => prefs, set_prefs: ({ p }) => { prefs = p; }, export_video: ({ dir }) => dir + '\\video.mp4', choose_out_dir: () => null, pick_flight: () => null,
  };
  const listeners = {};
  window.__TAURI__ = {
    core: { invoke: async (cmd, args) => { console.log('[mock] invoke', cmd, args || ''); return (handlers[cmd] || (() => null))(args || {}); } },
    event: { listen: async (ev, fn) => { (listeners[ev] = listeners[ev] || []).push(fn); return () => {}; } },
  };
  setInterval(() => {
    if (replay && replay.playing) replay.pos = Math.min(replay.end, replay.pos + 0.5);
    (listeners.tick || []).forEach((fn) => fn({ payload: state() }));
  }, 500);
})();
