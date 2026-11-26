//! Replay of a session in the viewer: its packets.rxlog lines, paced by their capture times (½× to 4×), through the same
//! receiver engine and decoder as live. Seeking reopens the file a few seconds before the target and feeds those
//! seconds at full speed without showing them ("warming"), so the decoder has the pictures the target is predicted
//! from; then it plays on from the target.
use crate::session;
use std::io::BufRead;
use std::path::PathBuf;
use std::sync::mpsc::{Receiver, RecvTimeoutError};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

const WARM_S: f64 = 3.0;

pub enum Cmd {
    Play,
    Pause,
    Toggle,
    /// absolute unix time
    Seek(f64),
    /// relative, seconds
    Step(f64),
    Speed(f64),
    Stop,
}

#[derive(Clone, Default, serde::Serialize)]
pub struct State {
    pub dir: String,
    pub name: String,
    pub start: f64,
    pub end: f64,
    pub pos: f64,
    pub playing: bool,
    pub speed: f64,
    pub ended: bool,
    pub error: String,
}

/// What the replay hands on: one receiver line, its capture time, and whether its pictures are to be shown (false
/// while warming up after a seek). `seek` is true for the first line after a seek (the decoder starts over).
pub type Sink = Box<dyn FnMut(&str, f64, bool, bool) + Send>;

pub fn run(path: PathBuf, state: Arc<Mutex<State>>, cmds: Receiver<Cmd>, mut sink: Sink) {
    let idx = match session::index_rxlog(&path) {
        Ok(i) if !i.is_empty() => i,
        Ok(_) => {
            state.lock().unwrap().error = "the packet capture is empty".into();
            return;
        }
        Err(e) => {
            state.lock().unwrap().error = e.to_string();
            return;
        }
    };
    let (start, end) = (idx[0].0, idx.last().unwrap().0 + 0.5);
    {
        let mut s = state.lock().unwrap();
        s.start = start;
        s.end = s.end.max(end);
        s.pos = start;
        s.speed = 1.0;
        s.playing = true;
    }
    let mut target = start; // where the picture is to continue from
    let mut reader = None;
    let mut fresh = true; // the next line starts over (after open / seek)
    let mut clock: Option<(Instant, f64)> = None; // (wall time, log time) of the last pacing anchor
    let mut pending: Option<(f64, String)> = None;
    loop {
        let (playing, speed) = {
            let s = state.lock().unwrap();
            (s.playing, s.speed)
        };
        // commands
        let wait = if playing && pending.is_none() {
            Duration::from_millis(0)
        } else {
            Duration::from_millis(if playing { 2 } else { 50 })
        };
        match cmds.recv_timeout(wait) {
            Ok(cmd) => {
                let mut s = state.lock().unwrap();
                let jump = |s: &mut State, t: f64| {
                    let t = t.clamp(s.start, s.end);
                    s.pos = t;
                    s.ended = false;
                    t
                };
                match cmd {
                    Cmd::Play => s.playing = true,
                    Cmd::Pause => {
                        s.playing = false;
                        target = s.pos; // hold here
                    }
                    Cmd::Toggle => {
                        if s.ended {
                            let st = s.start;
                            target = jump(&mut s, st);
                            reader = None;
                            s.playing = true;
                        } else {
                            s.playing = !s.playing;
                            if !s.playing {
                                target = s.pos;
                            }
                        }
                    }
                    Cmd::Seek(t) => {
                        target = jump(&mut s, t);
                        reader = None;
                    }
                    Cmd::Step(d) => {
                        let p = s.pos + d;
                        target = jump(&mut s, p);
                        reader = None;
                    }
                    Cmd::Speed(v) => s.speed = v.clamp(0.25, 8.0),
                    Cmd::Stop => return,
                }
                clock = None;
                continue;
            }
            Err(RecvTimeoutError::Disconnected) => return,
            Err(RecvTimeoutError::Timeout) => {}
        }
        if reader.is_none() {
            match session::open_rxlog_at(&path, &idx, target - WARM_S) {
                Ok(r) => reader = Some(r.lines()),
                Err(e) => {
                    state.lock().unwrap().error = e.to_string();
                    return;
                }
            }
            pending = None;
            fresh = true;
            clock = None;
        }
        // the next line
        if pending.is_none() {
            match reader.as_mut().unwrap().next() {
                Some(Ok(l)) => {
                    if let Some((t, text)) = session::line_time(&l) {
                        pending = Some((t, text.to_string()));
                    }
                    continue;
                }
                _ => {
                    let mut s = state.lock().unwrap();
                    s.playing = false;
                    s.ended = true;
                    s.pos = s.end;
                    continue;
                }
            }
        }
        let (t, text) = pending.take().unwrap();
        let warming = t < target;
        if !playing {
            // paused: still run through to the target (after a seek: warm-up, then its picture), then hold
            if t >= target + 0.2 {
                pending = Some((t, text));
                std::thread::sleep(Duration::from_millis(20));
                continue;
            }
            if !warming {
                state.lock().unwrap().pos = t;
            }
            sink(&text, t, !warming, fresh);
            fresh = false;
            continue;
        }
        if !warming {
            // paced: line time t is due at anchor_wall + (t - anchor_t) / speed
            let (w0, t0) = *clock.get_or_insert((Instant::now(), t));
            let due = w0 + Duration::from_secs_f64(((t - t0) / speed).max(0.0));
            let now = Instant::now();
            if due > now {
                let d = due - now;
                if d > Duration::from_millis(3) {
                    // not yet: keep it, and wait (in short steps, to stay responsive to commands)
                    std::thread::sleep(d.min(Duration::from_millis(20)));
                    pending = Some((t, text));
                    continue;
                }
                std::thread::sleep(d);
            }
            state.lock().unwrap().pos = t;
        }
        sink(&text, t, !warming, fresh);
        fresh = false;
    }
}
