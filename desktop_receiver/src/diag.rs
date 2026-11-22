//! "Why is there no picture?": when no new picture has come for NO_PICTURE_S, the best explanation from what the Ground
//! hears, shown over the video (render.rs draw_notice). The Air status (engine::AirStatus, in the TX radio's idle
//! packets) says what the Air itself knows: its camera computer silent, no camera, the transmitter stopped.
use crate::engine::{AirStatus, AIR_ERROR, AIR_NO_HOST, AIR_NO_VIDEO, AIR_SILENT, AIR_STOPPING};

/// No new picture for this long: the notice appears.
pub const NO_PICTURE_S: f64 = 0.5;
/// While a link is being established (the app just started, or packets just started arriving again after a gap) the
/// first picture takes a few seconds: no notice during that time.
pub const LINK_GRACE_S: f64 = 5.0;

/// What the diagnosis looks at. Times are unix seconds; 0 = never.
#[derive(Clone, Debug, Default)]
pub struct Inputs {
    pub now: f64,
    /// when the app started (the first picture is waited for from then)
    pub started: f64,
    /// a Ground is open
    pub ground: bool,
    /// why not (the serial thread's text), if it said
    pub ground_error: String,
    /// the newest live picture
    pub last_picture: f64,
    /// the newest packet of any kind, damaged ones included
    pub last_heard: f64,
    /// when packets started arriving after nothing for a second or more (the link came up): 0 = never
    pub link_since: f64,
    /// the newest video packet
    pub last_video: f64,
    /// the newest Air status, and when it came
    pub air: Option<(f64, AirStatus)>,
    /// the newest packet from another system (keyed with another link ID), and that link
    pub foreign: Option<(f64, u16)>,
    /// the frequency the Ground listens on
    pub freq_hz: Option<u64>,
    /// link quality (share of packets received, %) and SNR over the last seconds
    pub lq_pct: Option<f64>,
    pub snr_db: Option<f64>,
}

/// "12 s", "3 min", "2 h".
fn dur(s: f64) -> String {
    let s = s.max(0.0);
    if s < 90.0 { format!("{:.0} s", s) } else if s < 5400.0 { format!("{:.0} min", s / 60.0) } else { format!("{:.0} h", s / 3600.0) }
}

/// (title, explanation), or None while pictures come.
pub fn diagnose(d: &Inputs) -> Option<(String, String)> {
    let since = |t: f64| d.now - t;
    let waited = if d.last_picture > 0.0 { since(d.last_picture) } else { since(d.started) };
    if waited < NO_PICTURE_S {
        return None;
    }
    // establishing the link: give the first picture time to come
    if since(d.started) < LINK_GRACE_S
        || (d.link_since > 0.0 && since(d.link_since) < LINK_GRACE_S && d.last_picture < d.link_since)
    {
        return None;
    }
    let n = |t: &str, x: String| Some((t.to_string(), x));
    if !d.ground {
        let why = if d.ground_error.is_empty() || d.ground_error.starts_with("No receiver: plug") {
            "Plug the Horizon Dev Ground into this computer by USB.".to_string()
        } else {
            d.ground_error.clone()
        };
        return n("NO RECEIVER", why);
    }
    let freq = d.freq_hz.map(|f| format!("{:.3} MHz", f as f64 / 1e6)).unwrap_or_else(|| "its frequency".into());
    // nothing at all from the Air
    if d.last_heard <= 0.0 || since(d.last_heard) >= 1.0 {
        if let Some((t, a)) = d.air {
            if a.state == AIR_STOPPING && since(t) < 600.0 {
                return n("AIR UNIT TRANSMITTER STOPPED", format!(
                    "The Air unit stopped transmitting {} ago on purpose: its transmitter was stopped or the Air unit was shut down. \
                     The picture returns when its transmitter starts again.", dur(since(t))));
            }
        }
        if d.last_heard <= 0.0 {
            return n("WAITING FOR THE AIR UNIT", format!(
                "Nothing heard on {freq} yet. Is the Air unit powered? It starts sending about 30 s after power-up. \
                 Both boards must be paired to the same frequency (setup site, Pair)."));
        }
        return n("NOTHING HEARD FROM THE AIR UNIT", format!(
            "Nothing received for {}: out of range or blocked, or the Air unit lost power or restarted. Listening on {freq}.",
            dur(since(d.last_heard))));
    }
    // intact packets keyed with another link ID, and none of ours: the Air unit heard is not paired with this Ground
    if let Some((t, link)) = d.foreign {
        if since(t) < 3.0 && (d.last_video <= 0.0 || since(d.last_video) >= NO_PICTURE_S) {
            return n("AIR UNIT WITH ANOTHER LINK ID", format!(
                "The Ground hears an Air unit clearly, but it uses link ID {link}, not this Ground's: another system, or \
                 your Air unit with another callsign. Pair both boards again (setup site, Pair) so they share the callsign."));
        }
    }
    let weak = d.lq_pct.is_some_and(|lq| lq < 70.0);
    let weak_text = || {
        let lost = 100.0 - d.lq_pct.unwrap_or(0.0);
        let snr = d.snr_db.map(|s| format!(", SNR {s:.1} dB")).unwrap_or_default();
        format!("{lost:.0} % of packets lost{snr}: too few arrive to rebuild pictures. Move closer, raise the antennas, \
                 or turn on FEC.")
    };
    // the Air is heard but sends no video: its own status says why
    if d.last_video <= 0.0 || since(d.last_video) >= NO_PICTURE_S {
        match d.air.filter(|(t, _)| since(*t) < 3.0).map(|(_, a)| a) {
            Some(a) if a.state == AIR_ERROR => {
                let (t, x) = match a.error {
                    1 => ("AIR UNIT: NO CAMERA", "The Air unit's camera computer finds no camera. Check that the camera's ribbon cable is \
                          pushed in fully, contacts the right way round, then restart the Air unit."),
                    2 => ("AIR UNIT: CAMERA DOES NOT START", "The Air unit's camera was found but does not start. Restart the Air unit; if it \
                          stays, check the camera module and its cable."),
                    3 => ("AIR UNIT: ENCODER DOES NOT START", "The Air unit's video encoder does not start. Restart the Air unit."),
                    _ => ("AIR UNIT: TRANSMITTER DOES NOT START", "The Air unit's transmitter program fails to start. Restart the Air unit; \
                          the Air unit check on the setup site names the part."),
                };
                return n(t, x.split_whitespace().collect::<Vec<_>>().join(" "));
            }
            Some(a) if a.state == AIR_NO_HOST => {
                return n("AIR UNIT: CAMERA COMPUTER NOT STARTED", format!(
                    "The Air unit's radio has been on for {} but its camera computer has not started sending. It needs about \
                     30 s after power-up; if it stays, check its microSD card.", dur(a.up_s as f64)));
            }
            Some(a) if a.state == AIR_SILENT => {
                let ago = a.since_host.map(|s| format!("{} ago", dur(s as f64))).unwrap_or_else(|| "a while ago".into());
                return n("AIR UNIT: TRANSMITTER STOPPED RESPONDING", format!(
                    "The Air unit's camera computer stopped sending {ago}: its transmitter program stopped or the computer \
                     restarted. It restarts by itself; if it stays, run the Air unit check on the setup site."));
            }
            Some(a) if a.state == AIR_NO_VIDEO => {
                let ago = a.since_video.map(|s| format!(" for {}", dur(s as f64))).unwrap_or_default();
                return n("AIR UNIT: NO PICTURES FROM THE CAMERA", format!(
                    "The Air unit's transmitter runs but has sent no video{ago}: the camera may have stopped. Restart the Air unit if \
                     it stays."));
            }
            Some(a) if a.state == AIR_STOPPING => {
                return n("AIR UNIT TRANSMITTER STOPPING", "The Air unit's transmitter was asked to stop (end of transmission).".into());
            }
            Some(_) => {} // the Air unit sends video: lost on the way (below)
            None if weak => {}
            None => {
                return n("NO VIDEO FROM THE AIR UNIT", "The Air unit's radio is on, but its camera computer sends no video. Most \
                    likely the camera is not connected or not detected, or the camera computer is still starting (about \
                    30 s after power-up). This Air unit's firmware is too old to say which: run the Air unit check on the setup \
                    site, or update the Air unit so it reports the reason here.".split_whitespace().collect::<Vec<_>>().join(" "));
            }
        }
    }
    if weak {
        return n("LINK TOO WEAK", weak_text());
    }
    n("WAITING FOR A KEYFRAME", "Video is arriving; the picture returns with the next keyframe (within about 2 s).".into())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::AIR_VIDEO;

    fn base() -> Inputs {
        Inputs { now: 1000.0, started: 900.0, ground: true, last_picture: 999.0, last_heard: 999.9, last_video: 999.9,
                 freq_hz: Some(868_000_000), lq_pct: Some(100.0), ..Default::default() }
    }
    fn air(state: u8, error: u8) -> Option<(f64, AirStatus)> {
        Some((999.8, AirStatus { state, since_video: Some(12), since_host: Some(7), error, up_s: 45 }))
    }
    fn title(d: &Inputs) -> String {
        diagnose(d).map(|x| x.0).unwrap_or_default()
    }

    #[test]
    fn pictures_coming() {
        let mut d = base();
        d.last_picture = 999.8;
        assert!(diagnose(&d).is_none());
    }
    #[test]
    fn quiet_while_the_link_comes_up() {
        let mut d = base();
        d.last_picture = 0.0;
        d.started = 997.0;
        assert!(diagnose(&d).is_none(), "app started 3 s ago");
        d.started = 900.0;
        d.link_since = 998.0;
        assert!(diagnose(&d).is_none(), "packets since 2 s, no picture since");
        d.link_since = 990.0;
        assert_eq!(title(&d), "WAITING FOR A KEYFRAME");
    }
    #[test]
    fn no_ground() {
        let mut d = base();
        d.ground = false;
        assert_eq!(title(&d), "NO RECEIVER");
    }
    #[test]
    fn nothing_heard() {
        let mut d = base();
        d.last_heard = 990.0;
        assert_eq!(title(&d), "NOTHING HEARD FROM THE AIR UNIT");
        d.last_heard = 0.0;
        assert_eq!(title(&d), "WAITING FOR THE AIR UNIT");
        d.air = air(AIR_STOPPING, 0);
        assert_eq!(title(&d), "AIR UNIT TRANSMITTER STOPPED");
    }
    #[test]
    fn air_reasons() {
        let mut d = base();
        d.last_video = 990.0;
        d.air = air(AIR_ERROR, 1);
        assert_eq!(title(&d), "AIR UNIT: NO CAMERA");
        d.air = air(AIR_SILENT, 0);
        assert!(diagnose(&d).unwrap().1.contains("7 s ago"));
        d.air = air(AIR_NO_HOST, 0);
        assert_eq!(title(&d), "AIR UNIT: CAMERA COMPUTER NOT STARTED");
        d.air = air(AIR_NO_VIDEO, 0);
        assert_eq!(title(&d), "AIR UNIT: NO PICTURES FROM THE CAMERA");
        d.air = None;
        assert_eq!(title(&d), "NO VIDEO FROM THE AIR UNIT");
    }
    #[test]
    fn another_link() {
        let mut d = base();
        d.last_video = 990.0;
        d.lq_pct = Some(0.0);
        d.foreign = Some((999.9, 5489));
        assert_eq!(title(&d), "AIR UNIT WITH ANOTHER LINK ID");
        assert!(diagnose(&d).unwrap().1.contains("5489"));
    }
    #[test]
    fn weak_and_keyframe() {
        let mut d = base();
        d.lq_pct = Some(40.0);
        assert_eq!(title(&d), "LINK TOO WEAK");
        d.lq_pct = Some(99.0);
        assert_eq!(title(&d), "WAITING FOR A KEYFRAME");
        d.last_video = 990.0;
        d.air = air(AIR_VIDEO, 0);
        assert_eq!(title(&d), "WAITING FOR A KEYFRAME");
    }
}
