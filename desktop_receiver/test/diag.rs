//! Unit tests of src/diag.rs (diagnose(): which message the video area shows for which link state): compiled into that module by `cargo test`.
use super::*;
use crate::engine::AIR_VIDEO;

fn base() -> Inputs {
    Inputs {
        now: 1000.0,
        started: 900.0,
        ground: true,
        last_picture: 999.0,
        last_heard: 999.9,
        last_video: 999.9,
        freq_hz: Some(868_000_000),
        lq_pct: Some(100.0),
        ..Default::default()
    }
}
fn air(state: u8, error: u8) -> Option<(f64, AirStatus)> {
    Some((
        999.8,
        AirStatus {
            state,
            since_video: Some(12),
            since_host: Some(7),
            error,
            up_s: 45,
        },
    ))
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
