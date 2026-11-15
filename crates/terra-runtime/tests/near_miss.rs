//! The runtime measures observed clearance each tick and logs one near-miss
//! event per episode edge, so every host records the same episodes.
use terra_mapping::MapSnapshot;
use terra_runtime::*;
use terra_types::*;

fn pose(r: &mut Runtime, t: f64) {
    r.ingest(RuntimeEvent::Vio {
        sample: VioSample {
            timestamp: t,
            position: Vector3::ZERO,
            orientation: Quaternion::default(),
            velocity: Vector3::ZERO,
            tracked: true,
        },
        received_at: t,
    })
    .unwrap();
}

/// Free 4 m square around the origin with one obstacle `ahead` metres along +X.
fn map(r: &mut Runtime, t: f64, ahead: f64) {
    let resolution = 0.05;
    let mut snapshot = MapSnapshot {
        width: 80,
        height: 80,
        resolution,
        origin_x: -2.,
        origin_y: -2.,
        occupancy: vec![0; 80 * 80],
    };
    let col = ((ahead + 2.) / resolution) as usize;
    snapshot.occupancy[40 * 80 + col] = 100;
    r.ingest(RuntimeEvent::Map {
        snapshot,
        timestamp: t,
        sequence: (t * 10.) as u64,
        received_at: t,
    })
    .unwrap();
}

fn near_miss_kinds(output: &RuntimeOutput) -> Vec<String> {
    output
        .autonomy
        .events
        .iter()
        .filter(|e| e.reason == near_miss::REASON)
        .map(|e| e.kind.clone())
        .collect()
}

#[test]
fn clearance_episodes_become_decision_events() {
    let mut r = Runtime::new(RuntimeConfig::default(), "run".into()).unwrap();
    map(&mut r, 0., 0.5);
    let out = r.step(0.).unwrap();
    assert_eq!(out.clearance, None, "no pose, no clearance");
    assert!(near_miss_kinds(&out).is_empty());

    pose(&mut r, 0.1);
    let out = r.step(0.1).unwrap();
    let close = out.clearance.expect("obstacle observed");
    assert!(close < 0.3, "{close}");
    assert_eq!(near_miss_kinds(&out), ["near_miss_start"]);
    let out = r.step(0.15).unwrap();
    assert!(near_miss_kinds(&out).is_empty(), "one event per episode");

    map(&mut r, 0.2, 1.5);
    pose(&mut r, 0.2);
    let out = r.step(0.2).unwrap();
    assert!(out.clearance.unwrap() > 0.4);
    assert_eq!(near_miss_kinds(&out), ["near_miss_end"]);
}
