//! Tilt and flip hold at L0 (Terra issue #112): a rover that climbs a board
//! and flips must stop its wheels even under manual drive.
use serde_json::json;
use terra_runtime::*;
use terra_types::*;

fn pitched(degrees: f64) -> Quaternion {
    let h = degrees.to_radians() / 2.;
    Quaternion {
        x: 0.,
        y: h.sin(),
        z: 0.,
        w: h.cos(),
    }
}

fn vio(r: &mut Runtime, t: f64, pitch: f64) {
    r.ingest(RuntimeEvent::Vio {
        sample: VioSample {
            timestamp: t,
            position: Vector3::ZERO,
            orientation: pitched(pitch),
            velocity: Vector3::ZERO,
            tracked: true,
        },
        received_at: t,
    })
    .unwrap();
}

fn drive(r: &mut Runtime, t: f64) -> RuntimeOutput {
    r.ingest(RuntimeEvent::LocalManual {
        forward: 0.4,
        turn: 0.,
        received_at: t,
    })
    .unwrap();
    r.step(t).unwrap()
}

fn safety(r: &mut Runtime, action: &str, token: &str, t: f64) {
    let mut v = json!({"action": action, "token": token});
    v["run_id"] = json!(r.run_id());
    v["authority_revision"] = json!(r.autonomy.authority_revision());
    r.ingest(RuntimeEvent::Remote {
        kind: "safety".into(),
        payload: serde_json::to_vec(&v).unwrap(),
        received_at: t,
    })
    .unwrap();
}

#[test]
fn manual_drive_stops_when_the_rover_tips_over_and_waits_for_reset() {
    let mut r = Runtime::new(
        RuntimeConfig {
            mode: ActuationMode::DirectEffort,
            ..Default::default()
        },
        "run".into(),
    )
    .unwrap();
    vio(&mut r, 0., 0.);
    assert!(drive(&mut r, 0.).motor.left > 0., "drives on the flat");

    // Climbing the board: past the limit, but not yet for the debounce time.
    vio(&mut r, 1.0, 45.);
    assert!(drive(&mut r, 1.1).motor.left > 0.);
    // Still tilted after the debounce: the hold latches at L0.
    vio(&mut r, 1.3, 70.);
    let out = drive(&mut r, 1.3);
    assert_eq!(out.motor.left, 0.);
    assert_eq!(out.motor.right, 0.);
    assert_eq!(
        out.autonomy.status.safety,
        terra_autonomy::Safety::EmergencyStop
    );
    assert_eq!(out.autonomy.status.reason, "tilt_stop");
    assert!(out.events.iter().any(|e| e["kind"] == "safety.tilt_stop"));
    assert_eq!(out.status["runtime"]["tilt"]["latched"], true);

    // Upside down: a reset is refused while it is still tilted.
    vio(&mut r, 2.0, 180.);
    safety(&mut r, "reset", "reset-while-flipped", 2.0);
    let out = drive(&mut r, 2.0);
    assert_eq!(out.motor.left, 0.);
    assert_eq!(out.autonomy.status.reason, "tilt_stop");
    // The refusal names the cause, so the dashboard can say "right the rover".
    assert_eq!(
        out.autonomy.status.request_reason.as_deref(),
        Some("still_tilted")
    );
    assert_eq!(out.status["runtime"]["tilt"]["still_tilted"], true);

    // Set back on its wheels: still held until the operator resets.
    vio(&mut r, 5.0, 1.);
    assert_eq!(drive(&mut r, 5.0).motor.left, 0.);
    safety(&mut r, "reset", "reset-upright", 5.1);
    let out = drive(&mut r, 5.1);
    assert_eq!(out.autonomy.status.safety, terra_autonomy::Safety::Clear);
    // The reset drops the intent it arrived with, like an operator-stop reset.
    let out = drive(&mut r, 5.2);
    assert!(out.motor.left > 0., "drives again after the reset");
    assert_eq!(out.status["runtime"]["tilt"]["latched"], false);
}

#[test]
fn a_new_run_keeps_the_tilt_hold() {
    let mut r = Runtime::new(
        RuntimeConfig {
            mode: ActuationMode::DirectEffort,
            ..Default::default()
        },
        "run".into(),
    )
    .unwrap();
    vio(&mut r, 0., 0.);
    vio(&mut r, 1.0, 90.);
    assert_eq!(drive(&mut r, 1.5).motor.left, 0.);
    r.begin_run("run-2".into()).unwrap();
    let out = drive(&mut r, 1.6);
    assert_eq!(out.motor.left, 0.);
    assert_eq!(out.autonomy.status.reason, "tilt_stop");
}
