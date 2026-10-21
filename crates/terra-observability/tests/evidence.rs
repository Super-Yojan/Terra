use serde_json::json;
use std::{sync::Arc, time::Duration};
use terra_observability::*;
fn bundle() -> EvidenceBundle {
    EvidenceBundle {
        event: EventInput::new("4", "mapping", "mapping.evidence", json!({})),
        manifest: json!({"exposure_ns":1_000,"rgb_exposure_ns":1_000,"frame_id":"frame-A","depth_intrinsics":{"fx":12,"fy":12,"cx":0.5,"cy":0.5,"width":2,"height":2},"camera_to_world":{"position":[0,0,0.5],"quaternion_xyzw":[0,0,0,1]},"ground_plane":{"normal":[0,0,1],"offset":0},"thresholds":{"min_obstacle_height":0.15,"max_obstacle_height":2.0}}),
        attachments: vec![
            AttachmentInput {
                name: "depth".into(),
                data: vec![0; 16],
                encoding: "f32le".into(),
                dimensions: vec![2, 2],
                units: "metres".into(),
            },
            AttachmentInput {
                name: "rgb".into(),
                data: vec![1; 16],
                encoding: "rgba8".into(),
                dimensions: vec![2, 2],
                units: "rgba".into(),
            },
        ],
    }
}
#[test]
fn exact_mapper_inputs_survive_capture() {
    let d = tempfile::tempdir().unwrap();
    let j = Arc::new(
        Journal::open(JournalConfig {
            path: d.path().join("events.db"),
            ..Default::default()
        })
        .unwrap(),
    );
    let w = EvidenceWriter::open(d.path().join("evidence"), j.clone(), 512 * 1024).unwrap();
    let b = bundle();
    assert_eq!(w.try_capture(b.clone()), CaptureOutcome::Queued);
    w.flush(Duration::from_secs(2)).unwrap();
    j.flush(Duration::from_secs(2)).unwrap();
    let e = JournalReader::open(&d.path().join("events.db"))
        .unwrap()
        .export()
        .unwrap();
    let manifest = &e[0].input.attributes;
    assert_eq!(manifest["ground_plane"], b.manifest["ground_plane"]);
    assert_eq!(manifest["camera_to_world"], b.manifest["camera_to_world"]);
    assert_eq!(manifest["attachments"].as_array().unwrap().len(), 2);
}
#[test]
fn mismatched_rgb_is_marked_missing() {
    let d = tempfile::tempdir().unwrap();
    let j = Arc::new(
        Journal::open(JournalConfig {
            path: d.path().join("db"),
            ..Default::default()
        })
        .unwrap(),
    );
    let w = EvidenceWriter::open(d.path().join("evidence"), j.clone(), 512 * 1024).unwrap();
    let mut b = bundle();
    b.manifest["rgb_exposure_ns"] = json!(2000);
    w.try_capture(b);
    w.flush(Duration::from_secs(2)).unwrap();
    j.flush(Duration::from_secs(2)).unwrap();
    let e = JournalReader::open(&d.path().join("db"))
        .unwrap()
        .export()
        .unwrap();
    assert_eq!(
        e[0].input.attributes["rgb_missing_reason"],
        "exposure_mismatch"
    );
    assert_eq!(
        e[0].input.attributes["attachments"]
            .as_array()
            .unwrap()
            .len(),
        1
    );
}
#[test]
fn quota_loss_produces_gap() {
    let d = tempfile::tempdir().unwrap();
    let j = Arc::new(
        Journal::open(JournalConfig {
            path: d.path().join("db"),
            ..Default::default()
        })
        .unwrap(),
    );
    let w = EvidenceWriter::open(d.path().join("evidence"), j.clone(), 1).unwrap();
    w.try_capture(bundle());
    w.flush(Duration::from_secs(2)).unwrap();
    j.flush(Duration::from_secs(2)).unwrap();
    assert!(
        JournalReader::open(&d.path().join("db"))
            .unwrap()
            .export()
            .unwrap()
            .iter()
            .any(|e| e.input.kind == "observability.gap")
    );
}
