use terra_types::point_capture::*;

fn frame() -> CaptureFrame {
    CaptureFrame {
        rover_id: 7,
        request_id: "cap-1".into(),
        index: 2,
        count: 10,
        timestamp: 12.5,
        width: 3,
        height: 2,
        camera_position: [1.0, 2.0, 0.4],
        camera_orientation: [0.0, 0.0, 0.0, 1.0],
        intrinsics: [200.0, 201.0, 1.0, 0.5],
        classifier: CaptureClassifier::GroundPlane {
            normal: [0.0, 0.0, 1.0],
            offset: -0.1,
        },
        min_obstacle_height: 0.15,
        max_obstacle_height: 2.0,
        max_range: 15.0,
        points: PointLabel::ALL
            .iter()
            .enumerate()
            .map(|(i, &label)| CapturePoint {
                position: if label == PointLabel::NoReturn {
                    [f32::NAN; 3]
                } else {
                    [i as f32, -(i as f32), 0.25]
                },
                label,
            })
            .collect(),
    }
}

/// Every header field and label survives a round trip, and the size is the
/// documented header plus 13 bytes per pixel.
#[test]
fn frame_round_trips() {
    let original = frame();
    let bytes = original.encode().unwrap();
    assert_eq!(&bytes[..4], b"TPC1");
    assert_eq!(bytes.len(), 4 + 8 + 1 + 5 + 4 + 8 + 8 + 8 * 18 + 1 + 6 * 13);
    let decoded = CaptureFrame::decode(&bytes).unwrap();
    assert_eq!(decoded.points[5].label, PointLabel::NoReturn);
    assert!(decoded.points[5].position[0].is_nan());
    let mut comparable = decoded.clone();
    comparable.points[5].position = [0.0; 3];
    let mut expected = original;
    expected.points[5].position = [0.0; 3];
    assert_eq!(comparable, expected);

    let mut band = frame();
    band.classifier = CaptureClassifier::HorizonBand {
        half_angle: 0.17,
        max_range: 3.4,
    };
    let decoded = CaptureFrame::decode(&band.encode().unwrap()).unwrap();
    assert_eq!(decoded.classifier, band.classifier);
}

/// A 256 × 192 LiDAR frame, the phone's depth size, is about 640 KB.
#[test]
fn phone_depth_frame_is_about_640_kb() {
    let mut f = frame();
    f.width = 256;
    f.height = 192;
    f.points = vec![
        CapturePoint {
            position: [0.0; 3],
            label: PointLabel::Free,
        };
        256 * 192
    ];
    let size = f.encode().unwrap().len();
    assert!((630_000..650_000).contains(&size), "{size}");
}

/// Truncated, padded, foreign and malformed frames are rejected.
#[test]
fn malformed_frames_are_rejected() {
    let bytes = frame().encode().unwrap();
    assert_eq!(
        CaptureFrame::decode(&bytes[..bytes.len() - 1]),
        Err(FrameError::Length)
    );
    let mut padded = bytes.clone();
    padded.push(0);
    assert_eq!(CaptureFrame::decode(&padded), Err(FrameError::Length));
    let mut foreign = bytes.clone();
    foreign[3] = b'2';
    assert_eq!(CaptureFrame::decode(&foreign), Err(FrameError::Magic));
    let mut label = bytes.clone();
    *label.last_mut().unwrap() = 9;
    assert_eq!(CaptureFrame::decode(&label), Err(FrameError::Field));
    let mut short = frame();
    short.points.pop();
    assert_eq!(short.encode(), Err(FrameError::Field));
    let mut id = frame();
    id.request_id = "has space".into();
    assert_eq!(id.encode(), Err(FrameError::Field));
}

/// Defaults are 10 s at 1 Hz; out-of-range or malformed requests are refused.
#[test]
fn requests_default_and_validate() {
    let request = CaptureRequest::decode(br#"{"version":1,"request_id":"a_1"}"#).unwrap();
    assert_eq!(request, CaptureRequest::new("a_1"));
    assert_eq!(request.frame_count(), 10);
    let encoded = serde_json::to_vec(&request).unwrap();
    assert_eq!(CaptureRequest::decode(&encoded), Some(request));
    for bad in [
        r#"{"version":2,"request_id":"a"}"#,
        r#"{"version":1,"request_id":""}"#,
        r#"{"version":1,"request_id":"a/b"}"#,
        r#"{"version":1,"request_id":"a","duration_seconds":0}"#,
        r#"{"version":1,"request_id":"a","duration_seconds":61}"#,
        r#"{"version":1,"request_id":"a","rate_hz":3}"#,
        r#"not json"#,
    ] {
        assert_eq!(CaptureRequest::decode(bad.as_bytes()), None, "{bad}");
    }
    assert_eq!(CaptureRequest::decode(&vec![b' '; 4096]), None);
    let short = CaptureRequest {
        duration_seconds: 0.2,
        ..CaptureRequest::new("x")
    };
    assert_eq!(short.frame_count(), 1);
}

/// Live requests stream with a lease, a faster rate cap and a pixel stride.
#[test]
fn live_requests_validate() {
    let request = CaptureRequest::decode(
        br#"{"version":1,"request_id":"live","live":true,"rate_hz":4,"stride":3}"#,
    )
    .unwrap();
    assert!(request.live);
    assert_eq!(request.stride, 3);
    assert_eq!(request.frame_count(), 0);
    assert_eq!(request.duration_seconds, 10.0);
    let built = CaptureRequest::live("live", 4.0, 3, 10.0);
    assert_eq!(built, request);
    assert_eq!(
        CaptureRequest::decode(&serde_json::to_vec(&built).unwrap()),
        Some(built)
    );
    for bad in [
        r#"{"version":1,"request_id":"a","live":true,"rate_hz":6}"#,
        r#"{"version":1,"request_id":"a","rate_hz":4,"stride":2}"#,
        r#"{"version":1,"request_id":"a","stride":0}"#,
        r#"{"version":1,"request_id":"a","stride":9}"#,
    ] {
        assert_eq!(CaptureRequest::decode(bad.as_bytes()), None, "{bad}");
    }
}

/// Progress serializes with snake-case states.
#[test]
fn progress_json_is_stable() {
    let progress = CaptureProgress {
        version: 1,
        rover_id: 7,
        request_id: "a".into(),
        state: CaptureState::Running,
        frames_sent: 1,
        frames_total: 10,
        detail: String::new(),
    };
    let json = serde_json::to_value(&progress).unwrap();
    assert_eq!(json["state"], "running");
    assert_eq!(
        serde_json::from_value::<CaptureProgress>(json).unwrap(),
        progress
    );
}
