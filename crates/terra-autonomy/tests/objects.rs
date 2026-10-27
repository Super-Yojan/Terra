use terra_autonomy::objects::*;
use terra_autonomy::outdoor::{GpsFix, HeadingFix, OutdoorController, distance_bearing};
use terra_navigation::Pose;

fn sighting(frame: u64, x: f64, y: f64) -> ObjectSighting {
    ObjectSighting {
        class: "bucket".into(),
        color: Some("orange".into()),
        confidence: 0.9,
        frame_id: frame,
        timestamp: frame as f64 * 0.2,
        x,
        y,
        geo: None,
    }
}

#[test]
fn three_frames_near_each_other_confirm_one_bucket() {
    let mut m = ObjectMap::default();
    m.observe(sighting(1, 2., 1.)).unwrap();
    m.observe(sighting(2, 2.2, 1.)).unwrap();
    assert!(!m.objects()[0].confirmed);
    m.observe(sighting(3, 2.1, 1.3)).unwrap();
    assert_eq!(m.objects().len(), 1);
    let o = &m.objects()[0];
    assert!(o.confirmed);
    assert_eq!(o.sightings, 3);
    assert!((o.x - 2.1).abs() < 1e-9 && (o.y - 1.1).abs() < 1e-9);
}

#[test]
fn one_frame_counts_once_and_far_or_other_colour_is_new() {
    let mut m = ObjectMap::default();
    m.observe(sighting(1, 0., 0.)).unwrap();
    m.observe(sighting(1, 0.1, 0.)).unwrap();
    assert_eq!(m.objects()[0].sightings, 1);
    m.observe(sighting(2, 3., 0.)).unwrap();
    let mut blue = sighting(3, 0., 0.);
    blue.color = Some("blue".into());
    m.observe(blue).unwrap();
    assert_eq!(m.objects().len(), 3);
}

#[test]
fn rejects_bad_input_and_low_confidence() {
    let mut m = ObjectMap::default();
    let mut s = sighting(1, 0., 0.);
    s.class = "blue bucket".into();
    assert!(m.observe(s).is_err());
    let mut s = sighting(1, f64::NAN, 0.);
    assert!(m.observe(s.clone()).is_err());
    s.x = 0.;
    s.confidence = 0.3;
    assert!(m.observe(s).is_err());
    assert!(m.objects().is_empty());
    assert_eq!(m.revision, 0);
}

#[test]
fn keeps_confirmed_objects_when_full() {
    let mut m = ObjectMap::default();
    for f in 1..=3 {
        m.observe(sighting(f, 0., 0.)).unwrap();
    }
    for i in 0..MAX_OBJECTS as u64 {
        m.observe(sighting(10 + i, 5. + 2. * i as f64, 0.)).unwrap();
    }
    assert_eq!(m.objects().len(), MAX_OBJECTS);
    assert!(m.objects()[0].confirmed);
}

#[test]
fn geo_is_averaged_and_serialized() {
    let mut m = ObjectMap::default();
    let mut a = sighting(1, 0., 0.);
    a.geo = Some((38.0, -77.0, 4.));
    let mut b = sighting(2, 0., 0.);
    b.geo = Some((38.00002, -77.00002, 3.));
    m.observe(a).unwrap();
    m.observe(b).unwrap();
    let o = &m.objects()[0];
    assert!((o.latitude.unwrap() - 38.00001).abs() < 1e-9);
    assert_eq!(o.geo_accuracy_m, Some(3.));
    let t = m.telemetry(4, "run");
    assert_eq!(t["objects"][0]["class"], "bucket");
    assert_eq!(t["objects"][0]["color"], "orange");
    assert!(t["objects"][0].get("last_frame").is_none());
}

fn outdoor(heading: f64, gps_accuracy: f64) -> OutdoorController {
    let mut c = OutdoorController::default();
    c.observe_pose(
        Pose {
            x: 0.,
            y: 0.,
            yaw: 0.,
        },
        1.,
    );
    c.gps(
        GpsFix {
            timestamp: 1.,
            latitude: 38.83,
            longitude: -77.3,
            accuracy: gps_accuracy,
        },
        1.,
    )
    .unwrap();
    c.heading(
        HeadingFix {
            timestamp: 1.,
            true_heading: heading,
            accuracy: 3.,
        },
        1.,
    )
    .unwrap();
    c
}

#[test]
fn geolocates_points_ahead_of_the_rover() {
    // Rover yaw 0 faces east (true heading 90°); a point 5 m ahead is 5 m east.
    let c = outdoor(90., 2.);
    let here = Pose {
        x: 0.,
        y: 0.,
        yaw: 0.,
    };
    let (lat, lon, acc) = c.geolocate(1., here, 5., 0.).unwrap();
    let (d, b) = distance_bearing(38.83, -77.3, lat, lon);
    assert!((d - 5.).abs() < 0.01, "{d}");
    assert!((b - 90.).abs() < 0.1, "{b}");
    assert!(acc > 2. && acc < 2.5);
    // Local +y is left of forward, so north when facing east.
    let (lat, lon, _) = c.geolocate(1., here, 0., 3.).unwrap();
    let (d, b) = distance_bearing(38.83, -77.3, lat, lon);
    assert!((d - 3.).abs() < 0.01 && b.min(360. - b) < 0.1, "{d} {b}");
}

#[test]
fn no_geolocation_indoors_or_when_stale() {
    let here = Pose {
        x: 0.,
        y: 0.,
        yaw: 0.,
    };
    assert!(outdoor(90., 25.).geolocate(1., here, 5., 0.).is_none());
    assert!(outdoor(90., 2.).geolocate(5., here, 5., 0.).is_none());
    assert!(outdoor(90., 2.).geolocate(1., here, 50., 0.).is_none());
    assert!(
        OutdoorController::default()
            .geolocate(1., here, 5., 0.)
            .is_none()
    );
}
