use terra_control::{TurningConfig, TurningLimiter};
use terra_types::VelocityTarget;
fn target(t: f64, v: f64, w: f64) -> VelocityTarget {
    VelocityTarget {
        timestamp: t,
        forward: v,
        yaw_rate: w,
    }
}
#[test]
fn limits_radius_slew_reversal_and_resets() {
    let mut l = TurningLimiter::new(TurningConfig::default()).unwrap();
    let mut previous = 0.;
    for n in 0..100 {
        let out = l.step(target(n as f64 * 0.05, 1., 5.)).unwrap();
        assert!(out.yaw_rate <= 0.5);
        assert!(out.yaw_rate - previous <= 0.025000001);
        previous = out.yaw_rate;
    }
    let out = l.step(target(5., 0.01, 5.)).unwrap();
    assert!(out.yaw_rate <= 0.02);
    assert_eq!(l.step(target(5.05, 0., 5.)).unwrap().yaw_rate, 0.);
    assert!(l.step(target(5.05, 1., 1.)).is_err());
    assert!(l.step(target(10., 1., 1.)).unwrap().yaw_rate <= 0.00500001);
    l.reset();
    assert!(l.step(target(20., -0.1, 1.)).unwrap().yaw_rate <= 0.00500001);
    let mut l = TurningLimiter::new(TurningConfig {
        allow_in_place: true,
        ..Default::default()
    })
    .unwrap();
    for n in 0..30 {
        l.step(target(n as f64 * 0.05, 0., 1.)).unwrap();
    }
    let x = l.step(target(1.5, 0., -1.)).unwrap();
    assert!(x.yaw_rate > 0.);
}
#[test]
fn no_spin_defaults_and_invalid_values() {
    let c = TurningConfig::default();
    assert_eq!(c.constrain_wheels(-0.5, 0.5), (0., 0.));
    let (l, r) = c.constrain_wheels(-0.1, 0.5);
    assert!(l >= 0. && r <= 0.5);
    let c = TurningConfig {
        allow_in_place: true,
        ..c
    };
    assert_eq!(c.constrain_wheels(-0.5, 0.5), (-0.5, 0.5));
    assert!(
        TurningLimiter::new(TurningConfig {
            min_turn_radius: 0.,
            ..Default::default()
        })
        .is_err()
    );
    let mut l = TurningLimiter::new(c).unwrap();
    assert!(l.step(target(f64::NAN, 1., 1.)).is_err());
    l.step(target(0., 1., 1.)).unwrap();
    assert!(l.step(target(1., 1., 1.)).is_err());
}
