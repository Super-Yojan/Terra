//! Sensor pushes are queued and applied in timestamp order when the controller
//! next locks its state; a rejected sample is counted, not fatal.
use terra_mobile::{ImuReading, MobileController, default_control_settings};

fn imu(t: f64) -> ImuReading {
    ImuReading {
        timestamp: t,
        acceleration_forward: 0.,
        acceleration_left: 0.,
        acceleration_up: 0.,
        gyro_roll: 0.,
        gyro_pitch: 0.,
        gyro_yaw: 0.,
    }
}

#[test]
fn pushes_out_of_order_within_a_tick_are_sorted_before_they_are_applied() {
    let c = MobileController::new(default_control_settings()).unwrap();
    c.push_imu(imu(1.02)).unwrap();
    c.push_imu(imu(1.00)).unwrap();
    c.push_imu(imu(1.01)).unwrap();
    assert_eq!(c.sensor_queue_stats().imu_queued, 3);
    c.step(1.03).unwrap();
    let stats = c.sensor_queue_stats();
    assert_eq!(stats.imu_queued, 0);
    assert_eq!(stats.rejected, 0, "{:?}", stats.last_rejection);
}

#[test]
fn a_sample_older_than_one_already_applied_is_dropped_and_counted() {
    let c = MobileController::new(default_control_settings()).unwrap();
    c.push_imu(imu(2.0)).unwrap();
    c.step(2.0).unwrap();
    c.push_imu(imu(1.5)).unwrap();
    // The stale sample must not stop the controller.
    c.step(2.01).unwrap();
    let stats = c.sensor_queue_stats();
    assert_eq!(stats.rejected, 1);
    assert!(stats.last_rejection.is_some());
}

#[test]
fn reset_discards_queued_samples() {
    let c = MobileController::new(default_control_settings()).unwrap();
    c.push_imu(imu(1.0)).unwrap();
    c.reset().unwrap();
    c.push_imu(imu(0.5)).unwrap();
    c.step(0.6).unwrap();
    assert_eq!(c.sensor_queue_stats().rejected, 0);
}

#[test]
fn a_nonfinite_timestamp_is_refused_at_push() {
    let c = MobileController::new(default_control_settings()).unwrap();
    assert!(c.push_imu(imu(f64::NAN)).is_err());
    assert_eq!(c.sensor_queue_stats().imu_queued, 0);
}
