use serde_json::json;
use terra_runtime::*;
use terra_types::*;
/// Deterministic portable trace: two actuation modes, turning/reversal, expiry,
/// stale generation, stop dominance and router replacement.
pub fn trace() -> Vec<f64> {
    let mut values = Vec::new();
    for mode in [ActuationMode::DirectEffort, ActuationMode::VelocityFeedback] {
        let mut r = Runtime::new(
            RuntimeConfig {
                mode,
                ..Default::default()
            },
            "parity-run".into(),
        )
        .unwrap();
        for tick in 0..100 {
            let t = tick as f64 * 0.01;
            if tick < 75 {
                r.ingest(RuntimeEvent::Imu {
                    sample: ImuSample {
                        timestamp: t,
                        acceleration: Vector3::ZERO,
                        angular_velocity: Vector3::ZERO,
                    },
                    received_at: t,
                })
                .unwrap();
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
            if tick < 25 {
                r.ingest(RuntimeEvent::LocalManual {
                    forward: 0.5,
                    turn: 0.2,
                    received_at: t,
                })
                .unwrap();
            }
            if tick == 25 {
                r.ingest(RuntimeEvent::LocalManual {
                    forward: -0.5,
                    turn: 0.2,
                    received_at: t,
                })
                .unwrap();
            }
            if tick == 40 {
                r.link(false);
            }
            if tick == 45 {
                r.link(true);
            }
            if tick == 60 || tick == 90 {
                let payload = json!({"action":if tick==60{"stop"}else{"reset"},"token":format!("safety-{tick}"),"run_id":r.run_id,"authority_revision":r.autonomy.authority_revision()});
                r.ingest(RuntimeEvent::Remote {
                    kind: "safety".into(),
                    payload: serde_json::to_vec(&payload).unwrap(),
                    received_at: t,
                })
                .unwrap();
            }
            let out = r.step(t).unwrap();
            values.extend([
                out.motor.left,
                out.motor.right,
                out.autonomy.twist.linear,
                out.autonomy.twist.angular,
            ]);
            // Discrete status must match exactly; small digest fits exactly in f64.
            let status = serde_json::to_vec(&out.autonomy.status).unwrap();
            let hash = status.iter().fold(2166136261u32, |h, b| {
                (h ^ u32::from(*b)).wrapping_mul(16777619)
            });
            values.push(f64::from(hash));
        }
    }
    values
}
