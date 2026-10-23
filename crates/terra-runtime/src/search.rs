use crate::*;
pub fn dispatch_search(
    b: &mut Runtime,
    kind: &str,
    bytes: &[u8],
    timestamp: f64,
) -> Result<(), RuntimeError> {
    let pose = b.pose.unwrap_or_default();
    let map = b.map.clone();
    let e = b.estimator.estimate(timestamp);
    b.autonomy.run_id = b.run_id.clone();
    let input = terra_autonomy::ArbiterInput {
        now: timestamp,
        pose: pose.0,
        pose_time: pose.1,
        map: map.as_ref().map(|m| &m.0),
        map_time: map.as_ref().map(|m| m.1),
        map_revision: map.as_ref().map(|m| m.2).unwrap_or(0),
        measured: terra_navigation::Twist {
            linear: e.forward,
            angular: e.yaw_rate,
        },
        healthy: e.health == Health::Ready && b.recording_healthy,
    };
    match kind {
        "search" => b
            .autonomy
            .start_search(
                terra_autonomy::decode_search_request(bytes)
                    .ok_or_else(|| invalid("invalid search request"))?,
                &input,
            )
            .map_err(invalid),
        "search/action" => b
            .autonomy
            .apply_search_action(
                terra_autonomy::decode_search_action(bytes)
                    .ok_or_else(|| invalid("invalid search action"))?,
                &input,
            )
            .map_err(invalid),
        "search/report/ack" => {
            let r = terra_autonomy::decode_search_report_ack(bytes)
                .ok_or_else(|| invalid("invalid report acknowledgement"))?;
            if b.autonomy.ack_search_report(r) {
                Ok(())
            } else {
                Err(invalid("unknown report"))
            }
        }
        _ => Err(invalid("unsupported search request")),
    }
}
