use crate::*;
pub fn dispatch_search(
    b: &mut Runtime,
    kind: &str,
    bytes: &[u8],
    timestamp: f64,
) -> Result<(), RuntimeError> {
    let healthy = b.sensors_ready(timestamp);
    let input = arbiter_input(&b.estimator, b.pose, b.map.as_ref(), timestamp, healthy);
    match kind {
        requests::SEARCH => b
            .autonomy
            .start_search(
                terra_autonomy::decode_search_request(bytes)
                    .ok_or_else(|| invalid("invalid search request"))?,
                &input,
            )
            .map_err(invalid),
        requests::SEARCH_ACTION => b
            .autonomy
            .apply_search_action(
                terra_autonomy::decode_search_action(bytes)
                    .ok_or_else(|| invalid("invalid search action"))?,
                &input,
            )
            .map_err(invalid),
        requests::SEARCH_REPORT_ACK => {
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
