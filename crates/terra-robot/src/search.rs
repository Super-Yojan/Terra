use crate::*;

impl Runtime {
    /// Start a target search against the current pose, map, detector and authority.
    pub fn start_search(
        &mut self,
        request: terra_autonomy::SearchRequest,
        now: f64,
    ) -> Result<(), RuntimeError> {
        let healthy = self.sensors_ready(now);
        let input = arbiter_input(&self.estimator, self.pose, self.map.as_ref(), now, healthy);
        self.autonomy.start_search(request, &input).map_err(invalid)
    }
}

pub(crate) fn dispatch_search(
    b: &mut Runtime,
    kind: &str,
    bytes: &[u8],
    timestamp: f64,
) -> Result<(), RuntimeError> {
    match kind {
        requests::SEARCH => b.start_search(
            terra_autonomy::decode_search_request(bytes)
                .ok_or_else(|| invalid("invalid search request"))?,
            timestamp,
        ),
        requests::SEARCH_ACTION => {
            let action = terra_autonomy::decode_search_action(bytes)
                .ok_or_else(|| invalid("invalid search action"))?;
            let healthy = b.sensors_ready(timestamp);
            let input = arbiter_input(&b.estimator, b.pose, b.map.as_ref(), timestamp, healthy);
            b.autonomy
                .apply_search_action(action, &input)
                .map_err(invalid)
        }
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
