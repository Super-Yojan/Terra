//! Detector setup, target-search requests and report acknowledgements.
use super::*;

impl AutonomyArbiter {
    /// Advertise up to 64 valid classes and a valid version token at nondecreasing `now` seconds.
    ///
    /// Returns `invalid_detector_capability` for invalid labels, version, or time, leaving the previous capability intact.
    pub fn set_detector_capability(
        &mut self,
        classes: Vec<String>,
        version: String,
        now: f64,
    ) -> Result<(), SearchError> {
        if classes.len() > 64
            || !classes.iter().all(|s| valid_class(s))
            || !valid_token(&version)
            || !now.is_finite()
            || now < 0.
            || self.detector_time.is_some_and(|t| now < t)
        {
            return Err("invalid_detector_capability");
        }
        self.detector_classes = classes;
        self.detector_version = version;
        self.detector_time = Some(now);
        Ok(())
    }
    /// Return whether `class` is advertised and the capability timestamp is within the configured detector freshness limit.
    pub fn detector_available(&self, class: &str, now: f64) -> bool {
        self.detector_classes.iter().any(|c| c == class)
            && fresh(now, self.detector_time, timing::DETECTOR_FRESHNESS_SECONDS)
    }
    /// Return the current session snapshot and detector freshness, or `None` before a search is created.
    pub fn search_status(&self, now: f64) -> Option<SearchStatus> {
        self.search
            .as_ref()
            .map(|s| s.status(now, self.detector_available(&s.request.target_class, now)))
    }
    /// Start a validated search under target-search authority with fresh healthy pose, map, and detector evidence.
    ///
    /// Returns a machine-readable rejection for correlation, capacity, reused session, bounds, health, or authority failures.
    /// Matching accepted duplicate tokens are idempotent and do not restart the session.
    pub fn start_search(
        &mut self,
        r: SearchRequest,
        input: &ArbiterInput<'_>,
    ) -> Result<(), SearchError> {
        if !r.valid() {
            return Err("invalid_search");
        }
        let fp = format!("search:{}", serde_json::to_string(&r).unwrap());
        if let Some(result) = self.replay(&r.token, &fp) {
            return result;
        }
        let reason = if r.run_id != self.run_id {
            Some("wrong_run")
        } else if self.stopped {
            Some(SAFETY_STOPPED)
        } else if self.level != Level::TargetSearch {
            Some("authority_required")
        } else if self.search.as_ref().is_some_and(|s| !s.phase().terminal()) {
            Some("search_active")
        } else if self.search_sessions.contains(&r.search_id) {
            Some("search_id_reused")
        } else if self.search_sessions.len() >= 1024 {
            Some("search_session_capacity")
        } else if self.pending_reports.len() >= 128 {
            Some("report_capacity")
        } else if !self.detector_available(&r.target_class, input.now) {
            Some("detector_unavailable")
        } else if !body_inside(&self.planner.config.footprint, input.pose, &r.bounds) {
            Some("outside_search_bounds")
        } else if !input.healthy
            || !input.pose.finite()
            || !fresh(
                input.now,
                Some(input.pose_time),
                timing::POSE_FRESHNESS_SECONDS,
            )
            || input.map.is_none()
            || !self.map_is_fresh(input)
            || self.last_time.is_some_and(|t| input.now < t)
        {
            Some("search_inputs_unhealthy")
        } else {
            None
        };
        if let Some(reason) = reason {
            self.acknowledge(r.token, fp, false, reason);
            return Err(reason);
        }
        self.collect_search_report(input.now);
        let search = SearchController::new(r.clone(), input.now, self.planner.config.footprint)?;
        self.follower.cancel();
        self.plan.forget();
        self.search_sessions.push_back(r.search_id.clone());
        self.search = Some(search);
        self.paused = false;
        self.acknowledge(r.token.clone(), fp, true, "search_started");
        self.event("search_accepted", Some(r.token), "search_started");
        Ok(())
    }
    /// Apply an action to the matching active run and search, with token idempotency.
    ///
    /// Resume additionally requires fresh healthy pose, map, and detector evidence.
    /// Returns validation, correlation, authority, health, clock, or terminal-state errors.
    pub fn apply_search_action(
        &mut self,
        r: SearchActionRequest,
        input: &ArbiterInput<'_>,
    ) -> Result<(), SearchError> {
        if !r.valid() {
            return Err("invalid_search_action");
        }
        let fp = format!("search_action:{}", serde_json::to_string(&r).unwrap());
        if let Some(result) = self.replay(&r.token, &fp) {
            return result;
        }
        let valid =
            self.search.as_ref().is_some_and(|s| {
                s.request.search_id == r.search_id && s.request.run_id == r.run_id
            }) && self.run_id == r.run_id
                && self.level == Level::TargetSearch
                && !self.stopped;
        if !valid {
            self.acknowledge(r.token, fp, false, "wrong_search_or_authority");
            return Err("wrong_search_or_authority");
        }
        if r.action == SearchAction::Resume
            && (!input.healthy
                || input.map.is_none()
                || !fresh(
                    input.now,
                    Some(input.pose_time),
                    timing::POSE_FRESHNESS_SECONDS,
                )
                || !self.map_is_fresh(input)
                || !self.detector_available(
                    &self.search.as_ref().unwrap().request.target_class,
                    input.now,
                ))
        {
            self.acknowledge(r.token, fp, false, "search_inputs_unhealthy");
            return Err("search_inputs_unhealthy");
        }
        let result = self.search.as_mut().unwrap().action(r.action, input.now);
        self.acknowledge(
            r.token.clone(),
            fp,
            result.is_ok(),
            result.err().unwrap_or("search_action"),
        );
        self.search.as_mut().unwrap().receipt = Some((r.token.clone(), result.is_ok()));
        if result.is_ok() {
            self.follower.cancel();
            self.plan.forget();
            self.reset = true;
            self.event(
                match r.action {
                    SearchAction::Pause => "search_paused",
                    SearchAction::Resume => "search_resumed",
                    SearchAction::Cancel => "search_cancelled",
                },
                Some(r.token),
                "operator_search_action",
            );
        }
        result
    }
    /// Forward an observation only under target-search authority with a fresh matching detector.
    ///
    /// Returns detector/authority, missing-search, or controller observation errors; a new report is queued on confirmation.
    pub fn accept_target_observation(
        &mut self,
        o: TargetObservation,
        now: f64,
    ) -> Result<(), SearchError> {
        if self.level != Level::TargetSearch
            || self.stopped
            || !self.detector_available(&o.target_class, now)
        {
            return Err("detector_or_authority_unavailable");
        }
        let result = self.search.as_mut().ok_or("no_search")?.observe(o, now);
        self.collect_search_report(now);
        self.event(
            if result.is_ok() {
                "candidate_observed"
            } else {
                "candidate_rejected"
            },
            None,
            result.err().unwrap_or("observed_target"),
        );
        result
    }
    pub(super) fn collect_search_report(&mut self, now: f64) {
        let report = self.search_status(now).and_then(|s| s.report);
        if let Some(report) = report
            && self
                .reported_sessions
                .insert((report.run_id.clone(), report.search_id.clone()))
        {
            self.pending_reports.push_back(report);
            self.event("target_confirmed", None, "target_confirmed");
            self.event("report_queued", None, "report_pending");
        }
    }
    /// Remove a report matching the version, run, search, and report IDs.
    ///
    /// Returns `false` for malformed or unknown acknowledgements; repeats of a previously acknowledged report return `true`.
    pub fn ack_search_report(&mut self, r: SearchReportAck) -> bool {
        if r.version != 1
            || (![&r.run_id, &r.search_id, &r.token]
                .iter()
                .all(|s| valid_token(s))
                || !valid_report_id(&r.report_id))
        {
            return false;
        }
        if let Some(i) = self.pending_reports.iter().position(|p| {
            p.run_id == r.run_id && p.search_id == r.search_id && p.report_id == r.report_id
        }) {
            self.pending_reports.remove(i);
            self.acknowledged_reports.insert((
                r.run_id.clone(),
                r.search_id.clone(),
                r.report_id.clone(),
            ));
            self.event("report_acknowledged", Some(r.token), "report_delivered");
            true
        } else {
            self.acknowledged_reports
                .contains(&(r.run_id, r.search_id, r.report_id))
        }
    }
}

/// Whether the body rectangle at the pose's heading lies inside the bounds.
fn body_inside(footprint: &terra_navigation::Footprint, pose: Pose, bounds: &SearchBounds) -> bool {
    let (a, b) = footprint.half_extents(0.);
    let (sin, cos) = pose.yaw.sin_cos();
    let reach_x = a * cos.abs() + b * sin.abs();
    let reach_y = a * sin.abs() + b * cos.abs();
    pose.x - reach_x >= bounds.min_x
        && pose.x + reach_x <= bounds.max_x
        && pose.y - reach_y >= bounds.min_y
        && pose.y + reach_y <= bounds.max_y
}
