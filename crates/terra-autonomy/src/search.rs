//! Sensor-only target confirmation and finite search lifecycle.
use crate::*;
use serde::{Deserialize, Serialize};
/// Stable machine-readable rejection or lifecycle error string.
pub type SearchError = &'static str;
/// One detector frame in world coordinates; confirmation requires consistent distinct frames.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TargetObservation {
    /// Run identifier used to correlate commands and telemetry.
    pub run_id: String,
    /// Search-session identifier scoped to a run.
    pub search_id: String,
    /// Strictly increasing detector frame identifier within the search.
    pub frame_id: u64,
    /// Detector class label, using 1–64 ASCII letters, digits, underscores, or hyphens.
    pub target_class: String,
    /// Observation confidence in 0–1; must meet the configured minimum.
    pub confidence: f64,
    /// World-frame target x coordinate, in metres.
    pub world_x: f64,
    /// World-frame target y coordinate, in metres.
    pub world_y: f64,
    /// Observation receipt time in seconds on the search clock.
    pub received_at: f64,
    /// Token identifying the underlying sensor evidence.
    pub evidence_id: String,
}
/// Minimum observation confidence on the 0–1 scale.
const CONFIRM_CONFIDENCE: f64 = 0.8;
/// Supporting frames needed to confirm a target.
const CONFIRM_FRAMES: usize = 3;
/// Largest pairwise separation of supporting positions, metres.
const CONFIRM_AGREEMENT_M: f64 = 1.;
/// Shortest span between the first and last supporting frames, seconds.
const CONFIRM_SPAN_S: f64 = terra_types::timing::SEARCH_CONFIRMATION_SPAN_SECONDS;
/// Largest observation age, and gap between candidate frames, seconds.
const CONFIRM_FRESHNESS_S: f64 = terra_types::timing::SEARCH_OBSERVATION_FRESHNESS_SECONDS;
/// Finite target-search lifecycle combining frontier navigation with sensor-only confirmation.
pub struct SearchController {
    /// Validated session request retained for correlation and bounds checks.
    pub request: SearchRequest,
    /// Frontier navigation state used to select search destinations.
    navigator: terra_exploration::SearchNavigator,
    phase: SearchPhase,
    reason: String,
    started_at: f64,
    last_time: f64,
    last_frame: Option<u64>,
    candidate: Vec<TargetObservation>,
    report: Option<SearchReport>,
    goal: Option<(f64, f64)>,
    revision: u64,
    /// Latest request token and whether it was accepted.
    receipt: Option<(String, bool)>,
}
impl SearchController {
    /// Abandon the current search goal after a route failure (see
    /// [`terra_exploration::SearchNavigator::invalidate_goal`]).
    pub fn invalidate_goal(&mut self, now: f64) {
        self.navigator.invalidate_goal(now);
    }
    /// Record the latest request on this session for status.
    pub fn note_request(&mut self, token: String, accepted: bool) {
        self.receipt = Some((token, accepted));
    }
    /// Start a search for a rover with this body.
    ///
    /// Requires a valid request, nonnegative finite `now` seconds, and a navigator-valid body.
    /// Returns `invalid_search` or the navigator construction error on rejection.
    pub fn new(
        request: SearchRequest,
        now: f64,
        footprint: terra_navigation::Footprint,
    ) -> Result<Self, SearchError> {
        if !request.valid() || !now.is_finite() || now < 0. {
            return Err("invalid_search");
        }
        let navigator = terra_exploration::SearchNavigator::new(request.bounds, footprint)?;
        let token = request.token.clone();
        Ok(Self {
            request,
            navigator,
            phase: SearchPhase::Searching,
            reason: "search_started".into(),
            started_at: now,
            last_time: now,
            last_frame: None,
            candidate: vec![],
            report: None,
            goal: None,
            revision: 0,
            receipt: Some((token, true)),
        })
    }
    /// Return the current lifecycle phase without advancing time.
    pub fn phase(&self) -> SearchPhase {
        self.phase
    }
    fn transition(&mut self, phase: SearchPhase, reason: &str) {
        if self.phase != phase || self.reason != reason {
            self.phase = phase;
            self.reason = reason.into();
            self.revision += 1;
        }
        if phase != SearchPhase::Searching {
            self.goal = None;
        }
    }
    /// Advance the nondecreasing seconds clock and expire the budget, including while paused.
    ///
    /// Returns `invalid_time` for nonfinite or backward time without advancing it.
    pub fn update_time(&mut self, now: f64) -> Result<(), SearchError> {
        if !now.is_finite() || now < self.last_time {
            return Err("invalid_time");
        }
        self.last_time = now;
        if !self.phase.terminal() && now - self.started_at >= self.request.time_budget_s {
            self.candidate.clear();
            self.transition(SearchPhase::TimedOut, "time_budget_exceeded");
        }
        Ok(())
    }
    /// Apply a lifecycle action at `now` seconds, clearing confirmation candidates.
    ///
    /// Returns a clock error or `search_terminal` if the budget has expired or the session already ended.
    pub fn action(&mut self, action: SearchAction, now: f64) -> Result<(), SearchError> {
        self.update_time(now)?;
        if self.phase.terminal() {
            return Err("search_terminal");
        }
        self.candidate.clear();
        match action {
            SearchAction::Cancel => self.transition(SearchPhase::Cancelled, "operator_cancel"),
            SearchAction::Pause => self.transition(SearchPhase::Paused, "operator_pause"),
            SearchAction::Resume => {
                self.navigator.replan();
                self.transition(SearchPhase::Searching, "operator_resume");
            }
        }
        Ok(())
    }
    /// Clear candidate evidence and enter attention hold unless already paused or terminal.
    pub fn hold(&mut self, reason: &str) {
        if !self.phase.terminal() && self.phase != SearchPhase::Paused {
            self.candidate.clear();
            self.transition(SearchPhase::NeedsAttention, reason);
        }
    }
    /// Clear candidate evidence and permanently cancel a nonterminal session with the supplied reason.
    pub fn cancel(&mut self, reason: &str) {
        if !self.phase.terminal() {
            self.candidate.clear();
            self.transition(SearchPhase::Cancelled, reason);
        }
    }
    /// Accept one fresh, in-bounds, matching-class observation and update confirmation.
    ///
    /// Returns a clock, phase, or `invalid_observation` error for invalid correlation, confidence, frame order, or age.
    /// Disagreement or excessive inter-frame gaps restart the candidate set; consistent frames can complete the search.
    pub fn observe(&mut self, o: TargetObservation, now: f64) -> Result<(), SearchError> {
        self.update_time(now)?;
        if !matches!(self.phase, SearchPhase::Searching | SearchPhase::Confirming) {
            return Err("search_not_observing");
        }
        if o.run_id != self.request.run_id
            || o.search_id != self.request.search_id
            || o.target_class != self.request.target_class
            || !valid_token(&o.evidence_id)
            || !o.confidence.is_finite()
            || !(CONFIRM_CONFIDENCE..=1.).contains(&o.confidence)
            || !self.request.bounds.contains(o.world_x, o.world_y)
            || o.received_at < self.started_at
            || !terra_types::timing::fresh(now, Some(o.received_at), CONFIRM_FRESHNESS_S)
            || self.last_frame.is_some_and(|f| o.frame_id <= f)
            || self
                .candidate
                .last()
                .is_some_and(|c| o.received_at < c.received_at)
        {
            return Err("invalid_observation");
        }
        self.last_frame = Some(o.frame_id);
        if self
            .candidate
            .last()
            .is_some_and(|c| o.received_at - c.received_at > CONFIRM_FRESHNESS_S)
            || self
                .candidate
                .iter()
                .any(|c| (c.world_x - o.world_x).hypot(c.world_y - o.world_y) > CONFIRM_AGREEMENT_M)
        {
            self.candidate.clear();
        }
        if self.candidate.len() == 64 {
            self.candidate.remove(0);
        }
        self.candidate.push(o);
        self.transition(SearchPhase::Confirming, "target_candidate");
        let first = &self.candidate[0];
        let last = self.candidate.last().unwrap();
        if self.candidate.len() >= CONFIRM_FRAMES
            && last.received_at - first.received_at >= CONFIRM_SPAN_S
        {
            let n = self.candidate.len() as f64;
            self.report = Some(SearchReport {
                version: 1,
                report_id: format!("{}:report", self.request.search_id),
                run_id: self.request.run_id.clone(),
                search_id: self.request.search_id.clone(),
                target_class: self.request.target_class.clone(),
                world_x: self.candidate.iter().map(|c| c.world_x / n).sum(),
                world_y: self.candidate.iter().map(|c| c.world_y / n).sum(),
                confirmed_at: now,
                frame_ids: self.candidate.iter().map(|c| c.frame_id).collect(),
                evidence_ids: self
                    .candidate
                    .iter()
                    .map(|c| c.evidence_id.clone())
                    .collect(),
            });
            self.transition(SearchPhase::Completed, "target_confirmed");
        }
        Ok(())
    }
    /// Advance search at `now` seconds and select a world-metre frontier using the map revision and pose.
    ///
    /// Returns `Ok(None)` during holds or terminal phases, and a clock error for nonfinite or backward time.
    /// Stale confirmation evidence resumes navigation; blocked or exhausted navigation updates the lifecycle.
    pub fn tick(
        &mut self,
        map: &terra_mapping::MapSnapshot,
        pose: terra_navigation::Pose,
        revision: u64,
        now: f64,
    ) -> Result<Option<(f64, f64)>, SearchError> {
        self.update_time(now)?;
        if self.phase == SearchPhase::Confirming
            && self.candidate.last().is_some_and(|c| {
                !terra_types::timing::fresh(now, Some(c.received_at), CONFIRM_FRESHNESS_S)
            })
        {
            self.candidate.clear();
            self.transition(SearchPhase::Searching, "candidate_expired");
        }
        if self.phase != SearchPhase::Searching {
            return Ok(None);
        }
        match self.navigator.next_goal(map, pose, revision, now) {
            terra_exploration::SearchNavigationDecision::Goal { x, y } => {
                self.goal = Some((x, y));
                Ok(self.goal)
            }
            terra_exploration::SearchNavigationDecision::Exhausted => {
                self.transition(SearchPhase::Exhausted, "no_reachable_frontier");
                Ok(None)
            }
            terra_exploration::SearchNavigationDecision::Blocked { reason } => {
                self.hold(reason);
                Ok(None)
            }
        }
    }
    /// Snapshot the session at `now` seconds without advancing lifecycle state.
    ///
    /// Elapsed time never precedes the last accepted time; nonfinite `now` falls back to it. Detector health is caller-supplied.
    pub fn status(&self, now: f64, detector_available: bool) -> SearchStatus {
        SearchStatus {
            target_class: self.request.target_class.clone(),
            version: 1,
            run_id: self.request.run_id.clone(),
            search_id: self.request.search_id.clone(),
            revision: self.revision,
            phase: self.phase,
            reason: self.reason.clone(),
            elapsed_s: if now.is_finite() {
                (now.max(self.last_time) - self.started_at).max(0.)
            } else {
                self.last_time - self.started_at
            },
            active_goal: self.goal,
            report: self.report.clone(),
            detector_available,
            token: self.receipt.as_ref().map(|r| r.0.clone()),
            result: self
                .receipt
                .as_ref()
                .map(|r| if r.1 { "accepted" } else { "rejected" }.into()),
        }
    }
}
