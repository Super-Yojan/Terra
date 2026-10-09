//! Sensor-only target confirmation and finite search lifecycle.
use crate::*;
use serde::{Deserialize, Serialize};
pub type SearchError = &'static str;
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TargetObservation {
    pub run_id: String,
    pub search_id: String,
    pub frame_id: u64,
    pub target_class: String,
    pub confidence: f64,
    pub world_x: f64,
    pub world_y: f64,
    pub received_at: f64,
    pub evidence_id: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ConfirmationConfig {
    pub confidence: f64,
    pub frames: usize,
    pub span_s: f64,
    pub agreement_m: f64,
    pub freshness_s: f64,
}
impl Default for ConfirmationConfig {
    fn default() -> Self {
        Self {
            confidence: 0.8,
            frames: 3,
            span_s: 0.5,
            agreement_m: 1.,
            freshness_s: 0.5,
        }
    }
}
impl ConfirmationConfig {
    pub fn valid(&self) -> bool {
        self.confidence.is_finite()
            && (0.0..=1.0).contains(&self.confidence)
            && (3..=64).contains(&self.frames)
            && [self.span_s, self.agreement_m, self.freshness_s]
                .iter()
                .all(|v| v.is_finite() && *v > 0.)
    }
}
pub struct SearchController {
    pub request: SearchRequest,
    pub navigator: terra_navigation::SearchNavigator,
    config: ConfirmationConfig,
    phase: SearchPhase,
    reason: String,
    started_at: f64,
    last_time: f64,
    last_frame: Option<u64>,
    candidate: Vec<TargetObservation>,
    report: Option<SearchReport>,
    goal: Option<(f64, f64)>,
    revision: u64,
    pub receipt: Option<(String, bool)>,
}
impl SearchController {
    pub fn start(request: SearchRequest, now: f64) -> Result<Self, SearchError> {
        Self::with_config(request, now, ConfirmationConfig::default(), 0.35)
    }
    pub fn with_config(
        request: SearchRequest,
        now: f64,
        config: ConfirmationConfig,
        radius: f64,
    ) -> Result<Self, SearchError> {
        if !request.valid() || !now.is_finite() || now < 0. || !config.valid() {
            return Err("invalid_search");
        }
        let b = request.bounds;
        let navigator = terra_navigation::SearchNavigator::new(
            terra_navigation::SearchBounds {
                min_x: b.min_x,
                min_y: b.min_y,
                max_x: b.max_x,
                max_y: b.max_y,
            },
            radius,
        )?;
        let token = request.token.clone();
        Ok(Self {
            request,
            navigator,
            config,
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
    pub fn hold(&mut self, reason: &str) {
        if !self.phase.terminal() && self.phase != SearchPhase::Paused {
            self.candidate.clear();
            self.transition(SearchPhase::NeedsAttention, reason);
        }
    }
    pub fn cancel(&mut self, reason: &str) {
        if !self.phase.terminal() {
            self.candidate.clear();
            self.transition(SearchPhase::Cancelled, reason);
        }
    }
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
            || !(self.config.confidence..=1.).contains(&o.confidence)
            || !self.request.bounds.contains(o.world_x, o.world_y)
            || !o.received_at.is_finite()
            || o.received_at < self.started_at
            || o.received_at > now
            || now - o.received_at > self.config.freshness_s
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
            .is_some_and(|c| o.received_at - c.received_at > self.config.freshness_s)
            || self.candidate.iter().any(|c| {
                (c.world_x - o.world_x).hypot(c.world_y - o.world_y) > self.config.agreement_m
            })
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
        if self.candidate.len() >= self.config.frames
            && last.received_at - first.received_at >= self.config.span_s
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
    pub fn tick(
        &mut self,
        map: &terra_mapping::MapSnapshot,
        pose: terra_navigation::Pose,
        revision: u64,
        now: f64,
    ) -> Result<Option<(f64, f64)>, SearchError> {
        self.update_time(now)?;
        if self.phase == SearchPhase::Confirming
            && self
                .candidate
                .last()
                .is_some_and(|c| now - c.received_at > self.config.freshness_s)
        {
            self.candidate.clear();
            self.transition(SearchPhase::Searching, "candidate_expired");
        }
        if self.phase != SearchPhase::Searching {
            return Ok(None);
        }
        match self.navigator.next_goal(map, pose, revision, now) {
            terra_navigation::SearchNavigationDecision::Goal { x, y } => {
                self.goal = Some((x, y));
                Ok(self.goal)
            }
            terra_navigation::SearchNavigationDecision::Exhausted => {
                self.transition(SearchPhase::Exhausted, "no_reachable_frontier");
                Ok(None)
            }
            terra_navigation::SearchNavigationDecision::Blocked { reason } => {
                self.hold(reason);
                Ok(None)
            }
        }
    }
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
