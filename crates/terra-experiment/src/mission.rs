//! Mission definitions and versioned records used to interpret experiment outcomes.

use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
/// Ground-truth survivor placement for deterministic mission evaluation.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Survivor {
    /// Unique survivor identifier within this mission.
    pub id: u64,
    /// World-frame x coordinate, in metres.
    pub x: f64,
    /// World-frame y coordinate, in metres.
    pub y: f64,
}
/// Seeded search-and-return scenario; distances are metres and durations are seconds.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MissionConfig {
    /// Scenario identifier carried into mission status.
    pub mission_id: String,
    /// Seed determining the reproducible survivor layout.
    pub seed: u64,
    /// Half-width of the square mission region, in metres from the origin.
    pub half_extent: f64,
    /// Positive finite mission duration, in seconds.
    pub time_budget: f64,
    /// Maximum Euclidean observation distance, in metres.
    pub detection_range: f64,
    /// Ground-truth placements; not published as unobserved sightings.
    pub survivors: Vec<Survivor>,
}
impl MissionConfig {
    /// Build a deterministic three-survivor mission with a 600 s budget, 5 m detection range, and 40 m half-extent.
    pub fn rescue(seed: u64) -> Self {
        let offset = (seed % 7) as f64;
        Self {
            mission_id: format!("rescue-{seed}"),
            seed,
            half_extent: 40.,
            time_budget: 600.,
            detection_range: 5.,
            survivors: vec![
                Survivor {
                    id: 1,
                    x: 8. + offset,
                    y: 4.,
                },
                Survivor {
                    id: 2,
                    x: -12.,
                    y: 10. + offset,
                },
                Survivor {
                    id: 3,
                    x: 16.,
                    y: -14.,
                },
            ],
        }
    }
}
/// First observed survivor location and its reporting rover.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Sighting {
    /// Identifier of the observed survivor.
    pub survivor_id: u64,
    /// World-frame x coordinate, in metres.
    pub x: f64,
    /// World-frame y coordinate, in metres.
    pub y: f64,
    /// Identifier of the rover that first observed the survivor.
    pub rover_id: u64,
    /// Observation time in seconds on the run clock.
    pub observed_at: f64,
    /// Whether the observing rover has confirmed this sighting.
    pub confirmed: bool,
}
/// Mission progress exposing observed targets, confirmations, and return-to-base completion.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MissionStatus {
    /// Scenario identifier carried into mission status.
    pub mission_id: String,
    /// Human-readable search, confirmation, and return objective.
    pub objective: String,
    /// Seed determining the reproducible survivor layout.
    pub seed: u64,
    /// Current lifecycle phase.
    pub phase: String,
    /// Number of survivors confirmed by their observing rovers.
    pub confirmed: usize,
    /// Total survivors that must be confirmed.
    pub required: usize,
    /// Remaining budget in seconds, clamped to zero.
    pub remaining_seconds: f64,
    /// Half-width of the square mission region, in metres from the origin.
    pub half_extent: f64,
    /// Whether the recording or mission completed successfully, according to this record.
    pub complete: bool,
    /// Whether the budget expired before mission completion.
    pub failed: bool,
    /// First sightings ordered by survivor identifier; hidden targets are omitted.
    pub observations: Vec<Sighting>,
}
/// Deterministic mission evaluator separating sightings, confirmation, and safe return.
pub struct Mission {
    config: MissionConfig,
    sightings: BTreeMap<u64, Sighting>,
    confirmed: BTreeSet<u64>,
    last_time: f64,
    returned: BTreeSet<u64>,
}
impl Mission {
    /// Validate and create a mission with no sightings or confirmations.
    ///
    /// Returns an error for nonpositive/nonfinite dimensions or timing, zero or over 1000 survivors, duplicate IDs,
    /// or nonfinite/out-of-bounds placements.
    pub fn new(c: MissionConfig) -> Result<Self, &'static str> {
        if !c.half_extent.is_finite()
            || c.half_extent <= 0.
            || !c.time_budget.is_finite()
            || c.time_budget <= 0.
            || !c.detection_range.is_finite()
            || c.detection_range <= 0.
            || c.survivors.is_empty()
            || c.survivors.len() > 1000
        {
            return Err("invalid mission configuration");
        }
        let mut ids = BTreeSet::new();
        for s in &c.survivors {
            if !s.x.is_finite()
                || !s.y.is_finite()
                || s.x.abs() > c.half_extent
                || s.y.abs() > c.half_extent
                || !ids.insert(s.id)
            {
                return Err("invalid survivor placement");
            }
        }
        Ok(Self {
            config: c,
            sightings: BTreeMap::new(),
            confirmed: BTreeSet::new(),
            last_time: 0.,
            returned: BTreeSet::new(),
        })
    }
    /// Borrow all ground-truth placements for simulator evaluation; these are not observation telemetry.
    pub fn candidates(&self) -> &[Survivor] {
        &self.config.survivors
    }
    /// Return newly visible first sightings within detection range of world-metre `(x, y)`.
    ///
    /// `now` is nondecreasing seconds from mission start and must be within budget.
    /// Nonfinite inputs, backward/expired time, or false line of sight return an empty vector without recording sightings.
    pub fn observe(
        &mut self,
        rover: u64,
        x: f64,
        y: f64,
        now: f64,
        line_of_sight: bool,
    ) -> Vec<Sighting> {
        if !x.is_finite()
            || !y.is_finite()
            || !now.is_finite()
            || now < self.last_time
            || now > self.config.time_budget
            || !line_of_sight
        {
            return Vec::new();
        }
        self.last_time = now;
        let mut new = Vec::new();
        for s in &self.config.survivors {
            if (s.x - x).hypot(s.y - y) <= self.config.detection_range
                && !self.sightings.contains_key(&s.id)
            {
                let sight = Sighting {
                    survivor_id: s.id,
                    x: s.x,
                    y: s.y,
                    rover_id: rover,
                    observed_at: now,
                    confirmed: false,
                };
                self.sightings.insert(s.id, sight.clone());
                new.push(sight);
            }
        }
        new
    }
    /// Observe only the specified survivor at the given world-metre pose and seconds timestamp.
    ///
    /// Uses the same range and time checks as `observe`, assuming line of sight; unknown or already seen IDs yield no sightings.
    pub fn observe_target(
        &mut self,
        rover: u64,
        id: u64,
        x: f64,
        y: f64,
        now: f64,
    ) -> Vec<Sighting> {
        let others = self.config.survivors.clone();
        self.config.survivors.retain(|s| s.id == id);
        let result = self.observe(rover, x, y, now, true);
        self.config.survivors = others;
        result
    }
    /// Confirm a first sighting by its original rover at a nondecreasing seconds timestamp within budget.
    ///
    /// Returns `false` for invalid time, unknown sightings, another rover, or duplicate confirmation.
    pub fn report(&mut self, rover: u64, id: u64, now: f64) -> bool {
        if !now.is_finite() || now < self.last_time || now > self.config.time_budget {
            return false;
        }
        let Some(s) = self.sightings.get_mut(&id) else {
            return false;
        };
        if s.rover_id != rover || !self.confirmed.insert(id) {
            return false;
        }
        s.confirmed = true;
        self.last_time = now;
        true
    }
    /// Mark a rover returned when all targets are confirmed and it is within 2 m of the origin.
    ///
    /// Coordinates use metres; invalid or backward times, expired budgets, and nonfinite positions are ignored.
    pub fn update_pose(&mut self, rover: u64, x: f64, y: f64, now: f64) {
        if now.is_finite()
            && now >= self.last_time
            && now <= self.config.time_budget
            && x.is_finite()
            && y.is_finite()
            && x.hypot(y) <= 2.
            && self.confirmed.len() == self.config.survivors.len()
        {
            self.returned.insert(rover);
        }
    }
    /// Snapshot mission progress at `now` seconds from mission start.
    ///
    /// Completion requires every target confirmed and every observing rover returned; otherwise budget expiry marks failure.
    /// This method does not validate or advance the clock; callers should supply a finite nonnegative time.
    pub fn status(&self, now: f64) -> MissionStatus {
        let all_confirmed = self.confirmed.len() == self.config.survivors.len();
        let complete = all_confirmed
            && self
                .sightings
                .values()
                .all(|s| self.returned.contains(&s.rover_id));
        MissionStatus {
            mission_id: self.config.mission_id.clone(),
            objective: "Search sectors, confirm survivor locations, and return to base".into(),
            seed: self.config.seed,
            phase: if complete {
                "complete"
            } else if now >= self.config.time_budget {
                "failed"
            } else if all_confirmed {
                "return"
            } else {
                "search"
            }
            .into(),
            confirmed: self.confirmed.len(),
            required: self.config.survivors.len(),
            remaining_seconds: (self.config.time_budget - now).max(0.),
            half_extent: self.config.half_extent,
            complete,
            failed: !complete && now >= self.config.time_budget,
            observations: self.sightings.values().cloned().collect(),
        }
    }
}
