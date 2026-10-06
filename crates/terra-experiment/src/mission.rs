use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Survivor {
    pub id: u64,
    pub x: f64,
    pub y: f64,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MissionConfig {
    pub mission_id: String,
    pub seed: u64,
    pub half_extent: f64,
    pub time_budget: f64,
    pub detection_range: f64,
    pub survivors: Vec<Survivor>,
}
impl MissionConfig {
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
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Sighting {
    pub survivor_id: u64,
    pub x: f64,
    pub y: f64,
    pub rover_id: u64,
    pub observed_at: f64,
    pub confirmed: bool,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MissionStatus {
    pub mission_id: String,
    pub objective: String,
    pub seed: u64,
    pub phase: String,
    pub confirmed: usize,
    pub required: usize,
    pub remaining_seconds: f64,
    pub half_extent: f64,
    pub complete: bool,
    pub failed: bool,
    pub observations: Vec<Sighting>,
}
pub struct Mission {
    config: MissionConfig,
    sightings: BTreeMap<u64, Sighting>,
    confirmed: BTreeSet<u64>,
    last_time: f64,
    returned: BTreeSet<u64>,
}
impl Mission {
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
    pub fn candidates(&self) -> &[Survivor] {
        &self.config.survivors
    }
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
