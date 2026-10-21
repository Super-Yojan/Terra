//! Geographic mission state, independent of the local pose and occupancy frame.
use serde_json::{Value, json};
use terra_navigation::Pose;
#[derive(Clone, Copy, Debug)]
pub struct GpsFix {
    pub timestamp: f64,
    pub latitude: f64,
    pub longitude: f64,
    pub accuracy: f64,
}
#[derive(Clone, Copy, Debug)]
pub struct HeadingFix {
    pub timestamp: f64,
    pub true_heading: f64,
    pub accuracy: f64,
}
#[derive(Clone, Copy)]
pub struct Evidence {
    pub pose: Pose,
    pub pose_ready: bool,
    pub map_ready: bool,
    pub hardware_ready: bool,
    pub armed: bool,
    pub authority: bool,
}
#[derive(Clone)]
struct Mission {
    latitude: f64,
    longitude: f64,
    token: String,
    radius: f64,
    count: u8,
    last_fix: Option<f64>,
}
pub struct OutdoorController {
    pub supported: bool,
    pub selected: bool,
    pub revision: u64,
    pub radius: f64,
    pub token: Option<String>,
    pub result: Option<String>,
    gps: Option<GpsFix>,
    gps_time: Option<f64>,
    heading_time: Option<f64>,
    heading: Option<HeadingFix>,
    mission: Option<Mission>,
    phase: &'static str,
    hold: Option<&'static str>,
    last_guidance: Option<f64>,
    remaining: Option<f64>,
}
impl Default for OutdoorController {
    fn default() -> Self {
        Self {
            supported: false,
            selected: false,
            revision: 0,
            radius: 10.,
            token: None,
            result: None,
            gps: None,
            gps_time: None,
            heading_time: None,
            heading: None,
            mission: None,
            phase: "idle",
            hold: None,
            last_guidance: None,
            remaining: None,
        }
    }
}
fn fresh(now: f64, t: f64, limit: f64) -> bool {
    now.is_finite() && t.is_finite() && t >= 0. && (0.0..=limit).contains(&(now - t))
}
fn coordinates(lat: f64, lon: f64) -> bool {
    lat.is_finite()
        && lon.is_finite()
        && (-85.0..=85.).contains(&lat)
        && (-180.0..=180.).contains(&lon)
}
/// Great-circle metres and true-north clockwise bearing in degrees.
pub fn distance_bearing(lat: f64, lon: f64, to_lat: f64, to_lon: f64) -> (f64, f64) {
    let (a, b) = (lat.to_radians(), to_lat.to_radians());
    let dl = (to_lon - lon).to_radians();
    let da = b - a;
    let h = ((da / 2.).sin().powi(2) + a.cos() * b.cos() * (dl / 2.).sin().powi(2)).clamp(0., 1.);
    let bearing = (dl.sin() * b.cos())
        .atan2(a.cos() * b.sin() - a.sin() * b.cos() * dl.cos())
        .to_degrees()
        .rem_euclid(360.);
    (2. * 6_371_000. * h.sqrt().asin(), bearing)
}
impl OutdoorController {
    pub fn invalidate_sensors(&mut self) {
        self.gps = None;
        self.heading = None;
        self.pause("tracking_lost");
    }
    pub fn phase(&self) -> &str {
        self.phase
    }
    pub fn active(&self) -> bool {
        self.mission.is_some() && self.phase != "arrived"
    }
    /// Sensor recovery permits rearming, but never releases the explicit resume latch.
    pub fn recovery_ready(&self, now: f64, e: Evidence) -> bool {
        e.authority
            && self
                .readiness(
                    now,
                    e,
                    self.mission.as_ref().map_or(self.radius, |m| m.radius),
                )
                .is_none()
    }
    pub fn gps(&mut self, s: impl Into<GpsFix>, now: f64) -> Result<(), &'static str> {
        let s = s.into();
        if !coordinates(s.latitude, s.longitude)
            || !s.accuracy.is_finite()
            || s.accuracy < 0.
            || !fresh(now, s.timestamp, 3.)
            || self.gps_time.is_some_and(|t| s.timestamp <= t)
        {
            self.gps = None;
            self.pause("gps_stale");
            return Err("invalid GPS sample");
        }
        self.gps_time = Some(s.timestamp);
        self.gps = Some(s);
        Ok(())
    }
    pub fn heading(&mut self, s: impl Into<HeadingFix>, now: f64) -> Result<(), &'static str> {
        let s = s.into();
        if !s.true_heading.is_finite()
            || !(0.0..360.).contains(&s.true_heading)
            || !s.accuracy.is_finite()
            || s.accuracy < 0.
            || !fresh(now, s.timestamp, 2.)
            || self.heading_time.is_some_and(|t| s.timestamp <= t)
        {
            self.heading = None;
            self.pause("heading_uncertain");
            return Err("invalid heading sample");
        }
        self.heading_time = Some(s.timestamp);
        self.heading = Some(s);
        Ok(())
    }
    pub fn readiness(&self, now: f64, e: Evidence, radius: f64) -> Option<&'static str> {
        if self.gps.is_none_or(|g| !fresh(now, g.timestamp, 3.)) {
            Some("gps_stale")
        } else if self
            .gps
            .is_some_and(|g| g.accuracy > 10. || g.accuracy > radius)
        {
            Some("gps_uncertain")
        } else if self
            .heading
            .is_none_or(|h| !fresh(now, h.timestamp, 2.) || h.accuracy > 35.)
        {
            Some("heading_uncertain")
        } else if !e.pose_ready {
            Some("tracking_lost")
        } else if !e.map_ready {
            Some("obstacle_map_stale")
        } else if !e.hardware_ready {
            Some("hardware_unavailable")
        } else {
            None
        }
    }
    pub fn start(
        &mut self,
        lat: f64,
        lon: f64,
        token: String,
        radius: f64,
        now: f64,
        e: Evidence,
    ) -> Result<(), &'static str> {
        if !coordinates(lat, lon)
            || !radius.is_finite()
            || !(2.0..=20.).contains(&radius)
            || !crate::valid_token(&token)
        {
            return Err("invalid_goal");
        }
        if let Some(why) = self.readiness(now, e, radius) {
            return Err(why);
        }
        if !e.authority {
            return Err("authority_lost");
        }
        if !e.armed {
            return Err("disarmed");
        }
        let g = self.gps.unwrap();
        if distance_bearing(g.latitude, g.longitude, lat, lon).0 > 500. {
            return Err("goal_too_far");
        }
        self.mission = Some(Mission {
            latitude: lat,
            longitude: lon,
            token,
            radius,
            count: 0,
            last_fix: None,
        });
        self.phase = "planning";
        self.hold = None;
        self.last_guidance = None;
        self.remaining = None;
        Ok(())
    }
    pub fn pause(&mut self, reason: &'static str) {
        if self.active() {
            self.phase = "paused";
            self.hold = Some(reason);
            self.last_guidance = None;
            if let Some(m) = self.mission.as_mut() {
                m.count = 0;
            }
        }
    }
    pub fn resume(&mut self, now: f64, e: Evidence) -> Result<(), &'static str> {
        let m = self.mission.as_ref().ok_or("no_mission")?;
        if self.phase != "paused" {
            return Err("not_paused");
        }
        if let Some(why) = self.readiness(now, e, m.radius) {
            return Err(why);
        }
        if !e.authority {
            return Err("authority_lost");
        }
        if !e.armed {
            return Err("disarmed");
        }
        self.phase = "planning";
        self.hold = None;
        self.last_guidance = None;
        Ok(())
    }
    pub fn cancel(&mut self) {
        self.mission = None;
        self.phase = "idle";
        self.hold = None;
        self.last_guidance = None;
        self.remaining = None;
    }
    /// A changed guidance target at most twice a second. No pose/map mutation.
    pub fn tick(&mut self, now: f64, e: Evidence) -> Option<(f64, f64)> {
        if !self.active() {
            return None;
        }
        let radius = self.mission.as_ref()?.radius;
        if let Some(why) = self.readiness(now, e, radius) {
            self.pause(why);
            return None;
        }
        if !e.authority {
            self.pause("authority_lost");
            return None;
        }
        if !e.armed {
            self.pause("disarmed");
            return None;
        }
        if self.phase == "paused" {
            return None;
        }
        let g = self.gps?;
        let h = self.heading?;
        let m = self.mission.as_mut()?;
        let (distance, bearing) =
            distance_bearing(g.latitude, g.longitude, m.latitude, m.longitude);
        self.remaining = Some(distance);
        if distance <= m.radius {
            if m.last_fix != Some(g.timestamp) {
                m.count += 1;
                m.last_fix = Some(g.timestamp);
            }
            self.phase = if m.count >= 3 {
                "arrived"
            } else {
                "confirming_arrival"
            };
            self.last_guidance = None;
            return None;
        }
        m.count = 0;
        m.last_fix = Some(g.timestamp);
        self.phase = "planning";
        if self.last_guidance.is_some_and(|t| now - t < 0.5) {
            return None;
        }
        self.last_guidance = Some(now);
        // Local yaw is counter-clockwise; true bearing is clockwise.
        let yaw = e.pose.yaw + (h.true_heading - bearing).to_radians();
        let lookahead = distance.min(3.);
        Some((
            e.pose.x + lookahead * yaw.cos(),
            e.pose.y + lookahead * yaw.sin(),
        ))
    }
    pub fn status(&self, now: f64, e: Evidence, run: &str) -> Value {
        let radius = self.mission.as_ref().map_or(self.radius, |m| m.radius);
        let reason = self.hold.or_else(|| self.readiness(now, e, radius));
        json!({"schema_version":1,"capability":"outdoor_gps","supported":self.supported,"acknowledged":self.selected,
   "source":if self.selected {"outdoor_gps"} else {"local"},"run_id":run,"navigation_revision":self.revision,
   "ready":reason.is_none() && self.phase!="paused", "hold_reason":reason,"phase":self.phase,"arrival_radius_m":radius,
   "gps":self.gps.map(|g|json!({"latitude":g.latitude,"longitude":g.longitude,"accuracy_m":g.accuracy,"age_s":now-g.timestamp})),
   "gps_age_s":self.gps.map(|g|now-g.timestamp),"gps_accuracy_m":self.gps.map(|g|g.accuracy),
   "heading_age_s":self.heading.map(|h|now-h.timestamp),"heading_accuracy_deg":self.heading.map(|h|h.accuracy),
   "destination":self.mission.as_ref().map(|m|json!({"latitude":m.latitude,"longitude":m.longitude})),
   "remaining_m":self.remaining,"token":self.token,"result":self.result,
   "mission_token":self.mission.as_ref().map(|m|&m.token)})
    }
    pub fn mission_status(&self) -> Option<terra_waypoint::WaypointStatus> {
        let m = self.mission.as_ref()?;
        let mut s = terra_waypoint::WaypointStatus::idle();
        s.state = if self.phase == "arrived" {
            terra_waypoint::GoalState::Arrived
        } else {
            terra_waypoint::GoalState::Active
        };
        s.goal_id = 1;
        s.token = Some(m.token.clone());
        s.latitude = Some(m.latitude);
        s.longitude = Some(m.longitude);
        s.distance = self.remaining.unwrap_or(0.);
        Some(s)
    }
}
