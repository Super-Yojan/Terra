//! Geographic mission state, independent of the local pose and occupancy frame.
use serde_json::{Value, json};
use terra_navigation::Pose;
mod anchor;
mod heading;
pub use anchor::GeoAnchor;
pub use heading::{CourseFix, HeadingFusion, Source as HeadingSource};
/// Autonomy levels that accept a geographic destination, as wire names.
/// L2 and L3 follow it directly; L4 explore and L5 task plan toward it on the local map.
pub const GOAL_LEVELS: [&str; 4] = ["waypoint_direct", "waypoint", "explore", "task"];
/// Guidance lookahead along the destination bearing, in metres.
pub const LOOKAHEAD_M: f64 = 3.;
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
    heading: HeadingFusion,
    anchor: GeoAnchor,
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
            heading: HeadingFusion::default(),
            anchor: GeoAnchor::default(),
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
/// Point `distance` metres from (`lat`, `lon`) along true bearing `bearing` radians.
pub fn destination(lat: f64, lon: f64, distance: f64, bearing: f64) -> (f64, f64) {
    let (a, l) = (lat.to_radians(), lon.to_radians());
    let d = distance / 6_371_000.;
    let b = (a.sin() * d.cos() + a.cos() * d.sin() * bearing.cos()).asin();
    let m = l + (bearing.sin() * d.sin() * a.cos()).atan2(d.cos() - a.sin() * b.sin());
    (
        b.to_degrees(),
        (m.to_degrees() + 540.).rem_euclid(360.) - 180.,
    )
}
impl OutdoorController {
    pub fn invalidate_sensors(&mut self) {
        self.gps = None;
        self.heading.reset();
        self.anchor.reset();
        self.pause("tracking_lost");
    }
    pub fn phase(&self) -> &str {
        self.phase
    }
    pub fn active(&self) -> bool {
        self.mission.is_some() && self.phase != "arrived"
    }
    /// Sensor recovery restores readiness, but never releases the explicit resume latch.
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
        self.anchor.fix(s.latitude, s.longitude, s.accuracy);
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
            self.heading.reset();
            self.pause("heading_uncertain");
            return Err("invalid heading sample");
        }
        self.heading_time = Some(s.timestamp);
        self.heading.compass(s.true_heading, s.accuracy, now)
    }
    /// GPS course over ground; ignored unless the rover is travelling straight.
    pub fn course(&mut self, s: impl Into<CourseFix>, now: f64) -> Result<(), &'static str> {
        self.heading.course(s.into(), now)
    }
    /// Fused true heading, one-sigma accuracy and pose age, all in degrees or seconds.
    pub fn heading_estimate(&self, now: f64) -> Option<(f64, f64, f64)> {
        self.heading.heading(now)
    }
    /// Local VIO pose that carries the fused heading between corrections.
    /// The same pose carries the smoothed geographic position between GPS fixes.
    pub fn observe_pose(&mut self, pose: Pose, t: f64) {
        self.heading.observe_pose(pose, t);
        let east = self.heading.local_yaw(90.).zip(
            self.heading
                .heading(t)
                .map(|(_, accuracy, _)| accuracy.to_radians()),
        );
        self.anchor.observe_pose(pose, t, east);
    }
    /// Rover position for guidance: the smoothed anchor, else the newest fix.
    fn guidance_position(&self) -> Option<(f64, f64)> {
        self.anchor
            .position()
            .map(|(lat, lon, _)| (lat, lon))
            .or(self.gps.map(|g| (g.latitude, g.longitude)))
    }
    /// The active destination in the local frame, re-anchored on every pose and fix.
    /// `None` without a mission, a position or a heading.
    pub fn destination_local(&self, pose: Pose) -> Option<(f64, f64)> {
        // Kept through arrival so L5 sees it is there; withdrawn while paused.
        let m = self.mission.as_ref().filter(|_| self.phase != "paused")?;
        let (lat, lon) = self.guidance_position()?;
        let (distance, bearing) = distance_bearing(lat, lon, m.latitude, m.longitude);
        let yaw = self.heading.local_yaw(bearing)?;
        Some((pose.x + distance * yaw.cos(), pose.y + distance * yaw.sin()))
    }
    /// Latitude, longitude and rough one-sigma accuracy in metres of local point
    /// (`x`, `y`) seen from rover pose `from`. `None` unless the GPS fix is fresh
    /// and within 10 m and the fused heading is fresh and within 35°.
    pub fn geolocate(&self, now: f64, from: Pose, x: f64, y: f64) -> Option<(f64, f64, f64)> {
        let g = self
            .gps
            .filter(|g| fresh(now, g.timestamp, 3.) && g.accuracy <= 10.)?;
        let (_, heading_accuracy, age) = self.heading.heading(now)?;
        if !(0.0..=2.).contains(&age) || heading_accuracy > 35. {
            return None;
        }
        let (dx, dy) = (x - from.x, y - from.y);
        let distance = dx.hypot(dy);
        if !distance.is_finite() || distance > 20. {
            return None;
        }
        let bearing = self
            .heading
            .bearing_of_local_yaw(dy.atan2(dx))?
            .to_radians();
        let (lat, lon) = destination(g.latitude, g.longitude, distance, bearing);
        let accuracy = g.accuracy + distance * heading_accuracy.to_radians().sin();
        coordinates(lat, lon).then_some((lat, lon, accuracy))
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
            .heading(now)
            .is_none_or(|(_, accuracy, age)| !(0.0..=2.).contains(&age) || accuracy > 35.)
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
        if self.phase == "paused" {
            return None;
        }
        let g = self.gps?;
        let m = self.mission.as_mut()?;
        let (distance, _) = distance_bearing(g.latitude, g.longitude, m.latitude, m.longitude);
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
        let (to_lat, to_lon) = (m.latitude, m.longitude);
        self.phase = "planning";
        if self.last_guidance.is_some_and(|t| now - t < 0.5) {
            return None;
        }
        self.last_guidance = Some(now);
        // Steer from the smoothed position so a GPS jump does not swing the target.
        let (lat, lon) = self.guidance_position()?;
        let (distance, bearing) = distance_bearing(lat, lon, to_lat, to_lon);
        // Local yaw is counter-clockwise; true bearing is clockwise.
        let yaw = self.heading.local_yaw(bearing)?;
        let lookahead = distance.min(LOOKAHEAD_M);
        Some((
            e.pose.x + lookahead * yaw.cos(),
            e.pose.y + lookahead * yaw.sin(),
        ))
    }
    pub fn status(&self, now: f64, e: Evidence, run: &str) -> Value {
        let radius = self.mission.as_ref().map_or(self.radius, |m| m.radius);
        let reason = self.hold.or_else(|| self.readiness(now, e, radius));
        let heading = self.heading.heading(now);
        json!({"schema_version":1,"capability":"outdoor_gps","supported":self.supported,"acknowledged":self.selected,
   "source":if self.selected {"outdoor_gps"} else {"local"},"run_id":run,"navigation_revision":self.revision,
   "ready":reason.is_none() && self.phase!="paused", "hold_reason":reason,"phase":self.phase,"arrival_radius_m":radius,
   "gps":self.gps.map(|g|json!({"latitude":g.latitude,"longitude":g.longitude,"accuracy_m":g.accuracy,"age_s":now-g.timestamp})),
   "gps_age_s":self.gps.map(|g|now-g.timestamp),"gps_accuracy_m":self.gps.map(|g|g.accuracy),
   "heading_age_s":heading.map(|h|h.2),"heading_accuracy_deg":heading.map(|h|h.1),"heading_deg":heading.map(|h|h.0),
   "heading_source":self.heading.source().map(|s|s.name()),"heading_correction_age_s":self.heading.correction_age(now),
   "destination":self.mission.as_ref().map(|m|json!({"latitude":m.latitude,"longitude":m.longitude})),
   "remaining_m":self.remaining,"token":self.token,"result":self.result,"goal_levels":GOAL_LEVELS,
   "destination_local":self.destination_local(e.pose).map(|(x,y)|json!({"x":x,"y":y})),
   "anchor":self.anchor.position().map(|(_,_,sigma)|json!({"sigma_m":sigma,"reanchors":self.anchor.reanchors,"innovation_m":self.anchor.last_innovation})),
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
