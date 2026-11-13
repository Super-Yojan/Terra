//! Geographic mission state, independent of the local pose and occupancy frame.
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use terra_navigation::Pose;
use terra_types::timing::fresh;
use terra_waypoint::{EARTH_RADIUS_M, valid_coordinates as coordinates};
mod anchor;
mod heading;
pub use anchor::GeoAnchor;
pub use heading::{CourseFix, HeadingFusion, Source as HeadingSource};
/// Arrival radius when the operator does not choose one, in metres. GPS accuracy
/// no longer has to fit inside it: arrival uses the smoothed anchor position.
const DEFAULT_ARRIVAL_RADIUS_M: f64 = 3.;
/// Arrival radii an operator may choose, in metres.
pub const ARRIVAL_RADIUS_RANGE_M: std::ops::RangeInclusive<f64> = 2.0..=20.0;
/// Guidance lookahead along the destination bearing, in metres.
const LOOKAHEAD_M: f64 = 3.;
/// Oldest GPS fix guidance or geolocation uses, in seconds.
const GPS_FRESHNESS_SECONDS: f64 = 3.;
/// Worst GPS accuracy guidance or geolocation accepts, in metres.
const GPS_MAX_ACCURACY_M: f64 = 10.;
/// Oldest compass sample, and oldest fused heading, guidance uses, in seconds.
const HEADING_FRESHNESS_SECONDS: f64 = 2.;
/// Worst fused heading accuracy guidance or geolocation accepts, in degrees.
const HEADING_MAX_ACCURACY_DEG: f64 = 35.;
/// Farthest local point [`OutdoorController::geolocate`] places, in metres.
const GEOLOCATE_MAX_DISTANCE_M: f64 = 20.;
/// Farthest destination a mission may start toward, in metres.
const MAX_GOAL_DISTANCE_M: f64 = 500.;
/// Distinct GPS fixes inside the arrival radius that confirm arrival.
const ARRIVAL_FIXES: u8 = 3;
/// Shortest interval between guidance target updates, in seconds.
const GUIDANCE_INTERVAL_SECONDS: f64 = 0.5;

/// Where navigation goals come from: the local frame, or GPS destinations.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NavigationSource {
    #[default]
    Local,
    OutdoorGps,
}
impl NavigationSource {
    /// Wire name: `local` or `outdoor_gps`.
    pub fn as_str(self) -> &'static str {
        match self {
            NavigationSource::Local => "local",
            NavigationSource::OutdoorGps => "outdoor_gps",
        }
    }
    /// The source with this wire name.
    pub fn parse(name: &str) -> Option<Self> {
        [NavigationSource::Local, NavigationSource::OutdoorGps]
            .into_iter()
            .find(|s| s.as_str() == name)
    }
}

/// Where a geographic mission stands.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum OutdoorPhase {
    /// No mission.
    #[default]
    Idle,
    /// Guiding toward the destination.
    Planning,
    /// Held until an explicit resume (sensors, authority or operator).
    Paused,
    /// Inside the arrival radius, waiting for more fixes to agree.
    ConfirmingArrival,
    Arrived,
}
impl OutdoorPhase {
    /// Wire name, such as `confirming_arrival`.
    pub fn as_str(self) -> &'static str {
        match self {
            OutdoorPhase::Idle => "idle",
            OutdoorPhase::Planning => "planning",
            OutdoorPhase::Paused => "paused",
            OutdoorPhase::ConfirmingArrival => "confirming_arrival",
            OutdoorPhase::Arrived => "arrived",
        }
    }
}
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
    /// The platform measures GPS and a mount-aware true heading.
    supported: bool,
    source: NavigationSource,
    /// Arrival radius for the next mission, in metres.
    radius: f64,
    gps: Option<GpsFix>,
    gps_time: Option<f64>,
    heading_time: Option<f64>,
    heading: HeadingFusion,
    anchor: GeoAnchor,
    mission: Option<Mission>,
    phase: OutdoorPhase,
    hold: Option<&'static str>,
    last_guidance: Option<f64>,
    remaining: Option<f64>,
}
impl Default for OutdoorController {
    fn default() -> Self {
        Self {
            supported: false,
            source: NavigationSource::Local,
            radius: DEFAULT_ARRIVAL_RADIUS_M,
            gps: None,
            gps_time: None,
            heading_time: None,
            heading: HeadingFusion::default(),
            anchor: GeoAnchor::default(),
            mission: None,
            phase: OutdoorPhase::Idle,
            hold: None,
            last_guidance: None,
            remaining: None,
        }
    }
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
    (2. * EARTH_RADIUS_M * h.sqrt().asin(), bearing)
}
/// Point `distance` metres from (`lat`, `lon`) along true bearing `bearing` radians.
pub fn destination(lat: f64, lon: f64, distance: f64, bearing: f64) -> (f64, f64) {
    let (a, l) = (lat.to_radians(), lon.to_radians());
    let d = distance / EARTH_RADIUS_M;
    let b = (a.sin() * d.cos() + a.cos() * d.sin() * bearing.cos()).asin();
    let m = l + (bearing.sin() * d.sin() * a.cos()).atan2(d.cos() - a.sin() * b.sin());
    (
        b.to_degrees(),
        (m.to_degrees() + 540.).rem_euclid(360.) - 180.,
    )
}
impl OutdoorController {
    /// Accept outdoor GPS navigation (or not), for platforms with measured
    /// GPS and mount-aware true heading.
    pub fn set_supported(&mut self, supported: bool) {
        self.supported = supported;
    }
    pub fn supported(&self) -> bool {
        self.supported
    }
    /// Whether goals come from GPS destinations.
    pub fn selected(&self) -> bool {
        self.source == NavigationSource::OutdoorGps
    }
    /// Arrival radius for the next mission, in metres.
    pub fn arrival_radius(&self) -> f64 {
        self.radius
    }
    /// Switch the goal source and the arrival radius for later missions,
    /// cancelling any mission. Rejects a radius outside
    /// [`ARRIVAL_RADIUS_RANGE_M`] and an unsupported GPS source, changing nothing.
    pub fn select_source(
        &mut self,
        source: NavigationSource,
        radius: f64,
    ) -> Result<(), &'static str> {
        if source == NavigationSource::OutdoorGps && !self.supported {
            return Err("outdoor GPS unsupported");
        }
        if !ARRIVAL_RADIUS_RANGE_M.contains(&radius) {
            return Err("invalid arrival radius");
        }
        self.cancel();
        self.source = source;
        self.radius = radius;
        Ok(())
    }
    /// Back to local goals, cancelling any mission; the radius stays.
    pub fn select_local(&mut self) {
        self.cancel();
        self.source = NavigationSource::Local;
    }
    pub fn invalidate_sensors(&mut self) {
        self.gps = None;
        self.heading.reset();
        self.anchor.reset();
        self.pause("tracking_lost");
    }
    pub fn phase(&self) -> OutdoorPhase {
        self.phase
    }
    pub fn active(&self) -> bool {
        self.mission.is_some() && self.phase != OutdoorPhase::Arrived
    }
    /// Sensor recovery restores readiness, but never releases the explicit resume latch.
    pub fn recovery_ready(&self, now: f64, e: Evidence) -> bool {
        e.authority && self.readiness(now, e).is_none()
    }
    pub fn gps(&mut self, s: impl Into<GpsFix>, now: f64) -> Result<(), &'static str> {
        let s = s.into();
        if !coordinates(s.latitude, s.longitude)
            || !s.accuracy.is_finite()
            || s.accuracy < 0.
            || !fresh(now, Some(s.timestamp), GPS_FRESHNESS_SECONDS)
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
            || !fresh(now, Some(s.timestamp), HEADING_FRESHNESS_SECONDS)
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
        let m = self.mission.as_ref().filter(|_| self.phase != OutdoorPhase::Paused)?;
        let (lat, lon) = self.guidance_position()?;
        let (distance, bearing) = distance_bearing(lat, lon, m.latitude, m.longitude);
        let yaw = self.heading.local_yaw(bearing)?;
        Some((pose.x + distance * yaw.cos(), pose.y + distance * yaw.sin()))
    }
    /// Latitude, longitude and rough one-sigma accuracy in metres of local point
    /// (`x`, `y`) seen from rover pose `from`. `None` unless the GPS fix is fresh
    /// and accurate enough, the fused heading is fresh and accurate enough, and
    /// the point is within 20 m.
    pub fn geolocate(&self, now: f64, from: Pose, x: f64, y: f64) -> Option<(f64, f64, f64)> {
        let g = self.gps.filter(|g| self.gps_good(now, g))?;
        let (_, heading_accuracy, _) = self.heading_good(now)?;
        let (dx, dy) = (x - from.x, y - from.y);
        let distance = dx.hypot(dy);
        if !distance.is_finite() || distance > GEOLOCATE_MAX_DISTANCE_M {
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
    fn gps_good(&self, now: f64, g: &GpsFix) -> bool {
        fresh(now, Some(g.timestamp), GPS_FRESHNESS_SECONDS) && g.accuracy <= GPS_MAX_ACCURACY_M
    }
    /// The fused heading, when fresh and accurate enough to steer by.
    fn heading_good(&self, now: f64) -> Option<(f64, f64, f64)> {
        self.heading.heading(now).filter(|&(_, accuracy, age)| {
            (0.0..HEADING_FRESHNESS_SECONDS).contains(&age)
                && accuracy <= HEADING_MAX_ACCURACY_DEG
        })
    }
    pub fn readiness(&self, now: f64, e: Evidence) -> Option<&'static str> {
        if self
            .gps
            .is_none_or(|g| !fresh(now, Some(g.timestamp), GPS_FRESHNESS_SECONDS))
        {
            Some("gps_stale")
        } else if self.gps.is_some_and(|g| !self.gps_good(now, &g)) {
            Some("gps_uncertain")
        } else if self.heading_good(now).is_none() {
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
            || !ARRIVAL_RADIUS_RANGE_M.contains(&radius)
            || !crate::valid_token(&token)
        {
            return Err("invalid_goal");
        }
        if let Some(why) = self.readiness(now, e) {
            return Err(why);
        }
        if !e.authority {
            return Err("authority_lost");
        }
        let g = self.gps.unwrap();
        if distance_bearing(g.latitude, g.longitude, lat, lon).0 > MAX_GOAL_DISTANCE_M {
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
        self.phase = OutdoorPhase::Planning;
        self.hold = None;
        self.last_guidance = None;
        self.remaining = None;
        Ok(())
    }
    pub fn pause(&mut self, reason: &'static str) {
        if self.active() {
            self.phase = OutdoorPhase::Paused;
            self.hold = Some(reason);
            self.last_guidance = None;
            if let Some(m) = self.mission.as_mut() {
                m.count = 0;
            }
        }
    }
    pub fn resume(&mut self, now: f64, e: Evidence) -> Result<(), &'static str> {
        self.mission.as_ref().ok_or("no_mission")?;
        if self.phase != OutdoorPhase::Paused {
            return Err("not_paused");
        }
        if let Some(why) = self.readiness(now, e) {
            return Err(why);
        }
        if !e.authority {
            return Err("authority_lost");
        }
        self.phase = OutdoorPhase::Planning;
        self.hold = None;
        self.last_guidance = None;
        Ok(())
    }
    pub fn cancel(&mut self) {
        self.mission = None;
        self.phase = OutdoorPhase::Idle;
        self.hold = None;
        self.last_guidance = None;
        self.remaining = None;
    }
    /// A changed guidance target at most twice a second. No pose/map mutation.
    pub fn tick(&mut self, now: f64, e: Evidence) -> Option<(f64, f64)> {
        if !self.active() {
            return None;
        }
        if let Some(why) = self.readiness(now, e) {
            self.pause(why);
            return None;
        }
        if !e.authority {
            self.pause("authority_lost");
            return None;
        }
        if self.phase == OutdoorPhase::Paused {
            return None;
        }
        let g = self.gps?;
        // Arrival and remaining distance use the smoothed position too, so a fix
        // a few metres off neither ends the mission early nor keeps it going.
        let (lat, lon) = self.guidance_position()?;
        let m = self.mission.as_mut()?;
        let (distance, _) = distance_bearing(lat, lon, m.latitude, m.longitude);
        self.remaining = Some(distance);
        if distance <= m.radius {
            if m.last_fix != Some(g.timestamp) {
                m.count += 1;
                m.last_fix = Some(g.timestamp);
            }
            self.phase = if m.count >= ARRIVAL_FIXES {
                OutdoorPhase::Arrived
            } else {
                OutdoorPhase::ConfirmingArrival
            };
            self.last_guidance = None;
            return None;
        }
        m.count = 0;
        m.last_fix = Some(g.timestamp);
        let (to_lat, to_lon) = (m.latitude, m.longitude);
        self.phase = OutdoorPhase::Planning;
        if self
            .last_guidance
            .is_some_and(|t| now - t < GUIDANCE_INTERVAL_SECONDS)
        {
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
    /// Status for the `outdoor` topic, without the request bookkeeping the
    /// host adds (`navigation_revision`, `token`, `result`).
    pub fn status(&self, now: f64, e: Evidence, run: &str) -> Value {
        let radius = self.mission.as_ref().map_or(self.radius, |m| m.radius);
        let reason = self.hold.or_else(|| self.readiness(now, e));
        let heading = self.heading.heading(now);
        json!({"schema_version":1,"capability":"outdoor_gps","supported":self.supported,"acknowledged":self.selected(),
   "source":self.source,"run_id":run,
   "ready":reason.is_none() && self.phase!=OutdoorPhase::Paused, "hold_reason":reason,"phase":self.phase,"arrival_radius_m":radius,
   "gps":self.gps.map(|g|json!({"latitude":g.latitude,"longitude":g.longitude,"accuracy_m":g.accuracy,"age_s":now-g.timestamp})),
   "gps_age_s":self.gps.map(|g|now-g.timestamp),"gps_accuracy_m":self.gps.map(|g|g.accuracy),
   "heading_age_s":heading.map(|h|h.2),"heading_accuracy_deg":heading.map(|h|h.1),"heading_deg":heading.map(|h|h.0),
   "heading_source":self.heading.source().map(|s|s.name()),"heading_correction_age_s":self.heading.correction_age(now),
   "destination":self.mission.as_ref().map(|m|json!({"latitude":m.latitude,"longitude":m.longitude})),
   "remaining_m":self.remaining,"goal_levels":crate::Level::ALL.iter().filter(|l| l.accepts_outdoor_goal()).map(|l| l.as_str()).collect::<Vec<_>>(),
   "destination_local":self.destination_local(e.pose).map(|(x,y)|json!({"x":x,"y":y})),
   "anchor":self.anchor.position().map(|(_,_,sigma)|json!({"sigma_m":sigma,"reanchors":self.anchor.reanchors,"innovation_m":self.anchor.last_innovation})),
   "mission_token":self.mission.as_ref().map(|m|&m.token)})
    }
    pub fn mission_status(&self) -> Option<terra_waypoint::WaypointStatus> {
        let m = self.mission.as_ref()?;
        let mut s = terra_waypoint::WaypointStatus::idle();
        s.state = if self.phase == OutdoorPhase::Arrived {
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
