//! Map of detected objects (buckets, balls) in the local frame, with GPS
//! coordinates when the outdoor fix and heading are good enough.
use serde::Serialize;

/// Sightings closer than this, of the same class and colour, are one object.
pub const MERGE_RADIUS_M: f64 = 1.0;
/// Distinct camera frames needed before an object counts as found.
pub const CONFIRM_FRAMES: u32 = 3;
/// Detections below this confidence are ignored.
pub const MIN_CONFIDENCE: f64 = 0.5;
/// Oldest unconfirmed objects are dropped beyond this many.
pub const MAX_OBJECTS: usize = 64;

/// One detector sighting already placed in the local world frame.
#[derive(Clone, Debug)]
pub struct ObjectSighting {
    /// Class token, for example `bucket` or `ball`.
    pub class: String,
    /// Optional colour token, for example `blue` or `orange`.
    pub color: Option<String>,
    pub confidence: f64,
    pub frame_id: u64,
    pub timestamp: f64,
    /// Local world position, metres.
    pub x: f64,
    pub y: f64,
    /// Latitude, longitude and one-sigma horizontal accuracy in metres, when outdoor.
    pub geo: Option<(f64, f64, f64)>,
}

#[derive(Clone, Debug, Serialize, PartialEq)]
pub struct MappedObject {
    pub id: u32,
    pub class: String,
    pub color: Option<String>,
    /// Mean local position of all sightings, metres.
    pub x: f64,
    pub y: f64,
    pub latitude: Option<f64>,
    pub longitude: Option<f64>,
    /// Best single-sighting GPS accuracy, metres.
    pub geo_accuracy_m: Option<f64>,
    pub confidence: f64,
    pub sightings: u32,
    pub confirmed: bool,
    pub first_seen: f64,
    pub last_seen: f64,
    #[serde(skip)]
    last_frame: u64,
    #[serde(skip)]
    geo_sum: (f64, f64, u32),
}

#[derive(Default, Debug)]
pub struct ObjectMap {
    objects: Vec<MappedObject>,
    next_id: u32,
    /// Bumped on every change so callers publish only when something moved.
    pub revision: u64,
}

fn valid_token(s: &str) -> bool {
    (1..=32).contains(&s.len())
        && s.bytes()
            .all(|b| b.is_ascii_alphanumeric() || b"_-".contains(&b))
}

impl ObjectMap {
    pub fn objects(&self) -> &[MappedObject] {
        &self.objects
    }
    pub fn clear(&mut self) {
        if !self.objects.is_empty() {
            self.objects.clear();
            self.revision += 1;
        }
    }
    /// Merge one sighting into the nearest matching object, or start a new one.
    pub fn observe(&mut self, s: ObjectSighting) -> Result<(), &'static str> {
        if !valid_token(&s.class) || s.color.as_deref().is_some_and(|c| !valid_token(c)) {
            return Err("invalid object class");
        }
        if !(s.x.is_finite() && s.y.is_finite() && s.timestamp.is_finite()) {
            return Err("invalid object position");
        }
        if !s.confidence.is_finite() || s.confidence > 1. {
            return Err("invalid object confidence");
        }
        if s.confidence < MIN_CONFIDENCE {
            return Err("object confidence too low");
        }
        let geo = s.geo.filter(|(lat, lon, acc)| {
            lat.is_finite()
                && lon.is_finite()
                && (-85.0..=85.).contains(lat)
                && (-180.0..=180.).contains(lon)
                && acc.is_finite()
                && *acc >= 0.
        });
        let nearest = self
            .objects
            .iter_mut()
            .filter(|o| o.class == s.class && o.color == s.color)
            .map(|o| ((o.x - s.x).hypot(o.y - s.y), o))
            .filter(|(d, _)| *d <= MERGE_RADIUS_M)
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(_, o)| o);
        match nearest {
            Some(o) => {
                // A frame only counts once, so one frame of two boxes cannot confirm.
                if s.frame_id == o.last_frame {
                    return Ok(());
                }
                let n = f64::from(o.sightings);
                o.x = (o.x * n + s.x) / (n + 1.);
                o.y = (o.y * n + s.y) / (n + 1.);
                o.sightings += 1;
                o.last_frame = s.frame_id;
                o.last_seen = s.timestamp;
                o.confidence = o.confidence.max(s.confidence);
                o.confirmed = o.sightings >= CONFIRM_FRAMES;
                if let Some(g) = geo {
                    o.add_geo(g);
                }
            }
            None => {
                self.next_id += 1;
                let mut o = MappedObject {
                    id: self.next_id,
                    class: s.class,
                    color: s.color,
                    x: s.x,
                    y: s.y,
                    latitude: None,
                    longitude: None,
                    geo_accuracy_m: None,
                    confidence: s.confidence,
                    sightings: 1,
                    confirmed: CONFIRM_FRAMES <= 1,
                    first_seen: s.timestamp,
                    last_seen: s.timestamp,
                    last_frame: s.frame_id,
                    geo_sum: (0., 0., 0),
                };
                if let Some(g) = geo {
                    o.add_geo(g);
                }
                self.objects.push(o);
                if self.objects.len() > MAX_OBJECTS {
                    let drop = self
                        .objects
                        .iter()
                        .enumerate()
                        .filter(|(_, o)| !o.confirmed)
                        .min_by(|a, b| a.1.last_seen.total_cmp(&b.1.last_seen))
                        .map_or(0, |(i, _)| i);
                    self.objects.remove(drop);
                }
            }
        }
        self.revision += 1;
        Ok(())
    }
    /// Telemetry body for the `objects` topic.
    pub fn telemetry(&self, rover_id: u64, run_id: &str) -> serde_json::Value {
        serde_json::json!({
            "schema_version": 1,
            "rover_id": rover_id,
            "run_id": run_id,
            "revision": self.revision,
            "objects": self.objects,
        })
    }
}

impl MappedObject {
    fn add_geo(&mut self, (lat, lon, acc): (f64, f64, f64)) {
        let (a, b, n) = self.geo_sum;
        self.geo_sum = (a + lat, b + lon, n + 1);
        self.latitude = Some(self.geo_sum.0 / f64::from(self.geo_sum.2));
        self.longitude = Some(self.geo_sum.1 / f64::from(self.geo_sum.2));
        self.geo_accuracy_m = Some(self.geo_accuracy_m.map_or(acc, |a| a.min(acc)));
    }
}
