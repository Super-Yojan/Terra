//! No base station: drops everything. For tests and offline runs.
use super::{BaseStationLink, BaseStationMsg};
use crate::geometry::Vec3;

pub struct NoLink;

impl BaseStationLink for NoLink {
    fn publish_depth(&mut self, t: f64, points: &[Vec3]) {}
    fn poll(&mut self) -> Option<BaseStationMsg> {
        None
    }
}
