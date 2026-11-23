//! The path is the goal itself: no obstacle search. For open ground and the
//! roadmap demos, where the local planner does all the work.
use super::PathPlanner;
use crate::geometry::Pose2;
use crate::planning::registry::PlannerSpec;
use crate::planning::{Goal, Path};
use crate::{estimation::State, mapping::Map, vehicle::VehicleModel};

pub struct StraightLine;

impl StraightLine {
    pub fn spec() -> PlannerSpec {
        PlannerSpec {
            id: "straight_line",
            version: "1",
            description: "The goal itself, no search",
            params: vec![],
            make: |_| Box::new(StraightLine),
        }
    }
}

impl PathPlanner for StraightLine {
    fn plan(
        &mut self,
        state: &State,
        _map: &Map,
        goal: &Goal,
        _vehicle: &dyn VehicleModel,
    ) -> Option<Path> {
        let Goal::Pose(goal) = goal else {
            return Some(Path::default());
        };
        // Two poses: where the rover is, and the goal. A goal without a
        // heading is reached facing along the line.
        let start = state.pose;
        let along_the_line = (goal.y - start.y).atan2(goal.x - start.x);
        let end = Pose2 {
            x: goal.x,
            y: goal.y,
            yaw: goal.yaw.unwrap_or(along_the_line),
        };
        Some(Path { poses: vec![start, end] })
    }
}
