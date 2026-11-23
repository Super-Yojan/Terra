//! Stages 3 and 4 together for one pose goal: keep a route to the goal,
//! re-plan it when needed, and ask the controller for this tick's twist.
use super::global::PathPlanner;
use super::local::LocalPlanner;
use super::{Goal, Path, PoseGoal};
use crate::estimation::State;
use crate::mapping::Map;
use crate::vehicle::{Twist, VehicleModel};

/// When only the map changed, re-plan at most this often (s). A changed
/// goal always re-plans at once.
const REPLAN_PERIOD_S: f64 = 1.0;

/// The path planner found no route to the goal.
#[derive(Debug)]
pub struct NoRoute;

pub struct Navigator {
    pub path_planner: Box<dyn PathPlanner>,
    pub controller: Box<dyn LocalPlanner>,
    /// The newest plan, kept between ticks.
    plan: Option<Plan>,
}

struct Plan {
    /// What it was planned for.
    goal: PoseGoal,
    map_revision: u64,
    /// When (frame time, s).
    at: f64,
    /// `None`: the planner found no route.
    path: Option<Path>,
}

impl Navigator {
    pub fn new(path_planner: Box<dyn PathPlanner>, controller: Box<dyn LocalPlanner>) -> Self {
        Self { path_planner, controller, plan: None }
    }

    /// The current route (empty before the first plan, or with no route).
    pub fn path(&self) -> &[crate::geometry::Pose2] {
        match &self.plan {
            Some(Plan { path: Some(path), .. }) => &path.poses,
            _ => &[],
        }
    }

    /// Forget the route, so the next [`drive`](Self::drive) plans afresh.
    pub fn reset(&mut self) {
        self.plan = None;
    }

    /// This tick's twist toward `goal`: re-plan if needed (stage 3), then
    /// follow the route (stage 4).
    pub fn drive(
        &mut self,
        state: &State,
        map: &Map,
        goal: &PoseGoal,
        vehicle: &dyn VehicleModel,
    ) -> Result<Twist, NoRoute> {
        let replan = match &self.plan {
            None => true,
            Some(plan) if plan.goal != *goal => true,
            Some(plan) => {
                plan.map_revision != map.revision && state.t - plan.at >= REPLAN_PERIOD_S
            }
        };
        if replan {
            self.plan = Some(Plan {
                goal: *goal,
                map_revision: map.revision,
                at: state.t,
                path: self.path_planner.plan(state, map, &Goal::Pose(*goal), vehicle),
            });
        }

        match &self.plan {
            Some(Plan { path: Some(path), .. }) => {
                Ok(self.controller.follow(state, map, &Goal::Pose(*goal), path, vehicle))
            }
            _ => Err(NoRoute),
        }
    }

    /// The controller's time limit for driving to `goal` from here, along
    /// the current route (see `LocalPlanner::timeout`).
    pub fn timeout(&self, state: &State, goal: &PoseGoal, vehicle: &dyn VehicleModel) -> Option<f64> {
        let no_path = Path::default();
        let path = match &self.plan {
            Some(Plan { path: Some(path), .. }) => path,
            _ => &no_path,
        };
        self.controller.timeout(goal, state, path, vehicle)
    }
}
