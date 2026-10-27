//! L5 task autonomy: an on-device language model turns an operator task into waypoints.
//!
//! ARGOS sends a task in plain language (`task`, or inline on the `task` level
//! request). The arbiter builds a [`TaskQuery`] from the phone's occupancy map:
//! a 1 m symbolic grid, reachable frontiers, obstacle blobs and landmarks, each
//! with an id. The host runs the model and returns its text with
//! [`crate::AutonomyArbiter::submit_task_response`]. The reply is one JSON
//! object choosing ids or local points. Every waypoint is resolved to observed,
//! footprint-clear free space and checked with A* before the waypoint follower
//! latches it; the L3 route, local planner and safety holds then drive as usual.
//! The model never produces a velocity, effort or servo command, and a reply
//! that names one is rejected.

use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use terra_mapping::MapSnapshot;
use terra_navigation::{Pose, astar_route, footprint_clear};

use crate::valid_token;

/// Longest accepted task instruction or operator reply, in characters.
pub const TASK_TEXT_MAX_CHARS: usize = 500;
/// Budget used when a task omits `budget_seconds`.
pub const TASK_DEFAULT_BUDGET_SECONDS: f64 = 300.;
/// Largest accepted task budget.
pub const TASK_MAX_BUDGET_SECONDS: f64 = 3600.;
/// Model calls allowed when a task omits `max_steps`.
pub const TASK_DEFAULT_MAX_STEPS: u32 = 20;
/// Largest accepted `max_steps`.
pub const TASK_MAX_STEPS: u32 = 100;
/// Waypoints taken from one reply; extra entries are dropped.
pub const TASK_MAX_WAYPOINTS: usize = 5;
/// Largest model reply the arbiter parses.
pub const TASK_RESPONSE_MAX_BYTES: usize = 8192;
/// A query with no answer after this long counts as a failed step.
pub const TASK_QUERY_TIMEOUT_SECONDS: f64 = 90.;
/// A route failure lasting this long abandons the active waypoint and asks the model again.
pub const TASK_BLOCKED_SECONDS: f64 = 4.;
/// A waypoint not reached in this long is abandoned and the model is asked again.
pub const TASK_WAYPOINT_TIMEOUT_SECONDS: f64 = 90.;
/// Consecutive invalid replies, timeouts or model errors that fail the task.
pub const TASK_MAX_CONSECUTIVE_FAILURES: u32 = 3;
/// Landmarks older than this are left out of the prompt.
pub const TASK_LANDMARK_MAX_AGE_SECONDS: f64 = 120.;
/// Free point snapped to observed, clear space within this distance.
pub const TASK_SNAP_RADIUS: f64 = 0.5;
/// History entries kept in status; the prompt shows the most recent six.
const HISTORY_LIMIT: usize = 12;
const PROMPT_HISTORY: usize = 6;
const MAX_FRONTIERS: usize = 6;
const MAX_OBSTACLES: usize = 8;
const MAX_LANDMARKS: usize = 8;

/// Task text, budget and step limit shared by the `task` topic and the inline level request.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct TaskSpec {
    /// Operator-chosen task id; command-token alphabet.
    pub task_id: String,
    /// Plain-language task, 1–500 characters.
    pub instruction: String,
    /// Wall budget for the whole task, `(0, 3600]` seconds. Default 300.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub budget_seconds: Option<f64>,
    /// Model calls allowed, 1–100. Default 20.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_steps: Option<u32>,
}

impl TaskSpec {
    /// Whether ids, text, budget and step limit are in range.
    pub fn valid(&self) -> bool {
        valid_token(&self.task_id)
            && valid_text(&self.instruction)
            && self
                .budget_seconds
                .is_none_or(|s| s.is_finite() && s > 0. && s <= TASK_MAX_BUDGET_SECONDS)
            && self
                .max_steps
                .is_none_or(|n| (1..=TASK_MAX_STEPS).contains(&n))
    }
    pub fn budget(&self) -> f64 {
        self.budget_seconds.unwrap_or(TASK_DEFAULT_BUDGET_SECONDS)
    }
    pub fn steps(&self) -> u32 {
        self.max_steps.unwrap_or(TASK_DEFAULT_MAX_STEPS)
    }
}

/// `task` topic: start a new task while the rover is already at the `task` level.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct TaskRequest {
    #[serde(flatten)]
    pub spec: TaskSpec,
    /// Request correlation token.
    pub token: String,
    /// Run the request was made against. Checked when present.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub run_id: Option<String>,
}

/// What the operator does to a running task.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum TaskActionKind {
    /// End the task and hold. The level stays `task`.
    Cancel,
    /// Answer the model's question, or add guidance, then plan again.
    Reply,
}

/// `task/action` topic.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct TaskAction {
    pub task_id: String,
    pub action: TaskActionKind,
    /// Reply text, required for `reply`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub text: Option<String>,
    pub token: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub run_id: Option<String>,
}

fn valid_text(text: &str) -> bool {
    let trimmed = text.trim();
    !trimmed.is_empty()
        && text.chars().count() <= TASK_TEXT_MAX_CHARS
        && !text.chars().any(|c| c.is_control() && c != '\n')
}

fn decode<T: serde::de::DeserializeOwned>(b: &[u8]) -> Option<T> {
    if b.is_empty() || b.len() > 2048 {
        return None;
    }
    serde_json::from_slice(b).ok()
}

/// Decode at most 2048 bytes of strict JSON; `None` for invalid ids, text, budget or fields.
pub fn decode_task_request(b: &[u8]) -> Option<TaskRequest> {
    decode::<TaskRequest>(b).filter(|r| {
        r.spec.valid() && valid_token(&r.token) && r.run_id.as_ref().is_none_or(|s| valid_token(s))
    })
}

/// Decode at most 2048 bytes of strict JSON; `reply` needs 1–500 characters of text.
pub fn decode_task_action(b: &[u8]) -> Option<TaskAction> {
    decode::<TaskAction>(b).filter(|a| {
        valid_token(&a.task_id)
            && valid_token(&a.token)
            && a.run_id.as_ref().is_none_or(|s| valid_token(s))
            && match a.action {
                TaskActionKind::Reply => a.text.as_deref().is_some_and(valid_text),
                TaskActionKind::Cancel => a.text.as_deref().is_none_or(valid_text),
            }
    })
}

/// Task lifecycle shown on `task/status`.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum TaskPhase {
    /// Waiting for the model's next decision. The rover holds.
    Planning,
    /// Driving the model's waypoints through the L3 route.
    Navigating,
    /// The model asked the operator a question. The rover holds.
    WaitingOperator,
    /// The model said the task is done.
    Completed,
    /// The model gave up, or a budget or failure limit ended the task.
    Failed,
    /// The operator, a stop or a level change ended the task.
    Cancelled,
}

impl TaskPhase {
    pub fn terminal(self) -> bool {
        matches!(self, Self::Completed | Self::Failed | Self::Cancelled)
    }
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Planning => "planning",
            Self::Navigating => "navigating",
            Self::WaitingOperator => "waiting_operator",
            Self::Completed => "completed",
            Self::Failed => "failed",
            Self::Cancelled => "cancelled",
        }
    }
}

/// One validated waypoint of the current plan, in local metres.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct TaskWaypoint {
    pub x: f64,
    pub y: f64,
    /// Map id the model chose (`F2`, `O1`, `L1`) or `point`.
    pub label: String,
}

/// One line of the task's memory, shown to the operator and to the model.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct TaskHistoryEntry {
    /// Model call this entry belongs to.
    pub step: u32,
    /// `decision`, `arrived`, `blocked`, `timeout`, `invalid`, `model_error`, `operator`.
    pub kind: String,
    pub text: String,
}

/// `task/status`, also nested on `autonomy/status` as `task`.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct TaskStatus {
    pub run_id: String,
    pub task_id: String,
    pub instruction: String,
    pub phase: TaskPhase,
    /// Machine-readable detail, such as `awaiting_model`, `route_blocked` or `budget_exhausted`.
    pub reason: String,
    /// Model calls used.
    pub step: u32,
    pub max_steps: u32,
    pub elapsed_seconds: f64,
    pub budget_seconds: f64,
    pub remaining_seconds: f64,
    /// Name the host gave for its model.
    pub model: Option<String>,
    /// Outstanding query, if one is waiting for the model.
    pub query_id: Option<u64>,
    /// Seconds the outstanding query has waited.
    pub waiting_seconds: Option<f64>,
    /// Model time of the last answered query, as reported by the host.
    pub last_latency_seconds: Option<f64>,
    /// Remaining validated waypoints; the first is active while navigating.
    pub plan: Vec<TaskWaypoint>,
    /// Question for the operator while waiting.
    pub question: Option<String>,
    /// The model's last explanation.
    pub rationale: Option<String>,
    pub last_error: Option<String>,
    pub history: Vec<TaskHistoryEntry>,
}

/// One request for the model. The host answers with the same `query_id`.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct TaskQuery {
    pub query_id: u64,
    pub task_id: String,
    /// Fixed rules: the reply format and what the rover can and cannot do.
    pub system: String,
    /// Task, rover pose, map summary and history for this step.
    pub prompt: String,
}

/// Rules sent as the system prompt on every query.
pub fn task_system_prompt() -> String {
    "You plan for Terra, a small ground rover with an iPhone for sensing. \
An operator gives you a task. Each step you see the rover's local map and decide what to do next.\n\
\n\
How the rover moves: you choose waypoints. The onboard planner drives to them with A*, avoids obstacles \
and stops for safety. You never send speeds, efforts or motor commands. A waypoint must be free space \
the rover has already seen. Unknown space ('?') cannot be a waypoint; to see more, go to a frontier (F ids).\n\
\n\
Coordinates are metres in the rover's local map frame. Heading 0 deg points along +x and angles grow \
counterclockwise toward +y. Bearings in the lists are relative to the rover: +90 is left, -90 is right.\n\
\n\
Reply with exactly one JSON object and nothing else:\n\
{\"decision\":\"go\",\"waypoints\":[{\"target\":\"F1\"}],\"rationale\":\"short reason\"}\n\
decision is one of:\n\
- \"go\": 1 to 5 waypoints, visited in order. Each is {\"target\":\"<id from the lists>\"} or {\"x\":1.5,\"y\":-2.0}. Prefer ids.\n\
- \"ask_operator\": put a short question in \"question\". The rover waits for the answer.\n\
- \"done\": the task is finished. Say why in \"rationale\".\n\
- \"fail\": the task cannot be done. Say why in \"rationale\".\n\
If a GPS DESTINATION (G1) is listed, it is the operator's final goal: work toward it, through G1 itself \
or frontiers on its side, unless the task says otherwise.\n\
After the waypoints are reached, blocked or abandoned you are asked again with the outcome in HISTORY."
        .into()
}

/// Map id the model may name, resolved to a validated approach point.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct TaskTarget {
    pub id: String,
    pub approach: Option<[f64; 2]>,
}

/// A named point the host saw, such as a detector hit, in local metres.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct TaskLandmark {
    pub label: String,
    pub x: f64,
    pub y: f64,
    pub confidence: f64,
    pub seen_at: f64,
}

/// Prompt text and the ids it named.
pub(crate) struct MapSummary {
    pub text: String,
    pub targets: Vec<TaskTarget>,
}

/// Cells whose footprint the rover can reach from `pose`, with BFS distance in cells.
pub(crate) struct Reach {
    width: usize,
    height: usize,
    /// BFS distance in cells, or `u32::MAX` when unreachable.
    distance: Vec<u32>,
    /// Footprint fully observed and clear: a valid goal cell.
    goal: Vec<bool>,
}

impl Reach {
    pub fn build(map: &MapSnapshot, pose: Pose, radius: f64) -> Option<Self> {
        let width = map.width as usize;
        let height = map.height as usize;
        if width == 0 || height == 0 || map.occupancy.len() != width * height || !pose.finite() {
            return None;
        }
        let start = terra_navigation::grid_cell(map, pose.x, pose.y)?;
        let center = |c: usize, r: usize| {
            (
                map.origin_x + (c as f64 + 0.5) * map.resolution,
                map.origin_y + (r as f64 + 0.5) * map.resolution,
            )
        };
        let mut distance = vec![u32::MAX; width * height];
        let mut goal = vec![false; width * height];
        let mut seen = vec![false; width * height];
        let mut queue = VecDeque::from([(start.0, start.1, 0u32)]);
        seen[start.1 * width + start.0] = true;
        while let Some((c, r, d)) = queue.pop_front() {
            let index = r * width + c;
            let (x, y) = center(c, r);
            // The rover's own cell is the start even when its footprint touches unknown.
            if (c, r) != start && !footprint_clear(map, x, y, radius, Some(pose)) {
                continue;
            }
            distance[index] = d;
            goal[index] = footprint_clear(map, x, y, radius, None);
            for (dx, dy) in [(1isize, 0isize), (-1, 0), (0, 1), (0, -1)] {
                let nc = c as isize + dx;
                let nr = r as isize + dy;
                if nc < 0 || nr < 0 || nc >= width as isize || nr >= height as isize {
                    continue;
                }
                let n = nr as usize * width + nc as usize;
                if !seen[n] {
                    seen[n] = true;
                    queue.push_back((nc as usize, nr as usize, d + 1));
                }
            }
        }
        Some(Self {
            width,
            height,
            distance,
            goal,
        })
    }
    fn goal_cell(&self, index: usize) -> bool {
        self.goal[index] && self.distance[index] != u32::MAX
    }
    /// Nearest reachable goal cell to `(x, y)` within `within` metres, ties broken by travel distance.
    pub fn nearest_goal(&self, map: &MapSnapshot, x: f64, y: f64, within: f64) -> Option<[f64; 2]> {
        self.nearest_goal_to(map, &[(x, y)], within)
    }
    fn nearest_goal_to(
        &self,
        map: &MapSnapshot,
        points: &[(f64, f64)],
        within: f64,
    ) -> Option<[f64; 2]> {
        if points.is_empty() || !within.is_finite() || within < 0. {
            return None;
        }
        let (min_x, max_x, min_y, max_y) = points.iter().fold(
            (f64::MAX, f64::MIN, f64::MAX, f64::MIN),
            |(a, b, c, d), &(x, y)| (a.min(x), b.max(x), c.min(y), d.max(y)),
        );
        let col = |x: f64| ((x - map.origin_x) / map.resolution).floor() as isize;
        let row = |y: f64| ((y - map.origin_y) / map.resolution).floor() as isize;
        let c0 = col(min_x - within).max(0);
        let c1 = col(max_x + within).min(self.width as isize - 1);
        let r0 = row(min_y - within).max(0);
        let r1 = row(max_y + within).min(self.height as isize - 1);
        let mut cells: Vec<(f64, u32, [f64; 2])> = Vec::new();
        for r in r0..=r1 {
            for c in c0..=c1 {
                let index = r as usize * self.width + c as usize;
                if !self.goal_cell(index) {
                    continue;
                }
                let cx = map.origin_x + (c as f64 + 0.5) * map.resolution;
                let cy = map.origin_y + (r as f64 + 0.5) * map.resolution;
                let d = points
                    .iter()
                    .map(|&(x, y)| (cx - x).hypot(cy - y))
                    .fold(f64::MAX, f64::min);
                if d <= within {
                    cells.push((d, self.distance[index], [cx, cy]));
                }
            }
        }
        // Within 0.3 m of the closest clearance, prefer the shortest drive, so the
        // rover stops on the side it approaches from rather than at a far corner.
        let closest = cells.iter().map(|c| c.0).fold(f64::MAX, f64::min);
        let best = cells
            .into_iter()
            .filter(|c| c.0 <= closest + 0.3)
            .min_by(|a, b| a.1.cmp(&b.1).then(a.0.total_cmp(&b.0)));
        best.map(|b| [tidy(b.2[0]), tidy(b.2[1])])
    }
    fn travel(&self, map: &MapSnapshot, x: f64, y: f64) -> Option<f64> {
        let (c, r) = terra_navigation::grid_cell(map, x, y)?;
        let d = self.distance[r * self.width + c];
        (d != u32::MAX).then(|| f64::from(d) * map.resolution)
    }
}

fn tidy(value: f64) -> f64 {
    let rounded = (value * 100.).round() / 100.;
    if rounded.abs() < 5e-3 { 0. } else { rounded }
}

fn wrap(angle: f64) -> f64 {
    (angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

/// Relative bearing in degrees and a word for it.
fn bearing(pose: Pose, x: f64, y: f64) -> (f64, &'static str) {
    let rel = wrap((y - pose.y).atan2(x - pose.x) - pose.yaw).to_degrees();
    let word = match rel {
        r if r.abs() <= 20. => "ahead",
        r if (20. ..70.).contains(&r) => "ahead-left",
        r if (70. ..=110.).contains(&r) => "left",
        r if (110. ..160.).contains(&r) => "behind-left",
        r if (-70. ..-20.).contains(&r) => "ahead-right",
        r if (-110. ..-70.).contains(&r) => "right",
        r if (-160. ..-110.).contains(&r) => "behind-right",
        _ => "behind",
    };
    (rel, word)
}

/// 1 m symbolic grid with y labels on the left and x labels underneath.
/// `#` any occupied child, `.` all children observed and free, `?` otherwise, `R` rover.
pub(crate) fn symbolic_grid(map: &MapSnapshot, pose: Pose) -> String {
    let child = (1.0 / map.resolution).round().max(1.) as usize;
    let cols = (map.width as usize).div_ceil(child);
    let rows = (map.height as usize).div_ceil(child);
    let width = map.width as usize;
    let rover =
        terra_navigation::grid_cell(map, pose.x, pose.y).map(|(c, r)| (c / child, r / child));
    let mut lines = Vec::with_capacity(rows + 2);
    for br in (0..rows).rev() {
        let y0 = map.origin_y + (br * child) as f64 * map.resolution;
        let mut line = format!("{:>6.1} |", y0);
        for bc in 0..cols {
            let (mut occupied, mut free, mut unknown) = (0, 0, 0);
            for r in br * child..((br + 1) * child).min(map.height as usize) {
                for c in bc * child..((bc + 1) * child).min(width) {
                    match map.occupancy[r * width + c] {
                        v if v < 0 => unknown += 1,
                        v if v >= 65 => occupied += 1,
                        _ => free += 1,
                    }
                }
            }
            line.push(if rover == Some((bc, br)) {
                'R'
            } else if occupied > 0 {
                '#'
            } else if free > 0 && unknown == 0 {
                '.'
            } else {
                '?'
            });
        }
        lines.push(line);
    }
    let x0 = map.origin_x;
    let x1 = map.origin_x + (cols * child) as f64 * map.resolution;
    lines.push(format!("       +{}", "-".repeat(cols)));
    lines.push(format!(
        "        x from {x0:.1} (left) to {x1:.1} (right), 1 m per character; rows are the y of each row's lower edge"
    ));
    lines.join("\n")
}

/// Occupied 8-connected blobs as (cell centres, centroid, bounds).
fn obstacle_blobs(map: &MapSnapshot) -> Vec<Vec<(f64, f64)>> {
    let width = map.width as usize;
    let height = map.height as usize;
    let mut seen = vec![false; width * height];
    let mut blobs = Vec::new();
    for start in 0..width * height {
        if seen[start] || map.occupancy[start] < 65 {
            continue;
        }
        let mut cells = Vec::new();
        let mut stack = vec![start];
        seen[start] = true;
        while let Some(index) = stack.pop() {
            let (c, r) = ((index % width) as isize, (index / width) as isize);
            cells.push((
                map.origin_x + (c as f64 + 0.5) * map.resolution,
                map.origin_y + (r as f64 + 0.5) * map.resolution,
            ));
            for dy in -1..=1 {
                for dx in -1..=1 {
                    let (nc, nr) = (c + dx, r + dy);
                    if nc < 0 || nr < 0 || nc >= width as isize || nr >= height as isize {
                        continue;
                    }
                    let n = nr as usize * width + nc as usize;
                    if !seen[n] && map.occupancy[n] >= 65 {
                        seen[n] = true;
                        stack.push(n);
                    }
                }
            }
        }
        blobs.push(cells);
    }
    blobs
}

/// Reachable, fully observed cells next to unknown space, clustered at least 1.5 m apart.
fn frontier_points(
    map: &MapSnapshot,
    pose: Pose,
    radius: f64,
    reach: &Reach,
) -> Vec<([f64; 2], u32)> {
    let width = map.width as usize;
    let height = map.height as usize;
    let span = (radius / map.resolution).ceil() as isize + 2;
    let mut candidates = Vec::new();
    for index in 0..width * height {
        if !reach.goal_cell(index) {
            continue;
        }
        let (c, r) = ((index % width) as isize, (index / width) as isize);
        let x = map.origin_x + (c as f64 + 0.5) * map.resolution;
        let y = map.origin_y + (r as f64 + 0.5) * map.resolution;
        if (x - pose.x).hypot(y - pose.y) < radius.max(0.8) {
            continue;
        }
        let mut unknown = 0u32;
        for dy in -span..=span {
            for dx in -span..=span {
                let (nc, nr) = (c + dx, r + dy);
                if nc >= 0
                    && nr >= 0
                    && nc < width as isize
                    && nr < height as isize
                    && map.occupancy[nr as usize * width + nc as usize] < 0
                {
                    unknown += 1;
                }
            }
        }
        if unknown > 0 {
            let score = f64::from(unknown) / (1. + f64::from(reach.distance[index]) * 0.05);
            candidates.push((score, [tidy(x), tidy(y)], unknown));
        }
    }
    candidates.sort_by(|a, b| {
        b.0.total_cmp(&a.0)
            .then(a.1[0].total_cmp(&b.1[0]))
            .then(a.1[1].total_cmp(&b.1[1]))
    });
    let mut picked: Vec<([f64; 2], u32)> = Vec::new();
    for (_, point, unknown) in candidates {
        if picked
            .iter()
            .all(|(p, _)| (p[0] - point[0]).hypot(p[1] - point[1]) >= 1.5)
        {
            picked.push((point, unknown));
            if picked.len() == MAX_FRONTIERS {
                break;
            }
        }
    }
    picked
}

/// Build the map section of the prompt and the id table used to resolve the reply.
pub(crate) fn summarize(
    map: &MapSnapshot,
    pose: Pose,
    radius: f64,
    landmarks: &[TaskLandmark],
    destination: Option<(f64, f64)>,
    now: f64,
) -> MapSummary {
    let mut text = String::new();
    let mut targets = Vec::new();
    let reach = Reach::build(map, pose, radius);
    let (mut free, mut occupied, mut unknown) = (0, 0, 0);
    for v in &map.occupancy {
        match *v {
            v if v < 0 => unknown += 1,
            v if v >= 65 => occupied += 1,
            _ => free += 1,
        }
    }
    let cell_area = map.resolution * map.resolution;
    text.push_str(&format!(
        "MAP: {:.0} x {:.0} m window around the rover. Seen free {:.0} m2, obstacle {:.0} m2, unknown {:.0} m2.\n\
Legend: '#' obstacle, '.' free, '?' unknown or partly seen, 'R' rover. Up is +y, right is +x.\n",
        f64::from(map.width) * map.resolution,
        f64::from(map.height) * map.resolution,
        f64::from(free) * cell_area,
        f64::from(occupied) * cell_area,
        f64::from(unknown) * cell_area,
    ));
    text.push_str(&symbolic_grid(map, pose));
    text.push('\n');

    let describe = |x: f64, y: f64, travel: Option<f64>| {
        let (rel, word) = bearing(pose, x, y);
        let straight = (x - pose.x).hypot(y - pose.y);
        match travel {
            Some(t) => format!(
                "({x:.1}, {y:.1}), {straight:.1} m {word} (bearing {rel:+.0} deg), drive about {t:.1} m"
            ),
            None => format!("({x:.1}, {y:.1}), {straight:.1} m {word} (bearing {rel:+.0} deg)"),
        }
    };

    text.push_str(
        "\nFRONTIERS (seen free space at the edge of unknown space; go here to see more):\n",
    );
    let frontiers = reach
        .as_ref()
        .map(|reach| frontier_points(map, pose, radius, reach))
        .unwrap_or_default();
    if frontiers.is_empty() {
        text.push_str("  none reachable\n");
    }
    for (i, (point, unknown)) in frontiers.iter().enumerate() {
        let id = format!("F{}", i + 1);
        let travel = reach
            .as_ref()
            .and_then(|r| r.travel(map, point[0], point[1]));
        text.push_str(&format!(
            "  {id} {}, {} unknown cells nearby\n",
            describe(point[0], point[1], travel),
            unknown
        ));
        targets.push(TaskTarget {
            id,
            approach: Some(*point),
        });
    }

    text.push_str("\nOBSTACLES (occupied blobs; a target id drives to free space beside it):\n");
    let mut blobs: Vec<_> = obstacle_blobs(map)
        .into_iter()
        .filter(|cells| cells.len() >= 2)
        .map(|cells| {
            let near = cells
                .iter()
                .map(|&(x, y)| (x - pose.x).hypot(y - pose.y))
                .fold(f64::MAX, f64::min);
            (near, cells)
        })
        .collect();
    blobs.sort_by(|a, b| a.0.total_cmp(&b.0));
    if blobs.is_empty() {
        text.push_str("  none seen\n");
    }
    for (i, (near, cells)) in blobs.iter().take(MAX_OBSTACLES).enumerate() {
        let id = format!("O{}", i + 1);
        let n = cells.len() as f64;
        let cx = cells.iter().map(|p| p.0).sum::<f64>() / n;
        let cy = cells.iter().map(|p| p.1).sum::<f64>() / n;
        let (min_x, max_x, min_y, max_y) = cells.iter().fold(
            (f64::MAX, f64::MIN, f64::MAX, f64::MIN),
            |(a, b, c, d), &(x, y)| (a.min(x), b.max(x), c.min(y), d.max(y)),
        );
        let size_x = max_x - min_x + map.resolution;
        let size_y = max_y - min_y + map.resolution;
        let approach = reach
            .as_ref()
            .and_then(|r| r.nearest_goal_to(map, cells, radius + 1.0));
        let (rel, word) = bearing(pose, cx, cy);
        text.push_str(&format!(
            "  {id} centre ({cx:.1}, {cy:.1}) {word} (bearing {rel:+.0} deg), size {size_x:.1} x {size_y:.1} m, nearest edge {near:.1} m{}\n",
            if approach.is_some() { "" } else { ", no reachable free space beside it" }
        ));
        targets.push(TaskTarget { id, approach });
    }
    if blobs.len() > MAX_OBSTACLES {
        text.push_str(&format!(
            "  ...and {} smaller or farther blobs\n",
            blobs.len() - MAX_OBSTACLES
        ));
    }

    let mut recent: Vec<_> = landmarks
        .iter()
        .filter(|l| now - l.seen_at >= 0. && now - l.seen_at <= TASK_LANDMARK_MAX_AGE_SECONDS)
        .collect();
    recent.sort_by(|a, b| b.seen_at.total_cmp(&a.seen_at));
    text.push_str("\nLANDMARKS (things the phone recognised):\n");
    if recent.is_empty() {
        text.push_str("  none\n");
    }
    for (i, l) in recent.iter().take(MAX_LANDMARKS).enumerate() {
        let id = format!("L{}", i + 1);
        let approach = reach
            .as_ref()
            .and_then(|r| r.nearest_goal(map, l.x, l.y, radius + 1.0));
        text.push_str(&format!(
            "  {id} \"{}\" at {}, confidence {:.2}, seen {:.0} s ago{}\n",
            l.label,
            describe(l.x, l.y, None),
            l.confidence,
            now - l.seen_at,
            if approach.is_some() {
                ""
            } else {
                ", no reachable free space beside it"
            }
        ));
        targets.push(TaskTarget { id, approach });
    }

    // The operator's GPS goal, re-anchored into the local frame on every step.
    if let Some((x, y)) = destination.filter(|(x, y)| x.is_finite() && y.is_finite()) {
        // The reachable seen cell closest to the goal: the way to make progress toward it.
        let approach = reach
            .as_ref()
            .and_then(|r| r.nearest_goal(map, x, y, f64::from(u16::MAX)));
        text.push_str(
            "\nGPS DESTINATION (the operator's final goal from GPS; it may lie beyond the map):\n",
        );
        text.push_str(&format!("  G1 at {}", describe(x, y, None)));
        match approach {
            Some([ax, ay]) if (ax - x).hypot(ay - y) <= radius + 1. => {
                text.push_str(", reachable\n")
            }
            Some([ax, ay]) => text.push_str(&format!(
                ", not reachable yet; G1 drives to the closest seen point, {:.1} m short of it\n",
                (ax - x).hypot(ay - y)
            )),
            None => text.push_str(", no reachable free space toward it\n"),
        }
        targets.push(TaskTarget {
            id: "G1".into(),
            approach,
        });
    }
    MapSummary { text, targets }
}

/// The parts of a reply the arbiter acts on.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum ParsedDecision {
    Go {
        waypoints: Vec<RequestedWaypoint>,
        rationale: Option<String>,
    },
    AskOperator {
        question: String,
        rationale: Option<String>,
    },
    Done {
        rationale: Option<String>,
    },
    Fail {
        rationale: Option<String>,
    },
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) enum RequestedWaypoint {
    Target(String),
    Point(f64, f64),
}

/// Keys that would make a reply a motion command. A reply naming any is refused.
const MOTOR_KEYS: &[&str] = &[
    "linear",
    "angular",
    "cmd_vel",
    "twist",
    "velocity",
    "speed",
    "effort",
    "left_effort",
    "right_effort",
    "pwm",
    "motor",
    "motors",
    "servo",
    "throttle",
    "steering",
    "wheel",
];

fn has_motor_key(value: &serde_json::Value) -> bool {
    match value {
        serde_json::Value::Object(map) => map.iter().any(|(k, v)| {
            MOTOR_KEYS.contains(&k.to_ascii_lowercase().as_str()) || has_motor_key(v)
        }),
        serde_json::Value::Array(items) => items.iter().any(has_motor_key),
        _ => false,
    }
}

/// First balanced `{...}` in `text`, skipping braces inside strings.
pub(crate) fn extract_json_object(text: &str) -> Option<&str> {
    let start = text.find('{')?;
    let bytes = text.as_bytes();
    let (mut depth, mut in_string, mut escaped) = (0usize, false, false);
    for (i, &b) in bytes.iter().enumerate().skip(start) {
        if in_string {
            match b {
                _ if escaped => escaped = false,
                b'\\' => escaped = true,
                b'"' => in_string = false,
                _ => {}
            }
            continue;
        }
        match b {
            b'"' => in_string = true,
            b'{' => depth += 1,
            b'}' => {
                depth -= 1;
                if depth == 0 {
                    return Some(&text[start..=i]);
                }
            }
            _ => {}
        }
    }
    None
}

fn short(text: Option<&serde_json::Value>, limit: usize) -> Option<String> {
    let text = text?.as_str()?.trim();
    if text.is_empty() {
        return None;
    }
    let clean: String = text
        .chars()
        .map(|c| if c.is_control() { ' ' } else { c })
        .take(limit)
        .collect();
    Some(clean)
}

/// Parse the model's reply. Errors are short strings fed back to the model.
pub(crate) fn parse_decision(text: &str) -> Result<ParsedDecision, String> {
    if text.len() > TASK_RESPONSE_MAX_BYTES {
        return Err("reply_too_long".into());
    }
    let object = extract_json_object(text).ok_or("no JSON object in the reply")?;
    let value: serde_json::Value =
        serde_json::from_str(object).map_err(|e| format!("invalid JSON: {e}"))?;
    if !value.is_object() {
        return Err("the reply must be a JSON object".into());
    }
    if has_motor_key(&value) {
        return Err("motor_fields_forbidden: reply with waypoints, never speeds or efforts".into());
    }
    let rationale = short(value.get("rationale"), 300);
    let decision = value
        .get("decision")
        .and_then(|d| d.as_str())
        .ok_or("missing \"decision\"")?
        .trim()
        .to_ascii_lowercase();
    match decision.as_str() {
        "go" | "waypoints" | "waypoint" | "move" => {
            let list = value
                .get("waypoints")
                .and_then(|w| w.as_array())
                .ok_or("\"go\" needs a \"waypoints\" list")?;
            if list.is_empty() {
                return Err("\"waypoints\" is empty".into());
            }
            let mut waypoints = Vec::new();
            for item in list.iter().take(TASK_MAX_WAYPOINTS) {
                if let Some(target) = item
                    .get("target")
                    .or_else(|| item.get("id"))
                    .and_then(|t| t.as_str())
                {
                    waypoints.push(RequestedWaypoint::Target(
                        target.trim().to_ascii_uppercase(),
                    ));
                } else if let Some(target) = item.as_str() {
                    waypoints.push(RequestedWaypoint::Target(
                        target.trim().to_ascii_uppercase(),
                    ));
                } else if let (Some(x), Some(y)) = (
                    item.get("x").and_then(|v| v.as_f64()),
                    item.get("y").and_then(|v| v.as_f64()),
                ) {
                    if !x.is_finite() || !y.is_finite() {
                        return Err("waypoint coordinates must be finite".into());
                    }
                    waypoints.push(RequestedWaypoint::Point(x, y));
                } else {
                    return Err("each waypoint needs \"target\" or \"x\" and \"y\"".into());
                }
            }
            Ok(ParsedDecision::Go {
                waypoints,
                rationale,
            })
        }
        "ask_operator" | "ask" => Ok(ParsedDecision::AskOperator {
            question: short(value.get("question"), 300)
                .or_else(|| rationale.clone())
                .ok_or("\"ask_operator\" needs a \"question\"")?,
            rationale,
        }),
        "done" | "complete" | "completed" => Ok(ParsedDecision::Done { rationale }),
        "fail" | "failed" | "give_up" => Ok(ParsedDecision::Fail { rationale }),
        other => Err(format!(
            "unknown decision \"{}\"; use go, ask_operator, done or fail",
            other.chars().take(32).collect::<String>()
        )),
    }
}

/// Resolve one requested waypoint from `from` to an approach point A* can route to.
pub(crate) fn resolve_waypoint(
    map: &MapSnapshot,
    from: Pose,
    radius: f64,
    targets: &[TaskTarget],
    requested: &RequestedWaypoint,
) -> Result<TaskWaypoint, String> {
    let (point, label) = match requested {
        RequestedWaypoint::Target(id) => {
            let target = targets
                .iter()
                .find(|t| &t.id == id)
                .ok_or_else(|| format!("{id} is not in the lists"))?;
            (
                target
                    .approach
                    .ok_or_else(|| format!("{id} has no reachable free space beside it"))?,
                id.clone(),
            )
        }
        RequestedWaypoint::Point(x, y) => {
            let reach = Reach::build(map, from, radius).ok_or("rover is outside the map")?;
            let point = reach
                .nearest_goal(map, *x, *y, TASK_SNAP_RADIUS)
                .ok_or_else(|| {
                    format!(
                        "({x:.1}, {y:.1}) is not reachable seen free space for the rover footprint"
                    )
                })?;
            (point, "point".into())
        }
    };
    astar_route(map, from, radius, point[0], point[1])
        .map_err(|why| format!("{label} at ({:.1}, {:.1}): {why}", point[0], point[1]))?;
    Ok(TaskWaypoint {
        x: point[0],
        y: point[1],
        label,
    })
}

/// Validate a plan from the rover's pose. The first waypoint must resolve; a later
/// failure truncates the plan there and is reported back.
pub(crate) fn resolve_plan(
    map: &MapSnapshot,
    pose: Pose,
    radius: f64,
    targets: &[TaskTarget],
    requested: &[RequestedWaypoint],
) -> Result<(Vec<TaskWaypoint>, Option<String>), String> {
    let mut plan = Vec::new();
    let mut from = pose;
    for (i, item) in requested.iter().enumerate() {
        match resolve_waypoint(map, from, radius, targets, item) {
            Ok(w) => {
                from = Pose {
                    x: w.x,
                    y: w.y,
                    yaw: pose.yaw,
                };
                plan.push(w);
            }
            Err(why) if i == 0 => return Err(why),
            Err(why) => return Ok((plan, Some(format!("waypoint {} dropped: {why}", i + 1)))),
        }
    }
    Ok((plan, None))
}

/// Render the step prompt.
#[allow(clippy::too_many_arguments)]
pub(crate) fn step_prompt(
    instruction: &str,
    step: u32,
    max_steps: u32,
    elapsed: f64,
    budget: f64,
    pose: Pose,
    radius: f64,
    map_text: &str,
    history: &VecDeque<TaskHistoryEntry>,
) -> String {
    let mut text = format!(
        "TASK from the operator: {}\n\nStep {step} of at most {max_steps}. Time used {elapsed:.0} s of {budget:.0} s.\n\
ROVER: at ({:.1}, {:.1}), heading {:.0} deg, footprint radius {radius:.2} m.\n\n",
        instruction.trim(),
        pose.x,
        pose.y,
        wrap(pose.yaw).to_degrees(),
    );
    text.push_str(map_text);
    text.push_str("\nHISTORY (oldest first):\n");
    if history.is_empty() {
        text.push_str("  nothing yet; this is the first step\n");
    }
    let skip = history.len().saturating_sub(PROMPT_HISTORY);
    for entry in history.iter().skip(skip) {
        text.push_str(&format!(
            "  [{}] {}: {}\n",
            entry.step, entry.kind, entry.text
        ));
    }
    text.push_str("\nReply with one JSON object only.");
    text
}

pub(crate) fn push_history(
    history: &mut VecDeque<TaskHistoryEntry>,
    step: u32,
    kind: &str,
    text: String,
) {
    history.push_back(TaskHistoryEntry {
        step,
        kind: kind.into(),
        text: text.chars().take(300).collect(),
    });
    while history.len() > HISTORY_LIMIT {
        history.pop_front();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn room() -> MapSnapshot {
        // 10 x 10 m, all free except a wall at x 6.0..6.3 for y 2..8 and unknown beyond x 8.
        let mut map = MapSnapshot {
            width: 100,
            height: 100,
            resolution: 0.1,
            origin_x: 0.,
            origin_y: 0.,
            occupancy: vec![0; 10_000],
        };
        for r in 0..100 {
            for c in 0..100 {
                let x = c as f64 * 0.1;
                let y = r as f64 * 0.1;
                let v = if (6.0..6.3).contains(&x) && (2.0..8.0).contains(&y) {
                    100
                } else if x >= 8.0 || c == 0 || r == 0 || c == 99 || r == 99 {
                    -1
                } else {
                    0
                };
                map.occupancy[r * 100 + c] = v;
            }
        }
        map
    }

    #[test]
    fn decodes_task_requests_and_actions_strictly() {
        let r = decode_task_request(
            br#"{"task_id":"t1","instruction":"find the bucket","token":"k1","budget_seconds":120}"#,
        )
        .unwrap();
        assert_eq!(r.spec.budget(), 120.);
        assert_eq!(r.spec.steps(), TASK_DEFAULT_MAX_STEPS);
        assert!(
            decode_task_request(br#"{"task_id":"t1","instruction":"  ","token":"k1"}"#).is_none()
        );
        assert!(
            decode_task_request(br#"{"task_id":"t1","instruction":"x","token":"k1","extra":1}"#)
                .is_none()
        );
        assert!(
            decode_task_request(
                br#"{"task_id":"t1","instruction":"x","token":"k1","max_steps":0}"#
            )
            .is_none()
        );
        assert!(decode_task_action(br#"{"task_id":"t1","action":"reply","token":"a"}"#).is_none());
        assert!(
            decode_task_action(
                br#"{"task_id":"t1","action":"reply","text":"left one","token":"a"}"#
            )
            .is_some()
        );
        assert!(decode_task_action(br#"{"task_id":"t1","action":"cancel","token":"a"}"#).is_some());
    }

    #[test]
    fn parses_fenced_replies_and_refuses_motor_fields() {
        let reply = "Sure.\n```json\n{\"decision\":\"go\",\"waypoints\":[{\"target\":\"f1\"},{\"x\":2,\"y\":3}],\"rationale\":\"look {behind}\"}\n```";
        assert_eq!(
            parse_decision(reply).unwrap(),
            ParsedDecision::Go {
                waypoints: vec![
                    RequestedWaypoint::Target("F1".into()),
                    RequestedWaypoint::Point(2., 3.)
                ],
                rationale: Some("look {behind}".into()),
            }
        );
        let err = parse_decision(r#"{"decision":"go","waypoints":[{"x":1,"y":1}],"linear":0.5}"#)
            .unwrap_err();
        assert!(err.starts_with("motor_fields_forbidden"));
        assert!(parse_decision(r#"{"decision":"ask_operator"}"#).is_err());
        assert!(matches!(
            parse_decision(r#"{"decision":"done","rationale":"at the wall"}"#),
            Ok(ParsedDecision::Done { .. })
        ));
        assert!(parse_decision("no json here").is_err());
        assert!(parse_decision(r#"{"decision":"spin"}"#).is_err());
    }

    #[test]
    fn summary_lists_frontiers_and_obstacles_with_reachable_approaches() {
        let map = room();
        let pose = Pose {
            x: 2.,
            y: 5.,
            yaw: 0.,
        };
        let summary = summarize(&map, pose, 0.4, &[], None, 0.);
        assert!(summary.text.contains("F1"), "{}", summary.text);
        assert!(summary.text.contains("O1"), "{}", summary.text);
        assert!(summary.text.contains('R'));
        for target in &summary.targets {
            if let Some([x, y]) = target.approach {
                assert!(astar_route(&map, pose, 0.4, x, y).is_ok(), "{target:?}");
            }
        }
        // The wall's approach point is on free space beside it.
        let wall = summary.targets.iter().find(|t| t.id == "O1").unwrap();
        let [x, _] = wall.approach.unwrap();
        assert!(x < 6.0 || x > 6.3);
    }

    #[test]
    fn resolves_points_by_snapping_and_rejects_obstacles_and_unknown() {
        let map = room();
        let pose = Pose {
            x: 2.,
            y: 5.,
            yaw: 0.,
        };
        let w =
            resolve_waypoint(&map, pose, 0.4, &[], &RequestedWaypoint::Point(3.0, 5.0)).unwrap();
        assert!((w.x - 3.0).abs() <= TASK_SNAP_RADIUS && (w.y - 5.0).abs() <= TASK_SNAP_RADIUS);
        assert!(
            resolve_waypoint(&map, pose, 0.4, &[], &RequestedWaypoint::Point(6.1, 5.0)).is_err()
        );
        assert!(
            resolve_waypoint(&map, pose, 0.4, &[], &RequestedWaypoint::Point(9.0, 5.0)).is_err()
        );
        assert!(
            resolve_waypoint(
                &map,
                pose,
                0.4,
                &[],
                &RequestedWaypoint::Target("F9".into())
            )
            .is_err()
        );
        let (plan, note) = resolve_plan(
            &map,
            pose,
            0.4,
            &[],
            &[
                RequestedWaypoint::Point(3., 3.),
                RequestedWaypoint::Point(9.5, 9.5),
            ],
        )
        .unwrap();
        assert_eq!(plan.len(), 1);
        assert!(note.unwrap().contains("waypoint 2 dropped"));
    }

    #[test]
    fn prompt_carries_task_pose_and_recent_history() {
        let map = room();
        let pose = Pose {
            x: 2.,
            y: 5.,
            yaw: std::f64::consts::FRAC_PI_2,
        };
        let summary = summarize(&map, pose, 0.4, &[], None, 0.);
        let mut history = VecDeque::new();
        for i in 0..10 {
            push_history(&mut history, i, "arrived", format!("entry {i}"));
        }
        let text = step_prompt(
            "go to the wall",
            3,
            20,
            12.,
            300.,
            pose,
            0.4,
            &summary.text,
            &history,
        );
        assert!(text.contains("TASK from the operator: go to the wall"));
        assert!(text.contains("heading 90 deg"));
        assert!(text.contains("entry 9") && !text.contains("entry 3"));
    }
}
