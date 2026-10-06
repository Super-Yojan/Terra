//! Deterministic local planning over observed occupancy, shared by phone and Bevy.
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use terra_mapping::MapSnapshot;
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Twist {
    pub linear: f64,
    pub angular: f64,
}
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}
impl Pose {
    pub fn finite(self) -> bool {
        [self.x, self.y, self.yaw].iter().all(|v| v.is_finite())
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PlannerConfig {
    pub radius: f64,
    pub max_linear: f64,
    pub max_angular: f64,
    pub acceleration: f64,
    pub yaw_acceleration: f64,
    pub braking: f64,
    pub horizon: f64,
}
impl Default for PlannerConfig {
    fn default() -> Self {
        Self {
            radius: 0.65,
            max_linear: 2.,
            max_angular: 2.,
            acceleration: 1.,
            yaw_acceleration: 2.,
            braking: 1.,
            horizon: 2.,
        }
    }
}
#[derive(Clone, Debug, Default)]
pub struct LocalPlanner {
    pub config: PlannerConfig,
}
#[derive(Clone, Copy, Debug)]
pub struct PlanningDecision {
    pub twist: Twist,
    pub reason: &'static str,
}
fn valid_map(m: &MapSnapshot) -> bool {
    m.width > 0
        && m.height > 0
        && m.width <= 1000
        && m.height <= 1000
        && m.occupancy.len() == m.width as usize * m.height as usize
        && m.resolution.is_finite()
        && m.resolution >= 0.01
        && m.origin_x.is_finite()
        && m.origin_y.is_finite()
}
fn cell(m: &MapSnapshot, x: f64, y: f64) -> Option<(usize, usize)> {
    if !valid_map(m) || !x.is_finite() || !y.is_finite() {
        return None;
    }
    let a = ((x - m.origin_x) / m.resolution).floor();
    let b = ((y - m.origin_y) / m.resolution).floor();
    (a >= 0. && b >= 0. && a < (m.width as f64) && b < (m.height as f64))
        .then_some((a as usize, b as usize))
}
fn clear(m: &MapSnapshot, x: f64, y: f64, r: f64, current: Option<Pose>) -> bool {
    let Some((cx, cy)) = cell(m, x, y) else {
        return false;
    };
    let n = (r / m.resolution).ceil() as isize + 1;
    for dy in -n..=n {
        for dx in -n..=n {
            let a = cx as isize + dx;
            let b = cy as isize + dy;
            let px = m.origin_x + (a as f64 + 0.5) * m.resolution;
            let py = m.origin_y + (b as f64 + 0.5) * m.resolution;
            if (px - x).hypot(py - y) > r + m.resolution * 0.7072 {
                continue;
            }
            if a < 0 || b < 0 || a >= m.width as isize || b >= m.height as isize {
                return false;
            }
            let v = m.occupancy[b as usize * m.width as usize + a as usize];
            if v >= 65 {
                return false;
            }
            if v < 0
                && current.is_none_or(|p| (px - p.x).hypot(py - p.y) > r + m.resolution * 0.7072)
            {
                return false;
            }
        }
    }
    true
}
impl LocalPlanner {
    fn valid(&self) -> bool {
        let c = &self.config;
        [
            c.radius,
            c.max_linear,
            c.max_angular,
            c.acceleration,
            c.yaw_acceleration,
            c.braking,
            c.horizon,
        ]
        .iter()
        .all(|v| v.is_finite() && *v > 0.)
            && c.radius <= 5.
            && c.horizon <= 10.
    }
    pub fn admissible(&self, m: &MapSnapshot, pose: Pose, t: Twist) -> bool {
        if !self.valid()
            || !valid_map(m)
            || !pose.finite()
            || !t.linear.is_finite()
            || !t.angular.is_finite()
        {
            return false;
        }
        let c = &self.config;
        let step = (m.resolution / (2. * t.linear.abs().max(0.1))).min(0.1);
        let mut p = pose;
        let mut elapsed = 0.;
        let duration = c.horizon + t.linear.abs() / c.braking;
        while elapsed < duration {
            if !clear(m, p.x, p.y, c.radius, Some(pose)) {
                return false;
            }
            let scale = if elapsed <= c.horizon {
                1.
            } else {
                (1. - (elapsed - c.horizon) * c.braking / t.linear.abs().max(0.001)).max(0.)
            };
            p.yaw += t.angular * scale * step;
            p.x += t.linear * scale * p.yaw.cos() * step;
            p.y += t.linear * scale * p.yaw.sin() * step;
            elapsed += step;
        }
        clear(m, p.x, p.y, c.radius, Some(pose))
    }
    pub fn plan(
        &self,
        m: &MapSnapshot,
        pose: Pose,
        current: Twist,
        intent: Twist,
        dt: f64,
    ) -> PlanningDecision {
        let stop = PlanningDecision {
            twist: Twist::default(),
            reason: "obstacle_blocked",
        };
        if !self.valid()
            || !pose.finite()
            || !dt.is_finite()
            || !(0.0..=0.5).contains(&dt)
            || [
                current.linear,
                current.angular,
                intent.linear,
                intent.angular,
            ]
            .iter()
            .any(|v| !v.is_finite())
        {
            return stop;
        }
        let c = &self.config;
        let lo = (current.linear - c.acceleration * dt).max(-c.max_linear);
        let hi = (current.linear + c.acceleration * dt).min(c.max_linear);
        let al = (current.angular - c.yaw_acceleration * dt).max(-c.max_angular);
        let ah = (current.angular + c.yaw_acceleration * dt).min(c.max_angular);
        let mut best: Option<(f64, Twist)> = None;
        for i in 0..=10 {
            for j in 0..=10 {
                let t = Twist {
                    linear: lo + (hi - lo) * i as f64 / 10.,
                    angular: al + (ah - al) * j as f64 / 10.,
                };
                if !self.admissible(m, pose, t) {
                    continue;
                }
                let score =
                    (t.linear - intent.linear).powi(2) + 0.3 * (t.angular - intent.angular).powi(2);
                if best.is_none_or(|b| score < b.0) {
                    best = Some((score, t));
                }
            }
        }
        best.map(|(_, twist)| PlanningDecision {
            twist,
            reason: if (twist.linear - intent.linear).abs() > 0.15
                || (twist.angular - intent.angular).abs() > 0.15
            {
                "assisted"
            } else {
                "active"
            },
        })
        .unwrap_or(stop)
    }
}
/// Breadth-first search through inflated observed free cells. Stable ordering breaks ties.
pub fn frontier(
    m: &MapSnapshot,
    pose: Pose,
    radius: f64,
    excluded: &[(f64, f64)],
) -> Option<(f64, f64)> {
    if !pose.finite() || !radius.is_finite() || radius <= 0. || radius > 5. {
        return None;
    }
    let start = cell(m, pose.x, pose.y)?;
    let mut seen = vec![false; m.occupancy.len()];
    let mut q = VecDeque::from([(start.0, start.1, 0usize)]);
    let mut best: Option<(f64, (f64, f64))> = None;
    while let Some((x, y, d)) = q.pop_front() {
        let index = y * m.width as usize + x;
        if seen[index] {
            continue;
        }
        seen[index] = true;
        let px = m.origin_x + (x as f64 + 0.5) * m.resolution;
        let py = m.origin_y + (y as f64 + 0.5) * m.resolution;
        if !clear(m, px, py, radius, Some(pose)) {
            continue;
        }
        let reach = (radius / m.resolution).ceil() as isize + 2;
        let mut gain = 0;
        for dy in -reach..=reach {
            for dx in -reach..=reach {
                let a = x as isize + dx;
                let b = y as isize + dy;
                if a >= 0
                    && b >= 0
                    && a < m.width as isize
                    && b < m.height as isize
                    && m.occupancy[b as usize * m.width as usize + a as usize] < 0
                {
                    gain += 1;
                }
            }
        }
        if clear(m, px, py, radius, None)
            && gain > 0
            && (px - pose.x).hypot(py - pose.y) > radius.max(0.8)
            && excluded.iter().all(|&(a, b)| (px - a).hypot(py - b) > 1.)
        {
            let score = gain as f64 / (1. + d as f64);
            if best.is_none_or(|b| score > b.0) {
                best = Some((score, (px, py)));
            }
        }
        for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
            let a = x as isize + dx;
            let b = y as isize + dy;
            if a >= 0 && b >= 0 && a < m.width as isize && b < m.height as isize {
                q.push_back((a as usize, b as usize, d + 1));
            }
        }
    }
    best.map(|b| b.1)
}
/// Reachability check for proposal approval using observed, footprint-inflated free cells.
pub fn reachable(m: &MapSnapshot, pose: Pose, radius: f64, x: f64, y: f64) -> bool {
    let Some(start) = cell(m, pose.x, pose.y) else {
        return false;
    };
    let Some(target) = cell(m, x, y) else {
        return false;
    };
    if !pose.finite()
        || !radius.is_finite()
        || radius <= 0.
        || radius > 5.
        || !clear(m, x, y, radius, None)
    {
        return false;
    }
    let mut seen = vec![false; m.occupancy.len()];
    let mut q = VecDeque::from([start]);
    while let Some((a, b)) = q.pop_front() {
        let i = b * m.width as usize + a;
        if seen[i] {
            continue;
        }
        seen[i] = true;
        let px = m.origin_x + (a as f64 + 0.5) * m.resolution;
        let py = m.origin_y + (b as f64 + 0.5) * m.resolution;
        if !clear(m, px, py, radius, Some(pose)) {
            continue;
        }
        if (a, b) == target {
            return true;
        }
        for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
            let nx = a as isize + dx;
            let ny = b as isize + dy;
            if nx >= 0 && ny >= 0 && nx < m.width as isize && ny < m.height as isize {
                q.push_back((nx as usize, ny as usize));
            }
        }
    }
    false
}
/// Clearance to observed obstacle cell boundaries; unknown space is not evidence of clearance.
pub fn observed_clearance(m: &MapSnapshot, pose: Pose, radius: f64) -> Option<f64> {
    if !valid_map(m) || !pose.finite() || !radius.is_finite() || radius <= 0. {
        return None;
    }
    let mut best = f64::INFINITY;
    for (i, v) in m.occupancy.iter().enumerate() {
        if *v < 65 {
            continue;
        }
        let x = m.origin_x + (i % m.width as usize) as f64 * m.resolution;
        let y = m.origin_y + (i / m.width as usize) as f64 * m.resolution;
        let dx = (x - pose.x).max(0.).max(pose.x - x - m.resolution);
        let dy = (y - pose.y).max(0.).max(pose.y - y - m.resolution);
        best = best.min(dx.hypot(dy) - radius);
    }
    best.is_finite().then_some(best.max(0.))
}
