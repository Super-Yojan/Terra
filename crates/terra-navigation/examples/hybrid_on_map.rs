//! Run Hybrid A* on a saved occupancy map (from scripts/dashboard/save-occupancy-map.py).
//!
//! cargo run -p terra-navigation --release --example hybrid_on_map -- maps/run1.json [out.json] [goal_x goal_y]
//!
//! Picks a start where the rover fits nearest the map centre (the phone map is
//! centred on the rover) and, as goals, the farthest reachable spot where the
//! rover fits in each quadrant around the start. Prints each result and writes
//! the routes as JSON for drawing. With `goal_x goal_y` it plans only to that point.
//! Set HYBRID_BODY=0.178 to plan for the terra_mini body instead of terra (0.85 m).
use std::time::Instant;
use terra_mapping::MapSnapshot;
use terra_navigation::{HybridConfig, HybridGoal, Pose, hybrid_astar_route};

fn main() {
    let mut args = std::env::args().skip(1);
    let path = args
        .next()
        .expect("usage: hybrid_on_map <map.json> [routes.json] [goal_x goal_y]");
    let out = args.next();
    let explicit: Vec<f64> = args.map(|a| a.parse().expect("goal coordinates")).collect();
    let saved: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(&path).expect("read map")).expect("map JSON");
    let map = MapSnapshot {
        width: saved["width"].as_u64().unwrap() as u32,
        height: saved["height"].as_u64().unwrap() as u32,
        resolution: saved["resolution"].as_f64().unwrap(),
        origin_x: saved["origin_x"].as_f64().unwrap(),
        origin_y: saved["origin_y"].as_f64().unwrap(),
        occupancy: saved["occupancy"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_i64().unwrap() as i8)
            .collect(),
    };
    // HYBRID_BODY=0.178 plans for a square body of that size (terra_mini); default is terra (0.85 m).
    let mut config = HybridConfig::default();
    if let Some(size) = std::env::var("HYBRID_BODY")
        .ok()
        .and_then(|v| v.parse().ok())
    {
        config.footprint.length = size;
        config.footprint.width = size;
    }
    let centre = |c: usize, r: usize| {
        (
            map.origin_x + (c as f64 + 0.5) * map.resolution,
            map.origin_y + (r as f64 + 0.5) * map.resolution,
        )
    };
    let fits: Vec<(f64, f64)> = (0..map.height as usize)
        .flat_map(|r| (0..map.width as usize).map(move |c| (c, r)))
        .filter(|&(c, r)| r % 2 == 0 && c % 2 == 0)
        .map(|(c, r)| centre(c, r))
        .filter(|&(x, y)| body_fits(&map, x, y, 0., &config))
        .collect();
    let mid = centre(map.width as usize / 2, map.height as usize / 2);
    let &(sx, sy) = fits
        .iter()
        .min_by(|a, b| dist(**a, mid).total_cmp(&dist(**b, mid)))
        .expect("no spot where the rover fits");
    let start = Pose {
        x: sx,
        y: sy,
        yaw: 0.,
    };
    println!(
        "{path}: {}x{} cells at {} m, {} spots where the body fits; start ({sx:.2}, {sy:.2}, yaw 0)",
        map.width,
        map.height,
        map.resolution,
        fits.len()
    );

    let mut routes = Vec::new();
    if let [gx, gy] = explicit[..] {
        let goal = HybridGoal {
            x: gx,
            y: gy,
            yaw: None,
        };
        let began = Instant::now();
        let result = hybrid_astar_route(&map, start, goal, &config);
        let ms = began.elapsed().as_secs_f64() * 1000.;
        let fits_any_heading =
            (0..36).any(|k| body_fits(&map, gx, gy, k as f64 * 5f64.to_radians(), &config));
        match result {
            Ok(route) => {
                let length: f64 = route
                    .windows(2)
                    .map(|w| (w[1].x - w[0].x).hypot(w[1].y - w[0].y))
                    .sum();
                let reverse = route.iter().filter(|p| p.reverse).count();
                println!(
                    "  goal ({gx:.2}, {gy:.2}): route {length:.1} m, {} samples, {reverse} reversing, {ms:.0} ms",
                    route.len()
                );
                routes.push(serde_json::json!({"name": "goal", "goal": [gx, gy], "route": route}));
            }
            Err(error) => println!(
                "  goal ({gx:.2}, {gy:.2}): {error} ({ms:.0} ms); body fits there at some heading: {fits_any_heading}"
            ),
        }
    }
    let quadrants: &[(f64, f64, &str)] = if explicit.is_empty() {
        &[
            (1., 1., "NE"),
            (-1., 1., "NW"),
            (-1., -1., "SW"),
            (1., -1., "SE"),
        ]
    } else {
        &[]
    };
    for &(qx, qy, name) in quadrants {
        // Farthest fitting spots in this quadrant, tried in order until one is reachable.
        let mut candidates: Vec<_> = fits
            .iter()
            .filter(|&&(x, y)| (x - sx) * qx > 0.5 && (y - sy) * qy > 0.5)
            .copied()
            .collect();
        candidates.sort_by(|a, b| dist(*b, (sx, sy)).total_cmp(&dist(*a, (sx, sy))));
        for &(gx, gy) in candidates.iter().step_by(25).take(6) {
            let goal = HybridGoal {
                x: gx,
                y: gy,
                yaw: None,
            };
            let began = Instant::now();
            let result = hybrid_astar_route(&map, start, goal, &config);
            let ms = began.elapsed().as_secs_f64() * 1000.;
            match result {
                Ok(route) => {
                    let length: f64 = route
                        .windows(2)
                        .map(|w| (w[1].x - w[0].x).hypot(w[1].y - w[0].y))
                        .sum();
                    let reverse = route.iter().filter(|p| p.reverse).count();
                    println!(
                        "  {name}: goal ({gx:.2}, {gy:.2}) {:.1} m away -> route {length:.1} m, {} samples, {reverse} reversing, {ms:.0} ms",
                        dist((gx, gy), (sx, sy)),
                        route.len()
                    );
                    routes
                        .push(serde_json::json!({"name": name, "goal": [gx, gy], "route": route}));
                    break;
                }
                Err(error) => println!(
                    "  {name}: goal ({gx:.2}, {gy:.2}) -> {error} ({ms:.0} ms), trying a nearer goal"
                ),
            }
        }
    }
    if let Some(out) = out {
        let json = serde_json::json!({"start": [sx, sy, 0.], "routes": routes, "config": config});
        std::fs::write(&out, json.to_string()).expect("write routes");
        println!("wrote {out}");
    }
}

fn dist(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - b.0).hypot(a.1 - b.1)
}

/// Dense points over the body rectangle all land on observed free cells.
fn body_fits(map: &MapSnapshot, x: f64, y: f64, yaw: f64, config: &HybridConfig) -> bool {
    let (a, b) = (
        config.footprint.length / 2. + config.footprint.margin,
        config.footprint.width / 2. + config.footprint.margin,
    );
    let (sin, cos) = yaw.sin_cos();
    (0..=20).all(|i| {
        (0..=20).all(|j| {
            let u = -a + 2. * a * i as f64 / 20.;
            let v = -b + 2. * b * j as f64 / 20.;
            let (px, py) = (x + u * cos - v * sin, y + u * sin + v * cos);
            let column = ((px - map.origin_x) / map.resolution).floor();
            let row = ((py - map.origin_y) / map.resolution).floor();
            column >= 0.
                && row >= 0.
                && column < map.width as f64
                && row < map.height as f64
                && (0..65)
                    .contains(&map.occupancy[row as usize * map.width as usize + column as usize])
        })
    })
}
