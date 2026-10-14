//! Frozen prompt. Do not tune it against the eight labels and then quote those labels.

use terra_navigation::Pose;

use crate::scene::{
    CONFIDENCE_ASK_BELOW, DUMP_BUCKET_RANGE_M, FOOTPRINT_RADIUS, Scenario, level_label, sha256_hex,
};

pub fn rules_text() -> String {
    format!(
        "\
You are Terra's NEXT decision layer. You see two images and a structured object list.
Image 1 is an occupancy map. Each source cell is 0.1 m, painted as a 4 by 4 block, so the map is 800 by 800.
Free cells (occupancy 0-64) are light gray. Occupied cells (65 or more) are black. Unknown cells (-1) are dark gray.
The red arrow is the rover pose and heading. The blue wedge is the camera field of view.
The legend and the 1 m scale bar are drawn on the map. World +X is to the right. World +Y is up.
Image 2 is a labelled schematic of detections inside that wedge, not a photograph and not a Zorvane render.
The object list is computed in code. Ids such as B1 and P1 are shared with the map glyphs.
Return one JSON object and nothing else. No markdown. Fields and only these fields:
decision (waypoint, hold, or ask_operator),
target_id (an id from the object list, or null),
waypoint (null, or {{\"x\": number, \"y\": number}}),
dump_proposal (true or false),
rationale (a short string),
confidence (a number from 0 to 1).
Do not emit motor, wheel, twist, cmd_vel, effort, pwm, or servo fields. dump_proposal does not move a servo.
A waypoint is legal only when autonomy_level is waypoint (L3). At any other level, decision is hold or ask_operator, target_id is null, and dump_proposal is false.
When decision is waypoint, set target_id and leave the coordinate decision to the validator. The validator copies approach_x and approach_y from that object. It ignores numbers you invent. It accepts the goal only if A* finds a path for a {FOOTPRINT_RADIUS:.2} m footprint on observed free cells.
Never use a person id as target_id. A person is an obstacle.
If rover.carrying_balls is true and a bucket with confidence >= {CONFIDENCE_ASK_BELOW:.1} has its centroid within {DUMP_BUCKET_RANGE_M:.1} m, decision is hold and dump_proposal is true.
If every bucket has confidence below {CONFIDENCE_ASK_BELOW:.1}, decision is ask_operator and dump_proposal is false. Do not invent a bucket.
If a reachable bucket has confidence >= {CONFIDENCE_ASK_BELOW:.1} and you are not dumping, decision is waypoint and target_id is that bucket. Skip buckets whose reachable field is false.
A ball that is not carried does not justify dump_proposal. Do not target a ball.
If there is no bucket and no person, decision is hold, or waypoint with a frontier id whose reachable field is true.
confidence is your own estimate. It is not a calibrated probability."
    )
}

pub fn rules_sha256() -> String {
    sha256_hex(rules_text().as_bytes())
}

pub fn user_prompt(scenario: &Scenario) -> String {
    let pose = Pose {
        x: scenario.rover.x,
        y: scenario.rover.y,
        yaw: scenario.rover.yaw,
    };
    let heading_deg = pose.yaw.to_degrees();
    let objects = serde_json::to_string_pretty(&scenario.objects).unwrap_or_else(|_| "[]".into());
    let rover = serde_json::to_string(&scenario.rover).unwrap_or_else(|_| "{}".into());
    format!(
        "{}\n\nautonomy_level: {}\nrover: {}\nheading_deg: {:.1}\nmap_origin_xy: [{:.1}, {:.1}]\nmap_resolution_m: {:.1}\nfootprint_radius_m: {:.2}\ncell_counts: free={} occupied={} unknown={}\ncamera_fx: {:.1} camera_width_px: {} fov_range_m: {:.1}\ncamera_source: {}\n\nsymbolic_grid_1m (row 0 is +Y, column 0 is the west edge, R is the rover):\n{}\n\nobject_list:\n{}\n",
        rules_text(),
        level_label(scenario.level),
        rover,
        heading_deg,
        scenario.map.origin_x,
        scenario.map.origin_y,
        scenario.map.resolution,
        FOOTPRINT_RADIUS,
        scenario.counts.free,
        scenario.counts.occupied,
        scenario.counts.unknown,
        scenario.camera.fx,
        scenario.camera.width,
        scenario.camera.range_m,
        scenario.camera_source,
        scenario.symbolic_grid,
        objects
    )
}
