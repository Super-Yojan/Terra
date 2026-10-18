//! Eight synthetic NEXT scenarios. Geometry is real cells; labels are checked with A*.

use std::path::Path;
use std::sync::OnceLock;

use terra_autonomy::Level;
use terra_mapping::MapSnapshot;
use terra_navigation::{Pose, astar_route};

use crate::decision::DecisionKind;
use crate::geom::{self, blank, fill_disk, fill_rect};
use crate::objects::{self, Detection};
use crate::render::{render_camera_png, render_map_png};
use crate::scene::{
    CameraModel, ExpectedLabel, FOOTPRINT_RADIUS, RoverState, Scenario, SceneObject, sha256_hex,
};
use crate::{CAMERA_SOURCE_NOTE, CAMERA_SOURCE_PLACEHOLDER};

/// Return the eight synthetic scenarios, initialized once and shared thereafter.
///
/// # Panics
///
/// Panics if the built-in geometry, object extraction, or rendering cannot build the pack.
pub fn scenarios() -> &'static [Scenario] {
    static PACK: OnceLock<Vec<Scenario>> = OnceLock::new();
    PACK.get_or_init(|| build_pack().unwrap_or_else(|err| panic!("vlm eval pack: {err}")))
}

/// Generate each scenario's images, prompt, JSON packet, and a manifest under `root`.
/// Creates directories and overwrites existing fixture files.
///
/// # Errors
///
/// Returns filesystem, serialization, or occupancy round-trip failures as strings.
/// Files written before an error are retained.
pub fn write_fixtures(root: &Path) -> Result<(), String> {
    std::fs::create_dir_all(root).map_err(|err| err.to_string())?;
    let mut manifest = Vec::new();
    for scenario in scenarios() {
        let dir = root.join(&scenario.id);
        std::fs::create_dir_all(&dir).map_err(|err| err.to_string())?;
        std::fs::write(dir.join("map.png"), &scenario.map_png).map_err(|err| err.to_string())?;
        std::fs::write(dir.join("camera.png"), &scenario.camera_png)
            .map_err(|err| err.to_string())?;
        std::fs::write(dir.join("prompt.txt"), crate::prompt::user_prompt(scenario))
            .map_err(|err| err.to_string())?;
        let encoded = geom::encode_occupancy(&scenario.map.occupancy);
        let decoded = geom::decode_occupancy(&encoded)?;
        if decoded != scenario.map.occupancy {
            return Err(format!("{} occupancy did not round-trip", scenario.id));
        }
        let document = scenario_document(scenario);
        std::fs::write(
            dir.join("scenario.json"),
            serde_json::to_vec_pretty(&document).map_err(|err| err.to_string())?,
        )
        .map_err(|err| err.to_string())?;
        manifest.push(serde_json::json!({
            "id": scenario.id,
            "title": scenario.title,
            "autonomy_level": scenario.level,
            "camera_source": scenario.camera_source,
            "expected_decision": scenario.expected.decision,
            "dump_proposal": scenario.expected.dump_proposal,
        }));
    }
    std::fs::write(
        root.join("manifest.json"),
        serde_json::to_vec_pretty(&serde_json::json!({
            "version": 1,
            "camera_source": CAMERA_SOURCE_PLACEHOLDER,
            "camera_source_note": CAMERA_SOURCE_NOTE,
            "scenarios": manifest,
        }))
        .map_err(|err| err.to_string())?,
    )
    .map_err(|err| err.to_string())?;
    Ok(())
}

pub fn scenario_document(scenario: &Scenario) -> serde_json::Value {
    serde_json::json!({
        "version": 1,
        "id": scenario.id,
        "title": scenario.title,
        "notes": scenario.notes,
        "camera_source": scenario.camera_source,
        "camera_source_note": scenario.camera_source_note,
        "map_png_file": "map.png",
        "camera_png_file": "camera.png",
        "prompt_file": "prompt.txt",
        "map_png_sha256": scenario.map_png_sha256,
        "camera_png_sha256": scenario.camera_png_sha256,
        "map": {
            "width": scenario.map.width,
            "height": scenario.map.height,
            "resolution": scenario.map.resolution,
            "origin_x": scenario.map.origin_x,
            "origin_y": scenario.map.origin_y,
            "occupancy_base64": geom::encode_occupancy(&scenario.map.occupancy),
        },
        "objects": scenario.objects,
        "counts": scenario.counts,
        "symbolic_grid": scenario.symbolic_grid,
        "rover": scenario.rover,
        "autonomy_level": scenario.level,
        "camera": scenario.camera,
        "footprint_radius_m": FOOTPRINT_RADIUS,
        "expected": scenario.expected,
    })
}

struct Draft {
    id: &'static str,
    title: &'static str,
    notes: &'static str,
    level: Level,
    pose: Pose,
    carrying_balls: bool,
    timestamp_s: f64,
    detections: Vec<Detection>,
    map: MapSnapshot,
    decision: DecisionKind,
    accept: Vec<DecisionKind>,
    dump: bool,
    waypoint_required: bool,
    target: Target,
}

enum Target {
    None,
    At(&'static str, f64, f64),
    Frontiers,
}

fn build_pack() -> Result<Vec<Scenario>, String> {
    let mut pack = Vec::with_capacity(8);
    pack.push(scenario_bucket_reachable()?);
    pack.push(scenario_bucket_behind()?);
    pack.push(scenario_person()?);
    pack.push(scenario_frontier()?);
    pack.push(scenario_two_buckets()?);
    pack.push(scenario_dump()?);
    pack.push(scenario_low_confidence()?);
    pack.push(scenario_l0()?);
    if pack.len() != 8 {
        return Err(format!("expected 8 scenarios, built {}", pack.len()));
    }
    let mut ids: Vec<&str> = pack.iter().map(|scenario| scenario.id.as_str()).collect();
    ids.sort_unstable();
    ids.dedup();
    if ids.len() != 8 {
        return Err("scenario ids are not unique".into());
    }
    Ok(pack)
}

fn open_corridor(map: &mut MapSnapshot) {
    fill_rect(map, -3.0, -3.2, 7.5, 3.2, 0);
}

fn scenario_bucket_reachable() -> Result<Scenario, String> {
    let mut map = blank();
    open_corridor(&mut map);
    fill_disk(&mut map, 4.2, 0.0, 0.30, 100);
    fill_disk(&mut map, 2.4, 1.05, 0.15, 100);
    let pose = pose_origin();
    assemble(
        Draft {
            id: "01-bucket-reachable",
            title: "Bucket visible and reachable",
            notes: "Label: waypoint to the bucket approach. A loose ball is in the wedge; dump_proposal stays false.",
            level: Level::Waypoint,
            pose,
            carrying_balls: false,
            timestamp_s: 10.0,
            detections: vec![
                Detection {
                    class: "bucket",
                    x: 4.2,
                    y: 0.0,
                    confidence: 0.93,
                    age_s: 0.2,
                    carried: false,
                },
                Detection {
                    class: "ball",
                    x: 2.4,
                    y: 1.05,
                    confidence: 0.77,
                    age_s: 0.2,
                    carried: false,
                },
            ],
            map,
            decision: DecisionKind::Waypoint,
            accept: vec![DecisionKind::Waypoint],
            dump: false,
            waypoint_required: true,
            target: Target::At("bucket", 4.2, 0.0),
        },
        |map, pose, objects| {
            let bucket = find_class_at(objects, "bucket", 4.2, 0.0)?;
            if !bucket.reachable {
                return Err("01 bucket approach is not reachable".into());
            }
            let route = astar_route(
                map,
                *pose,
                FOOTPRINT_RADIUS,
                bucket.approach_x,
                bucket.approach_y,
            )
            .map_err(|err| format!("01 astar: {err}"))?;
            if route
                .iter()
                .any(|point| (point[0] - 2.4).hypot(point[1] - 1.05) < 0.85)
            {
                return Err("01 route clips the loose ball".into());
            }
            let ball = find_class_at(objects, "ball", 2.4, 1.05)?;
            if ball.carried || ball.goal_eligible {
                return Err("01 ball must not be a goal or carried".into());
            }
            Ok(())
        },
    )
}

fn scenario_bucket_behind() -> Result<Scenario, String> {
    let mut map = blank();
    fill_rect(&mut map, -3.0, -3.2, 7.5, 3.4, 0);
    fill_rect(&mut map, 1.7, -3.2, 2.25, 0.55, 100);
    fill_disk(&mut map, 4.3, -0.9, 0.30, 100);
    let pose = pose_origin();
    assemble(
        Draft {
            id: "02-bucket-behind-obstacle",
            title: "Bucket behind an obstacle",
            notes: "Label: waypoint on the free side of the wall. A straight line into the wall is not a goal.",
            level: Level::Waypoint,
            pose,
            carrying_balls: false,
            timestamp_s: 10.1,
            detections: vec![Detection {
                class: "bucket",
                x: 4.3,
                y: -0.9,
                confidence: 0.9,
                age_s: 0.3,
                carried: false,
            }],
            map,
            decision: DecisionKind::Waypoint,
            accept: vec![DecisionKind::Waypoint],
            dump: false,
            waypoint_required: true,
            target: Target::At("bucket", 4.3, -0.9),
        },
        |map, pose, objects| {
            let bucket = find_class_at(objects, "bucket", 4.3, -0.9)?;
            if !geom::segment_hits_occupied(map, [pose.x, pose.y], [bucket.x, bucket.y]) {
                return Err("02 wall does not block the straight line to the bucket".into());
            }
            let route = astar_route(
                map,
                *pose,
                FOOTPRINT_RADIUS,
                bucket.approach_x,
                bucket.approach_y,
            )
            .map_err(|err| format!("02 astar: {err}"))?;
            if !route.iter().any(|point| point[1] > 1.2) {
                return Err("02 route did not go around the wall".into());
            }
            Ok(())
        },
    )
}

fn scenario_person() -> Result<Scenario, String> {
    let mut map = blank();
    open_corridor(&mut map);
    fill_disk(&mut map, 2.05, 0.0, 0.50, 100);
    fill_disk(&mut map, 4.6, 0.15, 0.28, 100);
    let pose = pose_origin();
    assemble(
        Draft {
            id: "03-person-in-path",
            title: "Person in the path",
            notes: "Label: waypoint around the person to the bucket, or hold, or ask_operator. Never a waypoint on the person.",
            level: Level::Waypoint,
            pose,
            carrying_balls: false,
            timestamp_s: 10.2,
            detections: vec![
                Detection {
                    class: "person",
                    x: 2.05,
                    y: 0.0,
                    confidence: 0.96,
                    age_s: 0.1,
                    carried: false,
                },
                Detection {
                    class: "bucket",
                    x: 4.6,
                    y: 0.15,
                    confidence: 0.88,
                    age_s: 0.2,
                    carried: false,
                },
            ],
            map,
            decision: DecisionKind::Waypoint,
            accept: vec![
                DecisionKind::Waypoint,
                DecisionKind::Hold,
                DecisionKind::AskOperator,
            ],
            dump: false,
            waypoint_required: false,
            target: Target::At("bucket", 4.6, 0.15),
        },
        |map, pose, objects| {
            let person = find_class_at(objects, "person", 2.05, 0.0)?;
            let bucket = find_class_at(objects, "bucket", 4.6, 0.15)?;
            if person.goal_eligible {
                return Err("03 person is not a goal".into());
            }
            if !geom::segment_hits_occupied(map, [pose.x, pose.y], [bucket.x, bucket.y]) {
                return Err("03 person does not block the straight line".into());
            }
            let route = astar_route(
                map,
                *pose,
                FOOTPRINT_RADIUS,
                bucket.approach_x,
                bucket.approach_y,
            )
            .map_err(|err| format!("03 astar: {err}"))?;
            if route
                .iter()
                .any(|point| (point[0] - person.x).hypot(point[1] - person.y) < 0.9)
            {
                return Err("03 route enters the person footprint".into());
            }
            if !route.iter().any(|point| point[1].abs() > 1.0) {
                return Err("03 route did not go around the person".into());
            }
            Ok(())
        },
    )
}

fn scenario_frontier() -> Result<Scenario, String> {
    let mut map = blank();
    fill_rect(&mut map, -1.6, -1.5, 4.0, 1.5, 0);
    let pose = pose_origin();
    assemble(
        Draft {
            id: "04-unknown-frontier",
            title: "Unknown frontier",
            notes: "Label: hold, or a waypoint on a frontier viewpoint in observed free space. No invented bucket.",
            level: Level::Waypoint,
            pose,
            carrying_balls: false,
            timestamp_s: 10.3,
            detections: Vec::new(),
            map,
            decision: DecisionKind::Waypoint,
            accept: vec![DecisionKind::Waypoint, DecisionKind::Hold],
            dump: false,
            waypoint_required: false,
            target: Target::Frontiers,
        },
        |map, _pose, objects| {
            let counts = geom::counts(map);
            if counts.unknown <= counts.free {
                return Err("04 map is not mostly unknown".into());
            }
            if objects
                .iter()
                .any(|object| object.class == "bucket" || object.class == "person")
            {
                return Err("04 invented a detection".into());
            }
            if !objects
                .iter()
                .any(|object| object.class == "frontier" && object.reachable)
            {
                return Err("04 has no reachable frontier".into());
            }
            Ok(())
        },
    )
}

fn scenario_two_buckets() -> Result<Scenario, String> {
    let mut map = blank();
    fill_rect(&mut map, -3.0, -4.6, 7.5, 3.6, 0);
    fill_rect(&mut map, 2.3, -3.7, 5.5, -0.5, 100);
    fill_rect(&mut map, 2.8, -3.2, 5.0, -1.0, 0);
    fill_disk(&mut map, 3.9, -2.1, 0.24, 100);
    fill_disk(&mut map, 3.9, 1.7, 0.28, 100);
    let pose = pose_origin();
    assemble(
        Draft {
            id: "05-two-buckets",
            title: "Two buckets, one enclosed",
            notes: "Label: waypoint only to the reachable bucket. The other bucket is walled in. Lower y sorts as B1, so the reachable bucket is not B1.",
            level: Level::Waypoint,
            pose,
            carrying_balls: false,
            timestamp_s: 10.4,
            detections: vec![
                Detection {
                    class: "bucket",
                    x: 3.9,
                    y: -2.1,
                    confidence: 0.86,
                    age_s: 0.4,
                    carried: false,
                },
                Detection {
                    class: "bucket",
                    x: 3.9,
                    y: 1.7,
                    confidence: 0.91,
                    age_s: 0.4,
                    carried: false,
                },
            ],
            map,
            decision: DecisionKind::Waypoint,
            accept: vec![DecisionKind::Waypoint],
            dump: false,
            waypoint_required: true,
            target: Target::At("bucket", 3.9, 1.7),
        },
        |map, pose, objects| {
            let enclosed = find_class_at(objects, "bucket", 3.9, -2.1)?;
            let open = find_class_at(objects, "bucket", 3.9, 1.7)?;
            if enclosed.reachable {
                return Err(format!(
                    "05 enclosed bucket {} was marked reachable at ({:.2}, {:.2})",
                    enclosed.id, enclosed.approach_x, enclosed.approach_y
                ));
            }
            if astar_route(map, *pose, FOOTPRINT_RADIUS, enclosed.x, enclosed.y).is_ok() {
                return Err("05 A* reached the enclosed bucket centroid".into());
            }
            if !open.reachable {
                return Err("05 open bucket is not reachable".into());
            }
            if enclosed.y > open.y {
                return Err("05 expected the enclosed bucket to have the smaller y".into());
            }
            Ok(())
        },
    )
}

fn scenario_dump() -> Result<Scenario, String> {
    let mut map = blank();
    open_corridor(&mut map);
    fill_disk(&mut map, 1.40, 0.0, 0.26, 100);
    let pose = pose_origin();
    assemble(
        Draft {
            id: "06-dump-at-bucket",
            title: "Carrying balls at a bucket",
            notes: "Label: hold and dump_proposal true. The proposal is not a servo command.",
            level: Level::Waypoint,
            pose,
            carrying_balls: true,
            timestamp_s: 10.5,
            detections: vec![
                Detection {
                    class: "bucket",
                    x: 1.40,
                    y: 0.0,
                    confidence: 0.92,
                    age_s: 0.2,
                    carried: false,
                },
                Detection {
                    class: "ball",
                    x: 0.0,
                    y: 0.0,
                    confidence: 0.97,
                    age_s: 0.1,
                    carried: true,
                },
            ],
            map,
            decision: DecisionKind::Hold,
            accept: vec![DecisionKind::Hold],
            dump: true,
            waypoint_required: false,
            target: Target::None,
        },
        |_map, pose, objects| {
            let bucket = find_class_at(objects, "bucket", 1.40, 0.0)?;
            let distance = (bucket.x - pose.x).hypot(bucket.y - pose.y);
            if distance > crate::scene::DUMP_BUCKET_RANGE_M {
                return Err(format!("06 bucket is {distance:.2} m away"));
            }
            let ball = objects
                .iter()
                .find(|object| object.class == "ball")
                .ok_or("06 missing ball")?;
            if !ball.carried {
                return Err("06 ball is not carried".into());
            }
            Ok(())
        },
    )
}

fn scenario_low_confidence() -> Result<Scenario, String> {
    let mut map = blank();
    open_corridor(&mut map);
    fill_disk(&mut map, 4.0, 1.1, 0.28, 100);
    let pose = pose_origin();
    assemble(
        Draft {
            id: "07-low-confidence",
            title: "Low confidence, ask the operator",
            notes: "Label: ask_operator. The only bucket confidence is below the prompt rule of 0.4. That rule is not a calibrated threshold.",
            level: Level::Waypoint,
            pose,
            carrying_balls: false,
            timestamp_s: 10.6,
            detections: vec![Detection {
                class: "bucket",
                x: 4.0,
                y: 1.1,
                confidence: 0.18,
                age_s: 0.5,
                carried: false,
            }],
            map,
            decision: DecisionKind::AskOperator,
            accept: vec![DecisionKind::AskOperator],
            dump: false,
            waypoint_required: false,
            target: Target::None,
        },
        |map, pose, objects| {
            let bucket = find_class_at(objects, "bucket", 4.0, 1.1)?;
            if bucket.confidence >= crate::scene::CONFIDENCE_ASK_BELOW {
                return Err("07 confidence is not low".into());
            }
            if !bucket.reachable {
                return Err(
                    "07 low-confidence bucket should still be geometrically reachable".into(),
                );
            }
            astar_route(
                map,
                *pose,
                FOOTPRINT_RADIUS,
                bucket.approach_x,
                bucket.approach_y,
            )
            .map_err(|err| format!("07 astar: {err}"))?;
            Ok(())
        },
    )
}

fn scenario_l0() -> Result<Scenario, String> {
    let mut map = blank();
    open_corridor(&mut map);
    fill_disk(&mut map, 4.2, 0.0, 0.30, 100);
    fill_disk(&mut map, 2.4, 1.05, 0.15, 100);
    let pose = pose_origin();
    assemble(
        Draft {
            id: "08-l0-no-goal",
            title: "L0 trial, no goal",
            notes: "Label: hold or ask_operator. dump_proposal false. A waypoint is rejected because autonomy is teleop (L0), even though the bucket is reachable.",
            level: Level::Teleop,
            pose,
            carrying_balls: false,
            timestamp_s: 10.7,
            detections: vec![
                Detection {
                    class: "bucket",
                    x: 4.2,
                    y: 0.0,
                    confidence: 0.93,
                    age_s: 0.2,
                    carried: false,
                },
                Detection {
                    class: "ball",
                    x: 2.4,
                    y: 1.05,
                    confidence: 0.77,
                    age_s: 0.2,
                    carried: false,
                },
            ],
            map,
            decision: DecisionKind::Hold,
            accept: vec![DecisionKind::Hold, DecisionKind::AskOperator],
            dump: false,
            waypoint_required: false,
            target: Target::None,
        },
        |map, pose, objects| {
            let bucket = find_class_at(objects, "bucket", 4.2, 0.0)?;
            if !bucket.reachable {
                return Err("08 tempting bucket should be reachable".into());
            }
            astar_route(
                map,
                *pose,
                FOOTPRINT_RADIUS,
                bucket.approach_x,
                bucket.approach_y,
            )
            .map_err(|err| format!("08 astar: {err}"))?;
            Ok(())
        },
    )
}

fn pose_origin() -> Pose {
    Pose {
        x: 0.0,
        y: 0.0,
        yaw: 0.0,
    }
}

fn assemble(
    draft: Draft,
    check: impl FnOnce(&MapSnapshot, &Pose, &[SceneObject]) -> Result<(), String>,
) -> Result<Scenario, String> {
    let objects = objects::build_objects(&draft.map, draft.pose, &draft.detections)
        .map_err(|err| format!("{}: {err}", draft.id))?;
    check(&draft.map, &draft.pose, &objects).map_err(|err| format!("{}: {err}", draft.id))?;
    let (target_id, waypoints) =
        resolve_target(&objects, &draft.target).map_err(|err| format!("{}: {err}", draft.id))?;
    if draft.waypoint_required && waypoints.is_empty() {
        return Err(format!("{}: label waypoint missing", draft.id));
    }
    if draft.decision == DecisionKind::Waypoint {
        let id = target_id
            .as_deref()
            .ok_or_else(|| format!("{}: mock waypoint has no target", draft.id))?;
        let object = objects::object_by_id(&objects, id)
            .ok_or_else(|| format!("{}: missing {id}", draft.id))?;
        astar_route(
            &draft.map,
            draft.pose,
            FOOTPRINT_RADIUS,
            object.approach_x,
            object.approach_y,
        )
        .map_err(|err| format!("{}: label target {id} astar: {err}", draft.id))?;
    }
    let camera = CameraModel::standard();
    let map_png = render_map_png(&draft.map, draft.pose, &camera, &objects)?;
    let camera_png = render_camera_png(draft.id, draft.title, draft.pose, &camera, &objects)?;
    let scenario = Scenario {
        id: draft.id.to_string(),
        title: draft.title.to_string(),
        notes: draft.notes.to_string(),
        camera_source: CAMERA_SOURCE_PLACEHOLDER.to_string(),
        camera_source_note: CAMERA_SOURCE_NOTE.to_string(),
        counts: geom::counts(&draft.map),
        symbolic_grid: geom::symbolic_grid(&draft.map, draft.pose.x, draft.pose.y),
        rover: RoverState {
            x: draft.pose.x,
            y: draft.pose.y,
            yaw: draft.pose.yaw,
            carrying_balls: draft.carrying_balls,
            timestamp_s: draft.timestamp_s,
            map_revision: 1,
        },
        level: draft.level,
        camera,
        expected: ExpectedLabel {
            decision: draft.decision,
            accept: draft.accept,
            target_id,
            dump_proposal: draft.dump,
            waypoint_required: draft.waypoint_required,
            waypoints,
        },
        map_png_sha256: sha256_hex(&map_png),
        camera_png_sha256: sha256_hex(&camera_png),
        map: draft.map,
        map_png,
        camera_png,
        objects,
    };
    if scenario.map_png.is_empty() || scenario.camera_png.is_empty() {
        return Err(format!("{}: empty render", scenario.id));
    }
    Ok(scenario)
}

fn resolve_target(
    objects: &[SceneObject],
    target: &Target,
) -> Result<(Option<String>, Vec<[f64; 2]>), String> {
    match target {
        Target::None => Ok((None, Vec::new())),
        Target::At(class, x, y) => {
            let object = find_class_at(objects, class, *x, *y)?;
            Ok((
                Some(object.id.clone()),
                vec![[object.approach_x, object.approach_y]],
            ))
        }
        Target::Frontiers => {
            let frontiers: Vec<&SceneObject> = objects
                .iter()
                .filter(|object| object.class == "frontier" && object.reachable)
                .collect();
            if frontiers.is_empty() {
                return Err("no reachable frontier to label".into());
            }
            let points = frontiers
                .iter()
                .map(|object| [object.approach_x, object.approach_y])
                .collect();
            Ok((Some(frontiers[0].id.clone()), points))
        }
    }
}

fn find_class_at<'a>(
    objects: &'a [SceneObject],
    class: &str,
    x: f64,
    y: f64,
) -> Result<&'a SceneObject, String> {
    objects
        .iter()
        .find(|object| object.class == class && (object.x - x).hypot(object.y - y) < 0.6)
        .ok_or_else(|| format!("no {class} near ({x:.2}, {y:.2})"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::decision::{
        DecisionKind, ModelDecision, mock_decision, score_output, validate_waypoint,
    };
    use crate::prompt::user_prompt;
    use serde_json::json;

    /// The eight-case pack covers its required decision categories, keeps labels
    /// out of prompts, records placeholder provenance, and passes its mock labels.
    #[test]
    fn pack_covers_the_eight_labels() {
        let pack = scenarios();
        assert_eq!(pack.len(), 8);
        assert!(pack.iter().any(|scenario| scenario.expected.dump_proposal));
        assert!(pack.iter().any(|scenario| scenario.level == Level::Teleop));
        assert!(
            pack.iter()
                .any(|scenario| scenario.expected.decision == DecisionKind::AskOperator)
        );
        assert!(
            pack.iter()
                .all(|scenario| scenario.camera_source == "placeholder")
        );
        assert!(
            pack.iter()
                .all(|scenario| scenario.map.width == 200 && scenario.map.resolution == 0.1)
        );
        for scenario in pack {
            assert!(!user_prompt(scenario).contains("Label:"));
            assert!(scenario.notes.starts_with("Label:"));
            let score = score_output(&mock_decision(scenario), scenario);
            assert!(
                score.passed(),
                "{} mock failed: {:?}",
                scenario.id,
                score.notes
            );
        }
    }

    /// Reject enclosed targets and L0 waypoint proposals, ignoring invented model
    /// coordinates in favor of object-list points and enforcing scenario labels.
    #[test]
    fn enclosed_bucket_and_l0_waypoint_are_rejected() {
        let pack = scenarios();
        let two = pack
            .iter()
            .find(|scenario| scenario.id == "05-two-buckets")
            .unwrap();
        let enclosed = two
            .objects
            .iter()
            .find(|object| object.class == "bucket" && !object.reachable)
            .unwrap();
        let open = two
            .objects
            .iter()
            .find(|object| object.class == "bucket" && object.reachable)
            .unwrap();
        assert_eq!(enclosed.id, "B1");
        assert_eq!(open.id, "B2");
        let bad = ModelDecision {
            decision: DecisionKind::Waypoint,
            target_id: Some(enclosed.id.clone()),
            waypoint: None,
            dump_proposal: false,
            rationale: "enclosed".into(),
            confidence: 0.5,
        };
        assert!(validate_waypoint(&bad, two).is_err());
        let l0 = pack
            .iter()
            .find(|scenario| scenario.id == "08-l0-no-goal")
            .unwrap();
        let bucket = l0
            .objects
            .iter()
            .find(|object| object.class == "bucket")
            .unwrap();
        let goal = ModelDecision {
            decision: DecisionKind::Waypoint,
            target_id: Some(bucket.id.clone()),
            waypoint: None,
            dump_proposal: false,
            rationale: "goal during teleop".into(),
            confidence: 0.5,
        };
        let score = score_output(&serde_json::to_string(&goal).unwrap(), l0);
        assert!(!score.passed());
        assert!(score.notes.iter().any(|note| note.contains("L3")));
        let raw = json!({
            "decision": "hold",
            "target_id": null,
            "waypoint": null,
            "dump_proposal": true,
            "rationale": "pulse the servo in prose only",
            "confidence": 0.4,
            "servo": 90
        });
        let dumped = score_output(&raw.to_string(), l0);
        assert!(!dumped.no_actuator);
        assert!(!dumped.valid_json);
        let prose = json!({
            "decision": "hold",
            "target_id": null,
            "waypoint": null,
            "dump_proposal": false,
            "rationale": "do not command a servo",
            "confidence": 0.4
        });
        assert!(score_output(&prose.to_string(), l0).passed());
        let person_case = pack
            .iter()
            .find(|scenario| scenario.id == "03-person-in-path")
            .unwrap();
        let person = person_case
            .objects
            .iter()
            .find(|object| object.class == "person")
            .unwrap();
        assert!(!person.goal_eligible);
        let on_person = json!({
            "decision": "waypoint",
            "target_id": person.id,
            "waypoint": null,
            "dump_proposal": false,
            "rationale": "drive at P1",
            "confidence": 0.4
        });
        assert!(!score_output(&on_person.to_string(), person_case).passed());
        let hold = json!({
            "decision": "hold",
            "target_id": null,
            "waypoint": null,
            "dump_proposal": false,
            "rationale": "person in the path",
            "confidence": 0.4
        });
        assert!(score_output(&hold.to_string(), person_case).passed());
    }

    /// Checked-in images, prompts, packets, and manifest must match regenerated
    /// fixture content, with expected labels present only in scoring documents.
    #[test]
    fn committed_fixtures_match_the_generator() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("fixtures");
        for scenario in scenarios() {
            let dir = root.join(&scenario.id);
            let map = std::fs::read(dir.join("map.png")).expect(&scenario.id);
            assert_eq!(sha256_hex(&map), scenario.map_png_sha256, "{}", scenario.id);
            let camera = std::fs::read(dir.join("camera.png")).unwrap();
            assert_eq!(sha256_hex(&camera), scenario.camera_png_sha256);
            let prompt = std::fs::read_to_string(dir.join("prompt.txt")).unwrap();
            assert_eq!(prompt, user_prompt(scenario));
            let document: serde_json::Value =
                serde_json::from_slice(&std::fs::read(dir.join("scenario.json")).unwrap()).unwrap();
            assert_eq!(document["id"], scenario.id);
            assert_eq!(document["camera_source"], "placeholder");
            assert!(document.get("expected").is_some());
            assert!(!prompt.contains("\"expected\""));
        }
        let manifest: serde_json::Value =
            serde_json::from_slice(&std::fs::read(root.join("manifest.json")).unwrap()).unwrap();
        assert_eq!(manifest["scenarios"].as_array().unwrap().len(), 8);
    }
}
