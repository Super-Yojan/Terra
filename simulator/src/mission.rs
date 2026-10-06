//! Seeded disaster-search layout. Target positions remain in the mission model, not rendered.
use avian3d::prelude::*;
use bevy::prelude::*;
#[derive(Component)]
pub struct MissionObstacle;
pub fn install(app: &mut App) {
    if std::env::var("TERRA_MISSION").as_deref() == Ok("1") && !crate::next_competition::enabled() {
        app.add_systems(Startup, spawn_layout);
    }
}
fn spawn_layout(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let seed = std::env::var("TERRA_MISSION_SEED")
        .ok()
        .and_then(|v| v.parse::<u64>().ok())
        .unwrap_or(42);
    let mesh = meshes.add(Cuboid::new(1., 1., 1.));
    let material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.42, 0.40, 0.36),
        perceptual_roughness: 1.,
        ..default()
    });
    // Broken walls form a chokepoint and an alternate open route; centre/spawn and objectives stay clear.
    for (x, y, sx, sy, h) in layout(seed) {
        commands.spawn((
            Name::new("Mission rubble"),
            MissionObstacle,
            RigidBody::Static,
            Collider::cuboid(sx, h, sy),
            Mesh3d(mesh.clone()),
            MeshMaterial3d(material.clone()),
            Transform::from_xyz(-y, h * 0.5, -x).with_scale(Vec3::new(sx, h, sy)),
        ));
    }
}
pub fn layout(seed: u64) -> Vec<(f32, f32, f32, f32, f32)> {
    let shift = (seed % 3) as f32;
    vec![
        (5., -6., 1., 7., 1.5),
        (5., 5., 1., 6., 1.5),
        (-6., 8., 6., 1., 1.2),
        (12., -8. - shift, 3., 2., 0.8),
        (-15., -10., 5., 3., 1.8),
    ]
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn layout_is_reproducible_and_keeps_targets_clear() {
        assert_eq!(layout(42), layout(42));
        let mission = terra_experiment::MissionConfig::rescue(42);
        for target in mission.survivors {
            for (x, y, sx, sy, _) in layout(42) {
                assert!(
                    (target.x - x as f64).abs() > sx as f64 / 2. + 0.65
                        || (target.y - y as f64).abs() > sy as f64 / 2. + 0.65
                );
            }
        }
    }
}
