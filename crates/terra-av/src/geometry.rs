//! Shared foundation: rotation matrices and poses (roadmap 1).

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// 2×2 rotation matrix.
pub struct Rotation2 {
    m: [[f64; 2]; 2],
}
/// 3×3 rotation matrix, for sensor → body frame changes (phone mount, IMU).
pub struct Rotation3 {
    m: [[f64; 3]; 3],
}

/// Planar pose: position and heading `yaw` (rad, + = left of +X).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose2 {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

impl Rotation2 {
    pub fn new(theta: f64) -> Self {
        // This is the normal 2d rotation matrix
        // R(\theta) = [cos \theta -sin \theta]
        //         [sin \theta cos\theta]
        let (s, c) = theta.sin_cos();
        Self {
            m: [[c, -s], [s, c]],
        }
    }
    pub fn transpose(&self) -> Self {
        // For a rotation matrix transpose = inverse(rotates by -\theta)
        let m = self.m;
        Self {
            m: [[m[0][0], m[1][0]], [m[0][1], m[1][1]]],
        }
    }
    pub fn apply(&self, v: Vec2) -> Vec2 {
        let m = self.m;
        Vec2 {
            x: m[0][0] * v.x + m[0][1] * v.y,
            y: m[1][0] * v.x + m[1][1] * v.y,
        }
    }
}

impl Rotation3 {
    /// Z-Y-X Euler angles in radians: R = Rz(yaw) · Ry(pitch) · Rx(roll).
    /// The config stores the mount in degrees; convert with `to_radians()`.
    pub fn from_euler(roll: f64, pitch: f64, yaw: f64) -> Self {
        let (sr, cr) = roll.sin_cos();
        let (sp, cp) = pitch.sin_cos();
        let (sy, cy) = yaw.sin_cos();
        Self {
            m: [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ],
        }
    }
    /// Row-major 3×3, e.g. the rotation part of ARKit's camera transform.
    pub fn from_matrix(m: [[f64; 3]; 3]) -> Self {
        Self { m }
    }
    /// For a rotation matrix, transpose = inverse.
    pub fn transpose(&self) -> Self {
        let mut t = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                t[i][j] = self.m[j][i];
            }
        }
        Self { m: t }
    }
    pub fn apply(&self, v: Vec3) -> Vec3 {
        let m = self.m;
        Vec3 {
            x: m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            y: m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            z: m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z,
        }
    }
}

impl Pose2 {
    /// Body-frame point → world.
    pub fn to_world(&self, p: Vec2) -> Vec2 {
        // Rotate by yaw, then shift by the position.
        let r = Rotation2::new(self.yaw).apply(p);
        Vec2 {
            x: r.x + self.x,
            y: r.y + self.y,
        }
    }
    /// World point → body frame (+X forward, +Y left).
    pub fn to_body(&self, p: Vec2) -> Vec2 {
        // Undo to_world: shift back, then rotate by −yaw.
        let d = Vec2 {
            x: p.x - self.x,
            y: p.y - self.y,
        };
        Rotation2::new(self.yaw).transpose().apply(d)
    }
}
