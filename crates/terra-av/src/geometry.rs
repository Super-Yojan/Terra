//! Shared foundation: vectors, rotation matrices, and planar poses.
//!
//! # Coordinate conventions
//! Terra uses right-handed, Z-up frames (FLU — Forward / Left / Up):
//! - `+X` forward, `+Y` left, `+Z` up.
//! - Positive `yaw` rotates counter-clockwise when viewed from above (`+X` toward `+Y`, i.e. turning left).
//!
//! # Reading rotation matrices
//! Matrices are stored row-major (`m[row][col]`). A rotation `R` that maps vectors from
//! frame `A` into frame `B` (`R_{B←A}`) stores `A`'s unit axes expressed in frame `B` as its
//! **columns**. Because those columns are unit-length and mutually perpendicular (orthonormal),
//! `R⁻¹ = Rᵀ`, so [`Rotation2::transpose`] and [`Rotation3::transpose`] invert the rotation
//! (`R_{A←B} = R_{B←A}ᵀ`) without a matrix inversion.

/// 2D point or vector in the horizontal plane (metres when representing a position).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

/// 3D point or vector (metres for positions/offsets, rad/s for angular rates).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// 2×2 orthonormal rotation matrix stored row-major (`m[row][col]`).
pub struct Rotation2 {
    m: [[f64; 2]; 2],
}

/// 3×3 orthonormal rotation matrix stored row-major (`m[row][col]`),
/// used for 3D frame changes such as camera → body (phone mount) and ARKit → world.
#[derive(Debug)]
pub struct Rotation3 {
    m: [[f64; 3]; 3],
}

/// Planar rigid pose in the world frame: origin `(x, y)` in metres and heading
/// `yaw` in radians (`0` along `+X`, positive CCW / left of `+X`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose2 {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

impl Rotation2 {
    /// Counter-clockwise 2D rotation by `theta` radians (`+X` toward `+Y`).
    ///
    /// ```text
    ///          col 0 (x̂')    col 1 (ŷ')
    /// R(θ) = [  cos θ         -sin θ  ]
    ///        [  sin θ          cos θ  ]
    /// ```
    pub fn new(theta: f64) -> Self {
        let (s, c) = theta.sin_cos();
        Self {
            m: [[c, -s], [s, c]],
        }
    }

    /// Inverse rotation (`Rᵀ = R⁻¹ = R(-θ)`), since rotation matrices are orthonormal.
    pub fn transpose(&self) -> Self {
        let m = self.m;
        Self {
            m: [[m[0][0], m[1][0]], [m[0][1], m[1][1]]],
        }
    }

    /// Rotate `v`: computes `R · v = v.x · col_0 + v.y · col_1`.
    pub fn apply(&self, v: Vec2) -> Vec2 {
        let m = self.m;
        Vec2 {
            x: m[0][0] * v.x + m[0][1] * v.y,
            y: m[1][0] * v.x + m[1][1] * v.y,
        }
    }
}

impl Rotation3 {
    /// Build a 3D rotation from Z-Y-X Euler angles in radians:
    /// `R = Rz(yaw) · Ry(pitch) · Rx(roll)` (roll applied first, then pitch, then yaw).
    ///
    /// Config files (`terra-av.toml`) store mount angles in degrees; convert with `to_radians()`.
    pub fn from_euler(roll: f64, pitch: f64, yaw: f64) -> Self {
        let (sr, cr) = roll.sin_cos();
        let (sp, cp) = pitch.sin_cos();
        let (sy, cy) = yaw.sin_cos();
        // Expanded product of Rz(yaw) * Ry(pitch) * Rx(roll):
        Self {
            m: [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ],
        }
    }

    /// Construct directly from a row-major 3×3 matrix (`m[row][col]`),
    /// e.g. the rotation block of ARKit's `camera.transform`.
    pub fn from_matrix(m: [[f64; 3]; 3]) -> Self {
        Self { m }
    }

    /// Inverse rotation (`Rᵀ = R⁻¹`): flips direction from `R_{dst←src}` to `R_{src←dst}`.
    pub fn transpose(&self) -> Self {
        let mut t = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                t[i][j] = self.m[j][i];
            }
        }
        Self { m: t }
    }

    /// Rotate `v` into the destination frame (`R · v`).
    ///
    /// To chain multiple frame transforms (`W ← A ← C ← B`), call `.apply()` from
    /// right to left: `r_wa.apply(r_ac.apply(r_cb.apply(v_b)))`.
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
    /// Transform a point from the rover's body frame (`+X` forward, `+Y` left) into the world frame:
    /// `p^W = R(yaw) · p^B + t`.
    pub fn to_world(&self, p: Vec2) -> Vec2 {
        // 1. Rotate body vector into world orientation, then 2. shift by rover origin.
        let r = Rotation2::new(self.yaw).apply(p);
        Vec2 {
            x: r.x + self.x,
            y: r.y + self.y,
        }
    }

    /// Transform a world-frame point into the rover's body frame (`+X` forward, `+Y` left):
    /// `p^B = R(yaw)ᵀ · (p^W - t)`.
    pub fn to_body(&self, p: Vec2) -> Vec2 {
        // Undo `to_world` in reverse order: 1. subtract rover origin first, then 2. rotate by -yaw.
        let d = Vec2 {
            x: p.x - self.x,
            y: p.y - self.y,
        };
        Rotation2::new(self.yaw).transpose().apply(d)
    }
}
