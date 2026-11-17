import Foundation
import simd

// The one boundary between Swift and Rust. Swift fills a SensorFrame each
// tick; terra-av returns the wheel command. These types mirror
// crates/terra-av/src/sense.rs, planning/mod.rs and execution/mod.rs.

/// Raw phone readings, in each sensor's own frame (terra_av::sense::SensorFrame).
struct SensorFrame {
    var t: Double
    var estop: Bool
    var vio: VioPose?
    var gyro: SIMD3<Double>?
    var gps: GpsFix?
    var depth: [SIMD3<Float>]
}

/// ARKit camera.transform: camera → ARKit world.
struct VioPose {
    var position: SIMD3<Double>
    var rotation: simd_double3x3
    var tracked: Bool
}

struct GpsFix { var latitude: Double; var longitude: Double; var accuracy: Double }

/// terra_av::planning::Goal.
enum Goal {
    case none
    case velocity(v: Double, w: Double)
    case pose(x: Double, y: Double, yaw: Double)
}

/// Wheel efforts in [-1, 1] (terra_av::execution::WheelCommand).
struct WheelCommand: Equatable {
    var left: Double
    var right: Double
    static let stop = WheelCommand(left: 0, right: 0)
}

/// Owns one terra-av Pipeline, built from the bundled terra-av.toml.
final class TerraAV {
    init(configTOML: String) {
        // TODO: terra_av::config::PipelineConfig::from_toml(configTOML).build()
        // once terra-av exposes its UniFFI bindings.
    }

    /// One control tick: sense → estimate → map → plan → execute → safety.
    func tick(_ frame: SensorFrame, goal: Goal) -> WheelCommand {
        // TODO: call Pipeline::tick through the generated bindings. Until the
        // Rust side is implemented this stays stopped.
        return .stop
    }
}
