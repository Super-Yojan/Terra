import Foundation

struct ActuatorLayoutDraft: Codable, Equatable {
    var schema_version = 1
    var revision: UInt32 = 0
    var actuators: [ActuatorDraft] = []
    static func rover1(revision: UInt32 = 0) -> Self {
        let actuators = (0..<2).map { id in
            var actuator = ActuatorDraft(id: id, name: id == 0 ? "Left ESC" : "Right ESC")
            actuator.changeKind("bidirectional_esc")
            actuator.port = id == 0 ? "P0" : "P1"
            actuator.route = ActuatorRoute(type: id == 0 ? "left_effort" : "right_effort")
            return actuator
        }
        return Self(revision: revision, actuators: actuators)
    }
}
struct ActuatorDraft: Codable, Equatable {
    var id: Int
    var name: String
    var port = ""
    var kind = "dc_motor"
    var inverted = false
    var limits = ActuatorLimits(min: -1, max: 1)
    var calibration = ActuatorCalibration(type: "dc_motor", max_power_fraction: 0.5)
    var route = ActuatorRoute(type: "manual", forward_coefficient: 1, turn_coefficient: 0)
    var safe = ActuatorSafe(type: "zero")
    mutating func changeKind(_ value: String) {
        kind = value
        if value == "unidirectional_esc" { inverted = false }
        limits = ActuatorLimits(min: value == "unidirectional_esc" ? 0 : -1, max: 1)
        route = value == "positional_servo" ? ActuatorRoute(type: "servo") : ActuatorRoute(type: "manual", forward_coefficient: 1, turn_coefficient: 0)
        safe = ActuatorSafe(type: value == "positional_servo" ? "position" : "zero", value: value == "positional_servo" ? 0 : nil)
        switch value {
        case "bidirectional_esc": calibration = ActuatorCalibration(type: value, reverse_us: 1000, neutral_us: 1500, forward_us: 2000, arming_duration_ms: 2000)
        case "unidirectional_esc": calibration = ActuatorCalibration(type: value, stop_us: 1000, full_power_us: 2000, arming_duration_ms: 2000)
        case "positional_servo": calibration = ActuatorCalibration(type: value, min_us: 1000, center_us: 1500, max_us: 2000)
        default: calibration = ActuatorCalibration(type: value, max_power_fraction: 0.5)
        }
    }
}
struct ActuatorLimits: Codable, Equatable { var min: Double; var max: Double }
struct ActuatorCalibration: Codable, Equatable {
    var type: String
    var max_power_fraction: Double?
    var reverse_us: Int?; var neutral_us: Int?; var forward_us: Int?
    var stop_us: Int?; var full_power_us: Int?; var arming_duration_ms: Int?
    var min_us: Int?; var center_us: Int?; var max_us: Int?
}
struct ActuatorRoute: Codable, Equatable { var type: String; var forward_coefficient: Double?; var turn_coefficient: Double? }
struct ActuatorSafe: Codable, Equatable { var type: String; var value: Double? }
struct ActuatorCapabilities: Decodable {
    struct Port: Decodable { var kinds: [String]; var resources: [String]; var frequency_hz: Int }
    var board: String; var library: String; var library_version: String
    var supported_kinds: [String]; var ports: [String: Port]
}

enum ActuatorConfigurationPolicy {
    static func blockingReason(ready: Bool, status: [String: Any]) -> String? {
        guard ready else { return "Waiting for rover connection and output capabilities. You can edit the draft now." }
        guard status["armed"] as? Bool == false, status["arming"] as? Bool == false else {
            return "Disarm the rover before applying configuration."
        }
        if let allowed = status["configuration_allowed"] as? Bool {
            if allowed { return nil }
            if status["gate_mode"] as? String == "bench", status["bench_enabled"] as? Bool == true {
                return "Disable bench control before applying configuration."
            }
            if status["gate_mode"] as? String == "physical" {
                return "The rover requires its Pi interlock to be open. Battery-switch rovers need the updated rover service."
            }
            return "The rover is not accepting configuration. Check its fault and connection status."
        }
        guard status["hardware_gate_open_confirmed"] as? Bool == true else {
            return "The rover requires a confirmed Pi interlock. Battery-switch rovers need the updated rover service."
        }
        return nil
    }
}
