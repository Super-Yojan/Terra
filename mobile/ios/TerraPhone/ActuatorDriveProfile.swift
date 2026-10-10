import Foundation

/// Runtime routing data only. Editable calibration and port assignments stay on the rover.
struct ActuatorDriveProfile: Codable {
    let id: Int
    let kind: String
    let limits: ActuatorLimits
    let route: ActuatorRoute
    let safe: ActuatorSafe

    func validate() throws {
        let servo = kind == "positional_servo"
        guard (0...255).contains(id), ["dc_motor", "bidirectional_esc", "unidirectional_esc", "positional_servo"].contains(kind),
              limits.min.isFinite, limits.max.isFinite, limits.min >= -1, limits.max <= 1, limits.min <= limits.max,
              kind != "unidirectional_esc" || limits.min >= 0,
              ["left_effort", "right_effort", "manual", "servo"].contains(route.type), servo == (route.type == "servo") else {
            throw ActuatorProfileError.invalid
        }
        if route.type == "manual" {
            guard let forward = route.forward_coefficient, let turn = route.turn_coefficient,
                  forward.isFinite, turn.isFinite, abs(forward) <= 1, abs(turn) <= 1 else { throw ActuatorProfileError.invalid }
        } else if route.forward_coefficient != nil || route.turn_coefficient != nil { throw ActuatorProfileError.invalid }
        switch safe.type {
        case "zero":
            guard !servo, safe.value == nil, limits.min <= 0, limits.max >= 0 else { throw ActuatorProfileError.invalid }
        case "position":
            guard servo, let value = safe.value, value.isFinite, value >= limits.min, value <= limits.max else { throw ActuatorProfileError.invalid }
        case "disabled":
            guard servo, safe.value == nil else { throw ActuatorProfileError.invalid }
        default: throw ActuatorProfileError.invalid
        }
    }

    var safeValue: Double { min(limits.max, max(limits.min, safe.value ?? 0)) }
}

enum ActuatorProfileError: Error { case invalid, staleRevision }

struct ActuatorDriveProfileSet: Codable {
    var schema_version = 1
    let revision: UInt32
    let actuators: [ActuatorDriveProfile]

    static func decode(json: String) throws -> Self {
        let value = try JSONDecoder().decode(Self.self, from: Data(json.utf8))
        guard value.schema_version == 1, value.actuators.count <= 16,
              Set(value.actuators.map(\.id)).count == value.actuators.count else { throw ActuatorProfileError.invalid }
        try value.actuators.forEach { try $0.validate() }
        return value
    }

    var safeValues: [[String: Any]] { actuators.map { ["id": $0.id, "value": $0.safeValue] } }
    var supportsFeedback: Bool {
        actuators.contains { $0.route.type == "left_effort" }
            && actuators.contains { $0.route.type == "right_effort" }
            && !actuators.contains { $0.route.type == "manual" }
    }

    func route(input: [String: Any]) throws -> [[String: Any]] {
        let keys = ["left_effort", "right_effort", "forward", "turn"]
        var values: [String: Double] = [:]
        for key in keys {
            guard let number = input[key] as? Double, number.isFinite, abs(number) <= 1 else { throw ActuatorProfileError.invalid }
            values[key] = number
        }
        let servos = input["servo_positions"] as? [String: Double] ?? [:]
        for (id, value) in servos {
            guard value.isFinite, abs(value) <= 1, let identifier = Int(id),
                  actuators.contains(where: { $0.id == identifier && $0.route.type == "servo" }) else { throw ActuatorProfileError.invalid }
        }
        return actuators.map { actuator in
            let value: Double
            switch actuator.route.type {
            case "left_effort": value = values["left_effort"]!
            case "right_effort": value = values["right_effort"]!
            case "manual": value = actuator.route.forward_coefficient! * values["forward"]! + actuator.route.turn_coefficient! * values["turn"]!
            default: value = servos[String(actuator.id)] ?? actuator.safeValue
            }
            return ["id": actuator.id, "value": min(actuator.limits.max, max(actuator.limits.min, value))]
        }
    }
}

/// Collects one revision of runtime descriptors before making drive data available.
struct ActuatorProfileSynchronization {
    private(set) var revision: UInt32?
    private(set) var remaining: [Int] = []
    private var profiles: [ActuatorDriveProfile] = []
    mutating func begin(revision: UInt32, ids: [Int]) throws {
        guard ids.count <= 16, Set(ids).count == ids.count, ids.allSatisfy({ (0...255).contains($0) }) else { throw ActuatorProfileError.invalid }
        self.revision = revision; remaining = ids; profiles = []
    }
    mutating func accept(revision: UInt32, actuator: ActuatorDriveProfile) throws {
        guard self.revision == revision else { throw ActuatorProfileError.staleRevision }
        guard remaining.first == actuator.id else { throw ActuatorProfileError.invalid }
        try actuator.validate(); profiles.append(actuator); remaining.removeFirst()
    }
    var completed: ActuatorDriveProfileSet? {
        guard let revision, remaining.isEmpty else { return nil }
        return ActuatorDriveProfileSet(revision: revision, actuators: profiles)
    }
    mutating func clear() { self = Self() }
}
