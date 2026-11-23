import Foundation
import simd

// Initial vehicle configuration: what the rover is, which actuators drive it
// and which Fusion HAT port each one is wired to. Produces:
//  - the rover-pi actuator layout (apps/rover-pi/terra_rover/layout.py), sent
//    over Bluetooth with the layout edit transaction;
//  - the terra-av `[vehicle]` table (crates/terra-av/src/config.rs).
// Pure data, no UI or Bluetooth, so it can be checked on its own.

enum Steering: String, Codable, CaseIterable, Identifiable {
    case differential, ackermann
    var id: String { rawValue }
    var title: String { self == .differential ? "Differential" : "Ackermann" }
    var detail: String {
        self == .differential ? "Tank / skid steer: turns by driving the sides at different speeds"
                              : "Car-like: a servo steers the front wheels, the rear wheels drive"
    }
    /// Wheel counts this steering type supports.
    var wheelCounts: [Int] { self == .differential ? [2, 4, 6] : [4] }
}

/// The actuator kind that drives the wheels (rover-pi `KINDS`, minus the servo).
enum DriveKind: String, Codable, CaseIterable, Identifiable {
    case dcMotor = "dc_motor", bidirectionalESC = "bidirectional_esc", unidirectionalESC = "unidirectional_esc"
    var id: String { rawValue }
    var title: String {
        switch self {
        case .dcMotor: "DC motor"
        case .bidirectionalESC: "ESC, forward + reverse"
        case .unidirectionalESC: "ESC, forward only"
        }
    }
    var detail: String {
        switch self {
        case .dcMotor: "Brushed motor on the HAT's motor driver (ports M0–M3)"
        case .bidirectionalESC: "RC speed controller on a servo header (P0–P11)"
        case .unidirectionalESC: "Speed controller that can't reverse (P0–P11)"
        }
    }
    /// A one-way ESC can't be inverted (rover-pi rejects it).
    var invertible: Bool { self != .unidirectionalESC }
}

/// How many outputs drive the wheels.
enum Grouping: String, Codable, CaseIterable, Identifiable {
    case perWheel, shared
    var id: String { rawValue }
}

/// One output to wire: a wheel, a side, the rear drive or the steering servo.
struct Output: Identifiable, Hashable {
    enum Role: Hashable { case left, right, drive, steering, dumpGate }
    let id: String          // stable key, e.g. "L0", "R", "drive", "steer"
    let name: String        // shown in the app and stored on the rover
    let role: Role
    let kind: String        // rover-pi actuator kind
    /// Wheels this output turns, as diagram positions (see `VehicleSetup.wheels`).
    let wheels: [String]
}

/// A port the rover advertises in its `capabilities` reply.
struct RoverPort: Identifiable, Hashable {
    let name: String
    let kinds: [String]
    let resources: [String]
    var id: String { name }
}

struct PortAssignment: Codable, Hashable {
    var port: String?
    var inverted = false
}

struct VehicleSetup: Codable, Equatable {
    /// Fleet rover ID for `[comms] rover_id` (Zenoh key prefix `terra/<id>/...`).
    var roverId = 0
    var steering: Steering = .differential
    var wheelCount = 4
    var driveKind: DriveKind = .dcMotor
    var grouping: Grouping = .shared
    var assignments: [String: PortAssignment] = [:]
    // Geometry and limits (terra-av `[vehicle]`).
    var trackWidth = 0.5        // m, between left and right wheel centres
    var wheelbase = 0.4         // m, front to rear axle (Ackermann)
    var maxWheelSpeed = 0.8     // m/s
    // Steering servo pulses (µs).
    var servoMinUs = 1000, servoCenterUs = 1500, servoMaxUs = 2000
    // Optional dump gate positional servo (Fusion HAT servo header P0–P11).
    var dumpGatePort: String? = nil
    var dumpGateOpen = 1.0       // normalized servo position in [-1, 1]
    var dumpGateClosed = -1.0    // normalized servo position in [-1, 1]
    var dumpGateMinUs = 1000, dumpGateCenterUs = 1500, dumpGateMaxUs = 2000
    // Body and phone mount (m), in the rover's body frame: origin at the
    // rover's centre on the ground, +X forward, +Z up (terra-av `[mount]`).
    var bodyLength = 0.6
    var bodyWidth = 0.45
    var phoneForward = 0.25      // phone ahead of the centre
    var phoneHeight = 0.23       // phone above the ground
    // Phone rotation. Start: landscape, rear camera looking straight ahead
    // (ARKit's native camera orientation), screen facing the rover.
    var phoneTurn = 0            // degrees about the lens axis: 0, 90, 180, 270
    var phoneTilt = 0.0          // degrees, + = camera looks down
    var phonePan = 0.0           // degrees, + = camera looks left

    init() {}

    /// Missing keys (setups saved by an older app) take the defaults.
    init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        let d = VehicleSetup()
        roverId = try c.decodeIfPresent(Int.self, forKey: .roverId) ?? d.roverId
        steering = try c.decodeIfPresent(Steering.self, forKey: .steering) ?? d.steering
        wheelCount = try c.decodeIfPresent(Int.self, forKey: .wheelCount) ?? d.wheelCount
        driveKind = try c.decodeIfPresent(DriveKind.self, forKey: .driveKind) ?? d.driveKind
        grouping = try c.decodeIfPresent(Grouping.self, forKey: .grouping) ?? d.grouping
        assignments = try c.decodeIfPresent([String: PortAssignment].self, forKey: .assignments) ?? d.assignments
        trackWidth = try c.decodeIfPresent(Double.self, forKey: .trackWidth) ?? d.trackWidth
        wheelbase = try c.decodeIfPresent(Double.self, forKey: .wheelbase) ?? d.wheelbase
        maxWheelSpeed = try c.decodeIfPresent(Double.self, forKey: .maxWheelSpeed) ?? d.maxWheelSpeed
        servoMinUs = try c.decodeIfPresent(Int.self, forKey: .servoMinUs) ?? d.servoMinUs
        servoCenterUs = try c.decodeIfPresent(Int.self, forKey: .servoCenterUs) ?? d.servoCenterUs
        servoMaxUs = try c.decodeIfPresent(Int.self, forKey: .servoMaxUs) ?? d.servoMaxUs
        dumpGatePort = try c.decodeIfPresent(String.self, forKey: .dumpGatePort)
        dumpGateOpen = try c.decodeIfPresent(Double.self, forKey: .dumpGateOpen) ?? d.dumpGateOpen
        dumpGateClosed = try c.decodeIfPresent(Double.self, forKey: .dumpGateClosed) ?? d.dumpGateClosed
        dumpGateMinUs = try c.decodeIfPresent(Int.self, forKey: .dumpGateMinUs) ?? d.dumpGateMinUs
        dumpGateCenterUs = try c.decodeIfPresent(Int.self, forKey: .dumpGateCenterUs) ?? d.dumpGateCenterUs
        dumpGateMaxUs = try c.decodeIfPresent(Int.self, forKey: .dumpGateMaxUs) ?? d.dumpGateMaxUs
        bodyLength = try c.decodeIfPresent(Double.self, forKey: .bodyLength) ?? d.bodyLength
        bodyWidth = try c.decodeIfPresent(Double.self, forKey: .bodyWidth) ?? d.bodyWidth
        phoneForward = try c.decodeIfPresent(Double.self, forKey: .phoneForward) ?? d.phoneForward
        phoneHeight = try c.decodeIfPresent(Double.self, forKey: .phoneHeight) ?? d.phoneHeight
        phoneTurn = try c.decodeIfPresent(Int.self, forKey: .phoneTurn) ?? d.phoneTurn
        phoneTilt = try c.decodeIfPresent(Double.self, forKey: .phoneTilt) ?? d.phoneTilt
        phonePan = try c.decodeIfPresent(Double.self, forKey: .phonePan) ?? d.phonePan
    }

    // MARK: Shape

    /// Wheel positions, front to rear: "L0", "R0", "L1", ...
    var wheels: [String] {
        (0..<max(1, wheelCount / 2)).flatMap { ["L\($0)", "R\($0)"] }
    }

    var axleCount: Int { max(1, wheelCount / 2) }

    /// The outputs to wire, in id order.
    var outputs: [Output] {
        let kind = driveKind.rawValue
        var list: [Output]
        switch (steering, grouping) {
        case (.differential, .perWheel) where wheelCount > 2:
            list = wheels.map { w in
                let left = w.hasPrefix("L")
                return Output(id: w, name: "\(position(w)) \(left ? "left" : "right")", role: left ? .left : .right,
                              kind: kind, wheels: [w])
            }
        case (.differential, _):
            let l = wheels.filter { $0.hasPrefix("L") }, r = wheels.filter { $0.hasPrefix("R") }
            list = [Output(id: "L", name: "Left side", role: .left, kind: kind, wheels: l),
                    Output(id: "R", name: "Right side", role: .right, kind: kind, wheels: r)]
        case (.ackermann, .perWheel):
            list = [Output(id: "RL", name: "Rear left", role: .drive, kind: kind, wheels: ["L1"]),
                    Output(id: "RR", name: "Rear right", role: .drive, kind: kind, wheels: ["R1"]),
                    steeringOutput]
        case (.ackermann, .shared):
            list = [Output(id: "drive", name: "Rear drive", role: .drive, kind: kind, wheels: ["L1", "R1"]),
                    steeringOutput]
        }
        if dumpGatePort != nil {
            list.append(dumpGateOutput)
        }
        return list
    }

    /// Actuator ID of the dump gate servo in `actuators()`, or `nil` if unconfigured.
    var dumpGateActuatorId: Int? {
        outputs.firstIndex { $0.role == .dumpGate }
    }

    var dumpGateOutput: Output {
        Output(id: "dump_gate", name: "Dump gate", role: .dumpGate, kind: "positional_servo", wheels: [])
    }

    private var steeringOutput: Output {
        Output(id: "steer", name: "Steering", role: .steering, kind: "positional_servo", wheels: ["L0", "R0"])
    }

    private func position(_ wheel: String) -> String {
        let axle = Int(wheel.dropFirst()) ?? 0
        if axleCount == 1 { return "" }
        if axle == 0 { return "Front" }
        if axle == axleCount - 1 { return "Rear" }
        return "Middle"
    }

    /// Keep the choices consistent after the type or wheel count changes.
    mutating func normalize() {
        if !steering.wheelCounts.contains(wheelCount) { wheelCount = steering.wheelCounts.first ?? 4 }
        if steering == .differential && wheelCount == 2 { grouping = .shared }
        let ids = Set(outputs.map(\.id))
        assignments = assignments.filter { ids.contains($0.key) }
        if !driveKind.invertible {
            for key in assignments.keys where key != "steer" { assignments[key]?.inverted = false }
        }
    }

    // MARK: Ports

    /// Resources taken by every output except `except`.
    private func takenResources(ports: [RoverPort], except: String?) -> Set<String> {
        var taken = Set<String>()
        for (key, a) in assignments where key != except {
            if let name = a.port, let port = ports.first(where: { $0.name == name }) { taken.formUnion(port.resources) }
        }
        if except != "dump_gate", let name = dumpGatePort, let port = ports.first(where: { $0.name == name }) {
            taken.formUnion(port.resources)
        }
        return taken
    }

    /// Ports that fit this output: right kind, and no shared pin with another
    /// output (e.g. M0 uses P10 and P11).
    func availablePorts(for output: Output, ports: [RoverPort], occupied: Set<String> = []) -> [RoverPort] {
        let taken = takenResources(ports: ports, except: output.id).union(occupied)
        return ports.filter { $0.kinds.contains(output.kind) && taken.isDisjoint(with: $0.resources) }
            .sorted { Self.portOrder($0.name) < Self.portOrder($1.name) }
    }

    /// Whether the rover has enough ports of the drive kind for one output per
    /// wheel (the Fusion HAT has four motor ports, so six DC motors can't).
    func perWheelFits(ports: [RoverPort]) -> Bool {
        var trial = self
        trial.grouping = .perWheel
        trial.assignments = [:]
        trial.normalize()
        trial.autoAssign(ports: ports)
        return trial.outputs.allSatisfy { trial.assignments[$0.id]?.port != nil }
    }

    /// Give every unwired output the first port that fits.
    mutating func autoAssign(ports: [RoverPort], occupied: Set<String> = []) {
        for output in outputs where assignments[output.id]?.port == nil {
            if let port = availablePorts(for: output, ports: ports, occupied: occupied).first {
                assignments[output.id, default: PortAssignment()].port = port.name
            }
        }
    }

    static func portOrder(_ name: String) -> (Int, Int) {
        (name.hasPrefix("M") ? 0 : 1, Int(name.dropFirst()) ?? 99)
    }

    /// Why the setup can't be applied yet; empty when it can.
    var problems: [String] {
        var out: [String] = []
        if roverId < 0 { out.append("Rover ID must be zero or positive") }
        for o in outputs where o.role != .dumpGate && assignments[o.id]?.port == nil { out.append("\(o.name) has no port") }
        let used = outputs.compactMap { $0.role == .dumpGate ? dumpGatePort : assignments[$0.id]?.port }
        if Set(used).count != used.count { out.append("Two outputs share a port") }
        if trackWidth <= 0 || maxWheelSpeed <= 0 { out.append("Size and speed must be positive") }
        if bodyLength <= 0 || bodyWidth <= 0 { out.append("Rover length and width must be positive") }
        if phoneHeight <= 0 { out.append("Phone height must be above the ground") }
        if steering == .ackermann && !(servoMinUs < servoCenterUs && servoCenterUs < servoMaxUs) {
            out.append("Steering pulses must be min < center < max")
        }
        if dumpGatePort != nil && !(dumpGateMinUs < dumpGateCenterUs && dumpGateCenterUs < dumpGateMaxUs) {
            out.append("Dump gate pulses must be min < center < max")
        }
        return out
    }

    // MARK: rover-pi layout

    /// Actuators for `stage_actuator`, ids 0..n in output order.
    func actuators() -> [[String: Any]] {
        outputs.enumerated().map { index, output in
            let a = assignments[output.id] ?? PortAssignment()
            let port = output.role == .dumpGate ? (dumpGatePort ?? "") : (a.port ?? "")
            var entry: [String: Any] = [
                "id": index, "name": output.name, "port": port, "kind": output.kind,
                "inverted": output.kind == DriveKind.unidirectionalESC.rawValue ? false : a.inverted,
            ]
            switch output.kind {
            case "dc_motor":
                entry["calibration"] = ["type": "dc_motor", "max_power_fraction": 1.0]
                entry["limits"] = ["min": -1.0, "max": 1.0]
            case "bidirectional_esc":
                entry["calibration"] = ["type": "bidirectional_esc", "reverse_us": 1000, "neutral_us": 1500, "forward_us": 2000]
                entry["limits"] = ["min": -1.0, "max": 1.0]
            case "unidirectional_esc":
                entry["calibration"] = ["type": "unidirectional_esc", "stop_us": 1000, "full_power_us": 2000]
                entry["limits"] = ["min": 0.0, "max": 1.0]
            default: // positional_servo
                let (lo, mid, hi) = output.role == .dumpGate
                    ? (dumpGateMinUs, dumpGateCenterUs, dumpGateMaxUs)
                    : (servoMinUs, servoCenterUs, servoMaxUs)
                entry["calibration"] = ["type": "positional_servo", "min_us": lo,
                                        "center_us": mid, "max_us": hi]
                entry["limits"] = ["min": -1.0, "max": 1.0]
            }
            switch output.role {
            case .left: entry["route"] = ["type": "left_effort"]
            case .right: entry["route"] = ["type": "right_effort"]
            case .drive: entry["route"] = ["type": "manual", "forward_coefficient": 1.0, "turn_coefficient": 0.0]
            case .steering, .dumpGate: entry["route"] = ["type": "servo"]
            }
            switch output.role {
            case .steering: entry["safe"] = ["type": "position", "value": 0.0]
            case .dumpGate: entry["safe"] = ["type": "position", "value": min(1.0, max(-1.0, dumpGateClosed))]
            default: entry["safe"] = ["type": "zero"]
            }
            return entry
        }
    }

    // MARK: terra-av

    /// The `[vehicle]` table for terra-av.toml. terra-av only has a
    /// differential-drive model so far, so even Ackermann rovers pass their
    /// real track, speed, and body `length × width` rather than falling back
    /// to the bundled defaults.
    var terraAvVehicle: String {
        """
        [vehicle]
        type = "diff_drive"
        track = \(trackWidth)
        max_wheel_speed = \(maxWheelSpeed)
        length = \(bodyLength)
        width = \(bodyWidth)
        """
    }

    /// Radius of the circle around the rover's centre that holds the whole
    /// body: half its diagonal (m). terra-av's planners keep obstacles
    /// outside this circle.
    var bodyRadius: Double {
        let halfDiagonal = (bodyLength * bodyLength + bodyWidth * bodyWidth).squareRoot() / 2
        return (halfDiagonal * 1000).rounded() / 1000
    }

    /// The phone's position in the body frame, for `[mount] offset`.
    var mountOffset: String { "offset = [\(phoneForward), 0.0, \(phoneHeight)]" }

    /// ARKit camera axes (+X right, +Y up, -Z out of the lens) → rover body
    /// axes (+X forward, +Y left, +Z up). Columns are the camera axes in the
    /// body frame. Start (no turn, tilt or pan): lens looks along +X, camera
    /// right is the rover's right (-Y), camera up is up (+Z).
    var cameraToBody: simd_double3x3 {
        func rad(_ degrees: Double) -> Double { degrees * .pi / 180 }
        func rotZ(_ a: Double) -> simd_double3x3 {
            simd_double3x3(rows: [SIMD3(cos(a), -sin(a), 0), SIMD3(sin(a), cos(a), 0), SIMD3(0, 0, 1)])
        }
        func rotY(_ a: Double) -> simd_double3x3 {
            simd_double3x3(rows: [SIMD3(cos(a), 0, sin(a)), SIMD3(0, 1, 0), SIMD3(-sin(a), 0, cos(a))])
        }
        let start = simd_double3x3(rows: [SIMD3(0, 0, -1), SIMD3(-1, 0, 0), SIMD3(0, 1, 0)])
        // Pan about body Z, tilt about body Y (+ looks down), turn about the lens axis.
        return rotZ(rad(phonePan)) * rotY(rad(phoneTilt)) * start * rotZ(rad(Double(phoneTurn)))
    }

    /// `cameraToBody` as terra-av's Z-Y-X Euler angles in degrees:
    /// R = Rz(yaw) · Ry(pitch) · Rx(roll) (geometry.rs `from_euler`).
    var mountEuler: (roll: Double, pitch: Double, yaw: Double) {
        let m = cameraToBody
        func r(_ row: Int, _ col: Int) -> Double { m[col][row] } // simd is column-major
        let pitch = asin(max(-1, min(1, -r(2, 0))))
        let roll: Double, yaw: Double
        if abs(cos(pitch)) > 1e-6 {
            roll = atan2(r(2, 1), r(2, 2)); yaw = atan2(r(1, 0), r(0, 0))
        } else { // gimbal lock: fold everything into roll
            roll = atan2(-r(1, 2), r(1, 1)); yaw = 0
        }
        func deg(_ x: Double) -> Double { (x * 180 / .pi * 1000).rounded() / 1000 + 0 }
        return (deg(roll), deg(pitch), deg(yaw))
    }

    /// `toml` with its `[vehicle]` table replaced by this setup's (when there
    /// is one) and its `[mount]` set to the phone's position and rotation.
    func applying(to toml: String) -> String {
        var out: [String] = [], table = "", replacedVehicle = false
        for line in toml.components(separatedBy: "\n") {
            let trimmed = line.trimmingCharacters(in: .whitespaces)
            if trimmed.hasPrefix("[") {
                table = trimmed
                if table == "[vehicle]" {
                    out.append(terraAvVehicle); replacedVehicle = true; continue
                }
            }
            if table == "[vehicle]" && replacedVehicle { continue }
            if table == "[mount]" {
                let e = mountEuler
                if trimmed.hasPrefix("offset") { out.append(mountOffset); continue }
                if trimmed.hasPrefix("roll") { out.append("roll = \(e.roll)"); continue }
                if trimmed.hasPrefix("pitch") { out.append("pitch = \(e.pitch)"); continue }
                if trimmed.hasPrefix("yaw") { out.append("yaw = \(e.yaw)"); continue }
            }
            out.append(line)
        }
        if !replacedVehicle { out.insert(terraAvVehicle, at: 0) }
        return out.joined(separator: "\n")
    }

    // MARK: Pi serialization (stored on the Raspberry Pi, never on the phone)

    func jsonObject() -> [String: Any]? {
        guard let data = try? JSONEncoder().encode(self),
              let object = try? JSONSerialization.jsonObject(with: data) as? [String: Any] else { return nil }
        return object
    }

    static func from(jsonObject: Any) -> VehicleSetup? {
        guard JSONSerialization.isValidJSONObject(jsonObject),
              let data = try? JSONSerialization.data(withJSONObject: jsonObject) else { return nil }
        return try? JSONDecoder().decode(VehicleSetup.self, from: data)
    }
}
