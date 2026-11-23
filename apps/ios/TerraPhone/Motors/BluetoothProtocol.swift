import Foundation

// The Terra rover Bluetooth protocol, matching apps/rover-pi/terra_rover
// (protocol.py, ble.py, configuration.py). Pure data, no CoreBluetooth.

enum BluetoothPolicyError: Error { case unavailable, malformed }

/// Split a JSON message into `<id u16 LE><index u8><count u8><chunk>` writes
/// (protocol.py `fragment_message`).
func fragmentMessage(_ data: Data, id: UInt16, writeSize: Int) throws -> [Data] {
    guard !data.isEmpty, writeSize > 4, data.count <= 16384 else { throw BluetoothPolicyError.malformed }
    let chunk = writeSize - 4
    let count = (data.count + chunk - 1) / chunk
    guard count <= 255 else { throw BluetoothPolicyError.malformed }
    return (0..<count).map { index in
        var out = Data([UInt8(id & 0xff), UInt8(id >> 8), UInt8(index), UInt8(count)])
        out.append(data[data.startIndex + index * chunk ..< data.startIndex + min((index + 1) * chunk, data.count)])
        return out
    }
}

/// Reassembles fragmented status and reply notifications.
struct FragmentAssembler {
    let timeout: TimeInterval
    let limit: Int
    init(timeout: TimeInterval = 0.1, limit: Int = 16384) { self.timeout = timeout; self.limit = limit }
    private var id: UInt16?
    private var count: UInt8 = 0
    private var next: UInt8 = 0
    private var started: TimeInterval = 0
    private var data = Data()
    mutating func clear() { id = nil; data.removeAll(); next = 0 }
    mutating func push(_ bytes: Data, now: TimeInterval) throws -> Data? {
        if id != nil && (now < started || now - started >= timeout) { clear() }
        let b = [UInt8](bytes)
        guard b.count >= 5, b[3] > 0, b[2] < b[3] else { clear(); throw BluetoothPolicyError.malformed }
        let incoming = UInt16(b[0]) | UInt16(b[1]) << 8
        if id == nil { guard b[2] == 0 else { throw BluetoothPolicyError.malformed }; id = incoming; count = b[3]; started = now }
        guard id == incoming, count == b[3], next == b[2], data.count + b.count - 4 <= limit else { clear(); throw BluetoothPolicyError.malformed }
        data.append(contentsOf: b[4...])
        if next == count - 1 { let result = data; clear(); return result }
        next += 1; return nil
    }
}

// MARK: Actuator profiles (read from the rover, never edited here)

struct ActuatorLimits: Codable { let min: Double; let max: Double }
struct ActuatorRoute: Codable { let type: String; let forward_coefficient: Double?; let turn_coefficient: Double? }
struct ActuatorSafe: Codable { let type: String; let value: Double? }

struct ActuatorDriveProfile: Codable {
    let id: Int
    let kind: String
    let limits: ActuatorLimits
    let route: ActuatorRoute
    let safe: ActuatorSafe
    var safeValue: Double { Swift.min(limits.max, Swift.max(limits.min, safe.value ?? 0)) }
}

struct ActuatorDriveProfileSet {
    let revision: UInt32
    let actuators: [ActuatorDriveProfile]

    /// Every actuator at its safe value.
    var neutral: [(id: Int, value: Double)] { actuators.map { ($0.id, $0.safeValue) } }

    /// Route a wheel command onto the rover's actuators: left/right motors
    /// take their side, mixed motors take forward ± turn, a steering servo
    /// (Ackermann) takes the turn, and any actuator id in `overrides` (e.g.
    /// the dump gate servo) takes its explicit target position.
    func values(for command: WheelCommand, overrides: [Int: Double] = [:]) -> [(id: Int, value: Double)] {
        let forward = (command.left + command.right) / 2, turn = (command.right - command.left) / 2
        return actuators.map { a in
            let raw: Double
            if let override = overrides[a.id] {
                raw = override
            } else {
                switch a.route.type {
                case "left_effort": raw = command.left
                case "right_effort": raw = command.right
                case "manual": raw = (a.route.forward_coefficient ?? 0) * forward + (a.route.turn_coefficient ?? 0) * turn
                case "servo": raw = turn
                default: raw = a.safeValue
                }
            }
            return (a.id, Swift.min(a.limits.max, Swift.max(a.limits.min, raw)))
        }
    }

    /// Drive characteristic payload: `<id u8><effort i8 percent>` per actuator.
    func encode(_ values: [(id: Int, value: Double)]) throws -> Data {
        var bytes = Data()
        for (id, value) in values {
            guard value.isFinite, let a = actuators.first(where: { $0.id == id }) else { throw BluetoothPolicyError.malformed }
            var effort = Int((value * 100).rounded())
            while effort > -100 && Double(effort) / 100 > a.limits.max { effort -= 1 }
            while effort < 100 && Double(effort) / 100 < a.limits.min { effort += 1 }
            bytes.append(UInt8(id)); bytes.append(UInt8(bitPattern: Int8(effort)))
        }
        return bytes
    }
}

/// Collects one layout revision's profiles, one read per actuator.
struct ActuatorProfileSync {
    private(set) var revision: UInt32?
    private(set) var remaining: [Int] = []
    private var profiles: [ActuatorDriveProfile] = []
    mutating func begin(revision: UInt32, ids: [Int]) { self.revision = revision; remaining = ids; profiles = [] }
    mutating func accept(revision: UInt32, actuator: ActuatorDriveProfile) throws {
        guard self.revision == revision, remaining.first == actuator.id else { throw BluetoothPolicyError.malformed }
        profiles.append(actuator); remaining.removeFirst()
    }
    var completed: ActuatorDriveProfileSet? {
        guard let revision, remaining.isEmpty else { return nil }
        return ActuatorDriveProfileSet(revision: revision, actuators: profiles)
    }
    mutating func clear() { self = Self() }
}
