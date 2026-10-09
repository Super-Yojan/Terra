import Foundation

// Confined to BluetoothLink's serial queue. Only remote status authorizes motion.
final class BluetoothSession {
    enum Effect { case send(Data), disarm, state(String) }
    private(set) var generation: UInt64 = 0
    private(set) var session: UInt32?
    private(set) var revision: UInt32 = 0
    private(set) var sequence: UInt32 = 0
    private(set) var armed = false
    private(set) var arming = false
    private(set) var ready = false
    private var layoutAvailable = false
    private var faulted = false
    private var statusAt: TimeInterval?
    private var armRequested = false
    var motionRequested: Bool { armRequested }
    func reset() {
        generation &+= 1; session = nil; revision = 0; sequence = 0
        armed = false; arming = false; ready = false; layoutAvailable = false; faulted = false; statusAt = nil; armRequested = false
    }
    func invalidateLayout() { ready = false; stop() }
    func synchronized(revision: UInt32) { ready = session != nil && layoutAvailable && !faulted && self.revision == revision }
    func status(_ value: [String: Any], now: TimeInterval) -> [Effect] {
        guard value["type"] as? String == "status", value["schema_version"] as? Int == 1,
              let revision = value["active_revision"] as? UInt32 else { return [] }
        let incoming = value["session"] as? UInt32
        if incoming != session || revision != self.revision {
            ready = false; armRequested = false; sequence = 0
        }
        session = incoming; self.revision = revision; statusAt = now
        layoutAvailable = value["layout_available"] as? Bool == true
        faulted = value["fault"] is String || value["emergency_stop"] as? Bool == true
        if !layoutAvailable || faulted { invalidateLayout() }
        // An unsolicited status cannot resurrect local motion permission.
        armed = armRequested && value["armed"] as? Bool == true
        arming = armRequested && value["arming"] as? Bool == true
        if let last = value["last_sequence"] as? UInt32 { sequence = max(sequence, last) }
        if value["fault"] is String || value["emergency_stop"] as? Bool == true {
            armRequested = false; armed = false; arming = false
        }
        if let fault = value["fault"] as? String { return [.state("Fault: \(fault) · disarmed")] }
        if value["emergency_stop"] as? Bool == true { return [.state("Emergency stop latched · disarmed")] }
        let disarmed = value["stop_reason"] as? String == "watchdog" ? "Disarmed · command stream stopped" : "Disarmed"
        return [.state(armed ? "Armed" : arming ? "Arming at safe output" : disarmed)]
    }
    func control(_ kind: String) throws -> Data {
        guard let session = session, sequence < UInt32.max - 1 else { throw BluetoothPolicyError.unavailable }
        sequence += 1
        let object: [String: Any] = ["kind": kind, "session": session, "revision": revision, "sequence": sequence, "values": []]
        let json = String(decoding: try JSONSerialization.data(withJSONObject: object), as: UTF8.self)
        return try actuatorEncodeFrame(frameJson: json)
    }
    func frame(values: Any) throws -> Data {
        guard ready, let session = session, sequence < UInt32.max - 1 else { throw BluetoothPolicyError.unavailable }
        let object: [String: Any] = ["kind": "drive", "session": session, "revision": revision, "sequence": sequence + 1, "values": values]
        let json = String(decoding: try JSONSerialization.data(withJSONObject: object), as: UTF8.self)
        return try actuatorEncodeFrame(frameJson: json)
    }
    func arm(now: TimeInterval) throws -> Data {
        guard ready, let statusAt = statusAt, now >= statusAt, now - statusAt < 0.3 else { throw BluetoothPolicyError.unavailable }
        let frame = try control("arm"); armRequested = true; return frame
    }
    func stop() { armRequested = false; armed = false; arming = false }
    func statusAge(now: TimeInterval) -> TimeInterval? {
        statusAt.map { now - $0 }
    }
    func stale(now: TimeInterval) -> Bool {
        guard let statusAt = statusAt else { return false }
        return now < statusAt || now - statusAt >= 0.3
    }
    func drive(_ frame: Data, producedAt: TimeInterval, now: TimeInterval) -> [Effect] {
        guard ready, now >= producedAt, now - producedAt < 0.1, !stale(now: now), frame.count >= 16,
              frame[0] == 84, frame[1] == 65, frame[2] == 1, frame[3] == 1 else { return [] }
        func word(_ offset: Int) -> UInt32 { (0..<4).reduce(0) { $0 | UInt32(frame[offset + $1]) << (8 * $1) } }
        guard word(4) == session, word(8) == revision, word(12) > sequence, word(12) < UInt32.max else { return [] }
        // Before/while arming, allow freshly produced safe frames; the Pi checks safe policy.
        sequence = word(12); return [.send(frame)]
    }
}
enum BluetoothPolicyError: Error { case unavailable, malformed }

struct BluetoothJSONAssembler {
    private var id: UInt16?
    private var count: UInt8 = 0
    private var next: UInt8 = 0
    private var started: TimeInterval = 0
    private var data = Data()
    mutating func clear() { id = nil; data.removeAll(); next = 0 }
    mutating func push(_ bytes: Data, now: TimeInterval) throws -> Data? {
        if id != nil && (now < started || now - started >= 0.1) { clear() }
        guard bytes.count >= 5, bytes[3] > 0, bytes[2] < bytes[3] else { clear(); throw BluetoothPolicyError.malformed }
        let incoming = UInt16(bytes[0]) | UInt16(bytes[1]) << 8
        if id == nil { guard bytes[2] == 0 else { throw BluetoothPolicyError.malformed }; id = incoming; count = bytes[3]; started = now }
        guard id == incoming, count == bytes[3], next == bytes[2], data.count + bytes.count - 4 <= 16384 else { clear(); throw BluetoothPolicyError.malformed }
        data.append(bytes.dropFirst(4))
        if next == count - 1 { let result = data; clear(); return result }
        next += 1; return nil
    }
}
