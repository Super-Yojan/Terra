import Foundation

// Confined to BluetoothLink's serial queue. Status describes the streamed relay.
final class BluetoothSession {
    private(set) var generation: UInt64 = 0
    private(set) var connected = false
    private(set) var revision: UInt32 = 0
    private(set) var ready = false
    private(set) var outputActive = false
    private var layoutAvailable = false
    private var faulted = false
    private var profile: ActuatorDriveProfileSet?

    func reset() {
        generation &+= 1; connected = false; revision = 0
        ready = false; outputActive = false; layoutAvailable = false; faulted = false; profile = nil
    }
    func invalidateLayout() { ready = false; profile = nil }
    func synchronized(profile: ActuatorDriveProfileSet) {
        self.profile = profile
        refreshReady()
    }
    private func refreshReady() {
        ready = connected && layoutAvailable && !faulted && profile?.revision == revision && profile?.actuators.isEmpty == false
    }
    func status(_ value: [String: Any]) -> String? {
        guard value["type"] as? String == "status", value["schema_version"] as? Int == 1,
              value["control_mode"] as? String == "streamed_relay",
              let connected = value["connected"] as? Bool,
              let revision = value["active_revision"] as? UInt32,
              let available = value["layout_available"] as? Bool,
              let active = value["output_active"] as? Bool else { return nil }
        if revision != self.revision { invalidateLayout() }
        self.connected = connected; self.revision = revision; layoutAvailable = available
        faulted = value["fault"] is String; outputActive = active
        refreshReady()
        if let fault = value["fault"] as? String { return "Fault: \(fault)" }
        return connected ? (active ? "Streaming motor commands" : "Connected · neutral output") : "Waiting for relay connection"
    }
    func encodeDrive(valuesJSON: String) throws -> Data {
        struct Value: Decodable { let id: UInt8; let value: Double }
        guard ready, let profile else { throw BluetoothPolicyError.unavailable }
        let values = try JSONDecoder().decode([Value].self, from: Data(valuesJSON.utf8))
        guard values.count == profile.actuators.count, Set(values.map(\.id)).count == values.count else { throw BluetoothPolicyError.malformed }
        var bytes = Data()
        for record in values {
            guard record.value.isFinite, (-1...1).contains(record.value),
                  let actuator = profile.actuators.first(where: { $0.id == Int(record.id) }),
                  record.value >= actuator.limits.min, record.value <= actuator.limits.max else { throw BluetoothPolicyError.malformed }
            var effort = Int((record.value * 100).rounded())
            let lower = Float(actuator.limits.min), upper = Float(actuator.limits.max)
            while effort > -100 && Float(Double(effort) / 100) > upper { effort -= 1 }
            while effort < 100 && Float(Double(effort) / 100) < lower { effort += 1 }
            guard (-100...100).contains(effort), Float(Double(effort) / 100) >= lower,
                  Float(Double(effort) / 100) <= upper else { throw BluetoothPolicyError.malformed }
            bytes.append(record.id); bytes.append(UInt8(bitPattern: Int8(effort)))
        }
        return bytes
    }
}
