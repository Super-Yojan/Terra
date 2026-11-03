import Foundation

/// Structured operational events beside OSLog. The sink only admits to a bounded queue.
final class TerraObservation: @unchecked Sendable {
    private let lock = NSLock()
    private var sink: ((String) -> Bool)?
    private var roverID = "_producer"
    private var selectedRover: String?
    private var selectedSession: String?
    private var sessionID = UUID().uuidString
    private var previous: [String: Data] = [:]
    private var pending: [String] = []
    private var pendingBytes = 0
    private(set) var rejected: UInt64 = 0
    func attach(roverID: String, sessionID: String, sink: @escaping (String) -> Bool) {
        lock.lock(); defer { lock.unlock() }
        self.roverID = selectedRover ?? roverID; self.sessionID = selectedSession ?? sessionID; self.sink = sink
        for raw in pending { if !sink(raw) { rejected += 1 } }
        pending.removeAll(); pendingBytes = 0
        previous.removeAll()
    }
    func setContext(roverID: String, sessionID: String? = nil) {
        lock.lock(); defer { lock.unlock() }
        selectedRover = roverID
        if let sessionID { selectedSession = sessionID }
        guard self.roverID != roverID || (sessionID != nil && self.sessionID != sessionID) else { return }
        self.roverID = roverID
        if let sessionID { self.sessionID = sessionID }
        previous.removeAll()
    }
    func record(component: String, kind: String, attributes: [String: Any]) {
        emit(component: component, kind: kind, attributes: attributes, changedOnly: false)
    }
    func recordChanged(component: String, kind: String, attributes: [String: Any]) {
        emit(component: component, kind: kind, attributes: attributes, changedOnly: true)
    }
    func recordSafetyRequest(reset: Bool, source: String, route: String) {
        record(component: "control", kind: "safety.requested", attributes: [
            "action": reset ? "reset_stop" : "emergency_stop",
            "source": source, "route": route])
    }
    func recordHardwareStatus(_ status: [String: Any]) {
        // Only diagnostic state; never copy ownership material or arbitrary payloads.
        let fields = ["connected", "active_revision", "control_mode", "output_active",
            "stop_reason", "fault", "status_subscribed",
            "layout_available"]
        var attributes: [String: Any] = [:]
        for field in fields { attributes[field] = status[field] }
        recordChanged(component: "bluetooth", kind: "hardware.status", attributes: attributes)
    }
    private func emit(component: String, kind: String, attributes: [String: Any], changedOnly: Bool) {
        guard JSONSerialization.isValidJSONObject(attributes),
              let encoded = try? JSONSerialization.data(withJSONObject: attributes, options: [.sortedKeys]) else { return }
        lock.lock()
        let key = component + "/" + kind
        if changedOnly && previous[key] == encoded { lock.unlock(); return }
        previous[key] = encoded
        let event: [String: Any] = ["rover_id": roverID, "session_id": sessionID,
            "component": component, "kind": kind, "severity": "info",
            "wall_time_ns": UInt64(Date().timeIntervalSince1970 * 1e9),
            "monotonic_ns": UInt64(ProcessInfo.processInfo.systemUptime * 1e9),
            "cause_id": NSNull(), "attributes": attributes]
        guard let data = try? JSONSerialization.data(withJSONObject: event), let raw = String(data: data, encoding: .utf8) else { lock.unlock(); return }
        // Keep early Bluetooth failures until the independent startup journal attaches.
        guard let sink else {
            if pending.count < 128 && pendingBytes + data.count <= 1024 * 1024 {
                pending.append(raw); pendingBytes += data.count
            } else { rejected += 1 }
            lock.unlock(); return
        }
        lock.unlock()
        if !sink(raw) { lock.lock(); rejected += 1; lock.unlock() }
    }
}
