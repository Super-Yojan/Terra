import Foundation

/// Structured operational events beside OSLog. The sink only admits to a bounded queue.
final class TerraObservation: @unchecked Sendable {
    private let lock = NSLock()
    private var sink: ((String) -> Bool)?
    private var roverID = "_producer"
    private var sessionID = UUID().uuidString
    private var previous: [String: Data] = [:]
    private(set) var rejected: UInt64 = 0
    func attach(roverID: String, sessionID: String, sink: @escaping (String) -> Bool) {
        lock.lock(); defer { lock.unlock() }
        self.roverID = roverID; self.sessionID = sessionID; self.sink = sink
        previous.removeAll()
    }
    func setContext(roverID: String, sessionID: String? = nil) {
        lock.lock(); defer { lock.unlock() }
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
    private func emit(component: String, kind: String, attributes: [String: Any], changedOnly: Bool) {
        guard JSONSerialization.isValidJSONObject(attributes),
              let encoded = try? JSONSerialization.data(withJSONObject: attributes, options: [.sortedKeys]) else { return }
        lock.lock()
        let key = component + "/" + kind
        if changedOnly && previous[key] == encoded { lock.unlock(); return }
        previous[key] = encoded
        let sink = self.sink
        let event: [String: Any] = ["rover_id": roverID, "session_id": sessionID,
            "component": component, "kind": kind, "severity": "info",
            "wall_time_ns": UInt64(Date().timeIntervalSince1970 * 1e9),
            "monotonic_ns": UInt64(ProcessInfo.processInfo.systemUptime * 1e9),
            "cause_id": NSNull(), "attributes": attributes]
        lock.unlock()
        guard let data = try? JSONSerialization.data(withJSONObject: event), let raw = String(data: data, encoding: .utf8) else { return }
        if sink?(raw) != true { lock.lock(); rejected += 1; lock.unlock() }
    }
}
