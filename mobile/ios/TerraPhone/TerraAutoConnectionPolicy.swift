import Foundation

/// Discovery never establishes ownership. Only a remembered rover reconnects silently.
enum TerraAutoConnectionPolicy {
    static func select(candidates: [UUID], preferred: UUID?) -> UUID? {
        guard let preferred, candidates.contains(preferred) else { return nil }
        return preferred
    }
}

struct TerraDiscoveredRover: Identifiable, Equatable {
    let identifier: UUID
    let name: String
    let pairing: Bool
    var id: UUID { identifier }
}

/// Main-queue policy; adapters provide monotonic time and perform radio operations.
struct TerraConnectionPolicy {
    enum Decision: Equatable { case offer(UUID), connect(UUID) }
    private(set) var generation: UInt64 = 0
    private(set) var enabled = false
    private(set) var preferred: UUID?
    private(set) var connected: UUID?
    private(set) var attempting: UUID?
    private(set) var offered: UUID?
    private var declined: Set<UUID> = []
    private var deadline: TimeInterval?
    private var retryAt: TimeInterval?
    private var bleRetries = 0
    private var routerRetries = 0
    private var awaitingOperation = false
    private static let delays: [TimeInterval] = [1, 2, 4, 8, 15]
    mutating func begin(preferred: UUID?) {
        stop(); enabled = true; self.preferred = preferred; declined.removeAll()
        bleRetries = 0; routerRetries = 0
    }
    mutating func adoptAuthenticated(_ id: UUID) -> Bool {
        guard enabled, preferred == id else { return false }
        authenticated(id); return true
    }
    mutating func stop() {
        generation &+= 1; enabled = false; connected = nil; attempting = nil
        offered = nil; deadline = nil; retryAt = nil; awaitingOperation = false
    }
    mutating func discover(_ rover: TerraDiscoveredRover) -> Decision? {
        guard enabled, connected == nil, attempting == nil, retryAt == nil else { return nil }
        if rover.identifier == preferred, !rover.pairing {
            offered = nil; attempting = rover.identifier
            return .connect(rover.identifier)
        }
        guard !awaitingOperation, rover.pairing, !declined.contains(rover.identifier), offered == nil else { return nil }
        offered = rover.identifier; return .offer(rover.identifier)
    }
    mutating func decline(_ id: UUID) { declined.insert(id); if offered == id { offered = nil } }
    mutating func accept(_ id: UUID, now: TimeInterval) {
        offered = nil; declined.insert(id); attempting = id; deadline = now + 15
    }
    mutating func paired(_ id: UUID, now: TimeInterval) {
        preferred = id; attempting = nil; connected = nil; awaitingOperation = true
        deadline = now + 30; retryAt = nil
    }
    mutating func authenticated(_ id: UUID) {
        preferred = id; connected = id; attempting = nil; deadline = nil
        retryAt = nil; awaitingOperation = false; bleRetries = 0
    }
    mutating func failed(retryable: Bool, now: TimeInterval) {
        generation &+= 1; connected = nil; attempting = nil; offered = nil; deadline = nil
        if enabled && retryable {
            retryAt = now + Self.delays[min(bleRetries, 4)]; bleRetries += 1
        } else { retryAt = nil; enabled = false }
    }
    mutating func retryDue(now: TimeInterval) -> Bool {
        guard enabled, let retryAt, now >= retryAt else { return false }
        self.retryAt = nil; return true
    }
    func attemptExpired(now: TimeInterval) -> Bool { enabled && deadline.map { now >= $0 } == true }
    mutating func nextRouterDelay() -> TimeInterval {
        let delay = Self.delays[min(routerRetries, 4)]; routerRetries += 1; return delay
    }
    mutating func routerConnected() { routerRetries = 0 }
}

/// Keep discovery order, but never replay an expired or superseded pairing offer.
struct TerraPairingCandidates {
    private var entries: [(rover: TerraDiscoveredRover, seen: TimeInterval)] = []
    mutating func observe(_ rover: TerraDiscoveredRover, now: TimeInterval) {
        entries.removeAll { now - $0.seen >= 10 }
        if !rover.pairing { entries.removeAll { $0.rover.id == rover.id }; return }
        if let index = entries.firstIndex(where: { $0.rover.id == rover.id }) {
            entries[index] = (rover, now)
        } else { entries.append((rover, now)) }
    }
    func available(now: TimeInterval) -> [TerraDiscoveredRover] {
        entries.filter { now >= $0.seen && now - $0.seen < 10 }.map { $0.rover }
    }
    mutating func removeAll() { entries.removeAll() }
}

// Radio availability waits for central state changes; protocol/authentication failures stop retrying.
enum TerraBluetoothFailurePolicy {
    static func retryable(_ reason: String) -> Bool {
        !["Malformed", "Encrypted", "Invalid", "Terra service unavailable", "Status unavailable"]
            .contains(where: { reason.contains($0) })
    }
}

struct TerraRouterSettings {
    let endpoint: String
    let prefix: String
    let roverID: UInt64
    static func parse(endpoint: String, prefix: String, roverID: String) -> Self? {
        let endpoint = endpoint.trimmingCharacters(in: .whitespacesAndNewlines)
        let prefix = prefix.trimmingCharacters(in: .whitespacesAndNewlines)
        guard let id = UInt64(roverID.trimmingCharacters(in: .whitespacesAndNewlines)),
              endpoint.hasPrefix("tcp/"), !endpoint.contains(where: { $0.isWhitespace }),
              let separator = endpoint.lastIndex(of: ":"),
              let port = UInt16(endpoint[endpoint.index(after: separator)...]), port > 0,
              !prefix.isEmpty, !prefix.contains("*"), !prefix.contains("#"),
              !prefix.hasPrefix("/"), !prefix.hasSuffix("/"), !prefix.contains("//") else { return nil }
        let host = String(endpoint[endpoint.index(endpoint.startIndex, offsetBy: 4)..<separator])
        guard !host.isEmpty, !host.contains("/") else { return nil }
        if host.contains(":") {
            guard host.hasPrefix("["), host.hasSuffix("]"), host.count > 2 else { return nil }
        } else if host.contains("[") || host.contains("]") { return nil }
        return .init(endpoint: endpoint, prefix: prefix, roverID: id)
    }
}

/// Sensor-free manual driving must propagate a remote stop, not sensor-related holds.
enum TerraManualRouterSafety {
    static func emergencyStopped(statusJSON: String) -> Bool {
        guard let object = (try? JSONSerialization.jsonObject(with: Data(statusJSON.utf8))) as? [String: Any],
              let status = object["status"] as? [String: Any] else { return false }
        return status["safety"] as? String == "emergency_stop"
    }
}
