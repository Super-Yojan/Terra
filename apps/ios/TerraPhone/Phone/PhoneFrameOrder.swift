import Foundation

/// AR callbacks do not renew VIO freshness unless they carry a strictly newer sample.
/// Only the control queue owns this gate; the estimator's strict checks remain intact.
struct PhoneFrameOrder {
    private var sessionStart: Double?
    private(set) var lastTimestamp: Double?
    mutating func accept(_ timestamp: Double, notBefore start: Double) -> Bool {
        guard start.isFinite, timestamp.isFinite, timestamp >= start else { return false }
        if sessionStart != start { sessionStart = start; lastTimestamp = nil }
        guard lastTimestamp.map({ timestamp > $0 }) ?? true else { return false }
        lastTimestamp = timestamp
        return true
    }
}
