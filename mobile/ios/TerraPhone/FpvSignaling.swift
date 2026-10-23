import CoreFoundation
import Foundation

/// Dashboard-to-phone signaling after the topic suffix and JSON have been checked.
enum FpvInbound: Equatable {
    case start(session: String)
    case stop(session: String)
    case answer(session: String, sdp: String)
    case ice(session: String, candidate: String, sdpMid: String, sdpMLineIndex: Int)
}

/// What the phone should do with one accepted message.
/// A duplicate start for the live session is `ignore`. A different session replaces it.
enum FpvDecision: Equatable {
    case ignore
    case start(previous: String?, session: String)
    case stop(session: String)
    case answer(session: String, sdp: String)
    case ice(session: String, candidate: String, sdpMid: String, sdpMLineIndex: Int)
    case endOfCandidates(session: String)
}

/// One FPV session per phone. Stale session ids are ignored.
/// The machine records video signaling only. It has no arm, disarm, or velocity command.
struct FpvSignaling {
    /// Keep equal to `WEBRTC_SIGNALING_MAX_BYTES` in terra-transport.
    static let maxBytes = 65_536
    private var activeSession: String?
    private var activeID: UUID?
    private var failed = false

    var hasSession: Bool { activeSession != nil }
    func isCurrent(_ session: String) -> Bool { matches(session) }

    /// Decode a fleet sample. Unknown topics, versions, sessions, and extra-large
    /// bodies produce nil and leave the active session unchanged.
    /// Unknown JSON fields are ignored so the dashboard can add optional diagnostics.
    static func decode(kind: String, payload: String) -> FpvInbound? {
        guard Data(payload.utf8).count <= maxBytes,
              let data = payload.data(using: .utf8),
              let object = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
              let session = uuidString(object["session"]),
              isVersionOne(object["v"]) else { return nil }
        switch kind {
        case "webrtc/request":
            switch object["action"] as? String {
            case "start": return .start(session: session)
            case "stop": return .stop(session: session)
            default: return nil
            }
        case "webrtc/answer":
            guard let sdp = object["sdp"] as? String, !sdp.isEmpty else { return nil }
            return .answer(session: session, sdp: sdp)
        case "webrtc/ice/dashboard":
            guard let candidate = object["candidate"] as? String,
                  let index = lineIndex(object["sdpMLineIndex"]) else { return nil }
            let mid = object["sdpMid"] as? String ?? ""
            return .ice(session: session, candidate: candidate, sdpMid: mid, sdpMLineIndex: index)
        default:
            return nil
        }
    }

    static func offerJSON(session: String, sdp: String) -> String? {
        json(["v": 1, "session": session, "sdp": sdp])
    }

    static func iceJSON(session: String, candidate: String, sdpMid: String, sdpMLineIndex: Int) -> String? {
        json(["v": 1, "session": session, "candidate": candidate, "sdpMid": sdpMid, "sdpMLineIndex": sdpMLineIndex])
    }

    static func stateJSON(session: String, state: String) -> String? {
        guard ["connecting", "connected", "failed", "closed"].contains(state) else { return nil }
        return json(["v": 1, "session": session, "state": state])
    }

    mutating func receive(_ message: FpvInbound) -> FpvDecision {
        switch message {
        case .start(let session):
            guard let id = UUID(uuidString: session) else { return .ignore }
            if activeID == id, !failed { return .ignore }
            let previous = activeSession
            activeSession = session
            activeID = id
            failed = false
            return .start(previous: previous, session: session)
        case .stop(let session):
            guard matches(session) else { return .ignore }
            let current = activeSession ?? session
            clear()
            return .stop(session: current)
        case .answer(let session, let sdp):
            guard matches(session), !failed else { return .ignore }
            return .answer(session: activeSession ?? session, sdp: sdp)
        case .ice(let session, let candidate, let sdpMid, let sdpMLineIndex):
            guard matches(session), !failed else { return .ignore }
            let current = activeSession ?? session
            if candidate.isEmpty { return .endOfCandidates(session: current) }
            return .ice(session: current, candidate: candidate, sdpMid: sdpMid, sdpMLineIndex: sdpMLineIndex)
        }
    }

    /// First failure keeps the session so a later stop still matches. Further failures are ignored.
    mutating func noteFailed() -> String? {
        guard let activeSession, !failed else { return nil }
        failed = true
        return activeSession
    }

    mutating func stopLocal() -> String? {
        let session = activeSession
        clear()
        return session
    }

    private mutating func clear() {
        activeSession = nil
        activeID = nil
        failed = false
    }

    private func matches(_ session: String) -> Bool {
        guard let activeID, let id = UUID(uuidString: session) else { return false }
        return activeID == id
    }

    private static func uuidString(_ value: Any?) -> String? {
        guard let text = value as? String, UUID(uuidString: text) != nil else { return nil }
        return text
    }

    private static func isVersionOne(_ value: Any?) -> Bool {
        guard let number = value as? NSNumber else { return false }
        if CFGetTypeID(number) == CFBooleanGetTypeID() { return false }
        return number.doubleValue == 1
    }

    private static func lineIndex(_ value: Any?) -> Int? {
        guard let number = value as? NSNumber else { return nil }
        let index = number.intValue
        guard number.doubleValue == Double(index), index >= 0, index <= Int(Int32.max) else { return nil }
        return index
    }

    private static func json(_ fields: [String: Any]) -> String? {
        guard JSONSerialization.isValidJSONObject(fields),
              let data = try? JSONSerialization.data(withJSONObject: fields),
              data.count <= maxBytes,
              let text = String(data: data, encoding: .utf8) else { return nil }
        return text
    }
}
