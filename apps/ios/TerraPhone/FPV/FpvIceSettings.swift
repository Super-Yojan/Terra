import Foundation

/// One ICE server entry. Credentials stay in memory for the peer connection and are not logged.
struct FpvIceServer: Equatable {
    var urls: [String]
    var username: String
    var credential: String
}

/// STUN and optional TURN for the phone's FPV peer connection.
/// The public STUN default is not a secret. TURN credentials are supplied by the caller.
struct FpvIceSettings: Equatable {
    static let defaultStunURL = "stun:stun.l.google.com:19302"
    static let stunDefaultsKey = "fpvStunURL"
    static let turnURLDefaultsKey = "fpvTurnURL"
    static let turnUsernameDefaultsKey = "fpvTurnUsername"
    /// Keychain account. The service is the same fleet keychain as the router password.
    static let turnCredentialAccount = "fpv-turn"

    var stunURL: String
    var turnURL: String
    var turnUsername: String
    var turnCredential: String

    func iceServers() -> [FpvIceServer] {
        var servers = [FpvIceServer(urls: [Self.normalizedStun(stunURL)], username: "", credential: "")]
        if let turn = Self.normalizedTurn(turnURL) {
            servers.append(FpvIceServer(urls: [turn], username: turnUsername, credential: turnCredential))
        }
        return servers
    }

    /// Host labels only. The TURN credential and username are omitted.
    func redactedSummary() -> String {
        let turn = Self.normalizedTurn(turnURL) == nil ? "off" : "configured"
        return "STUN \(Self.normalizedStun(stunURL)); TURN \(turn)"
    }

    static func normalizedStun(_ value: String) -> String {
        let trimmed = value.trimmingCharacters(in: .whitespacesAndNewlines)
        return valid(trimmed, schemes: ["stun:", "stuns:"]) ? trimmed : defaultStunURL
    }

    static func normalizedTurn(_ value: String) -> String? {
        let trimmed = value.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmed.isEmpty, valid(trimmed, schemes: ["turn:", "turns:"]) else { return nil }
        return trimmed
    }

    private static func valid(_ value: String, schemes: [String]) -> Bool {
        guard !value.contains(where: \.isWhitespace),
              let scheme = schemes.first(where: { value.lowercased().hasPrefix($0) }) else { return false }
        return !value.dropFirst(scheme.count).isEmpty
    }
}
