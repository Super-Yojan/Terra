import Foundation

@main struct FpvSignalingTests {
    static func main() {
        var iceSettings = FpvIceSettings(stunURL: "", turnURL: "", turnUsername: "", turnCredential: "super-secret-turn")
        precondition(iceSettings.iceServers() == [FpvIceServer(urls: [FpvIceSettings.defaultStunURL], username: "", credential: "")])
        precondition(!iceSettings.redactedSummary().contains("super-secret-turn"))
        iceSettings.turnURL = "turn:turn.example.com:3478"
        iceSettings.turnUsername = "terra"
        let servers = iceSettings.iceServers()
        precondition(servers.count == 2)
        precondition(servers[1].urls == ["turn:turn.example.com:3478"])
        precondition(servers[1].credential == "super-secret-turn")
        precondition(!iceSettings.redactedSummary().contains("super-secret-turn"))
        precondition(!iceSettings.redactedSummary().contains("terra"))
        iceSettings.turnURL = "http://turn.example.com"
        precondition(iceSettings.iceServers().count == 1)
        iceSettings.stunURL = "stun:stun.example.com:19302"
        precondition(iceSettings.iceServers()[0].urls == ["stun:stun.example.com:19302"])
        print("FPV ICE redaction checks passed")
    }
}
