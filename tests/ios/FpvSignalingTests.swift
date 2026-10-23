import Foundation

@main struct FpvSignalingTests {
    static func main() {
        let first = "11111111-1111-4111-8111-111111111111"
        let second = "22222222-2222-4222-8222-222222222222"
        var machine = FpvSignaling()

        precondition(FpvSignaling.decode(kind: "hardware", payload: #"{"action":"arm","token":"x"}"#) == nil)
        precondition(FpvSignaling.decode(kind: "teleop", payload: #"{"linear":1,"angular":0}"#) == nil)
        precondition(FpvSignaling.decode(kind: "cmd_vel", payload: #"{"linear":1,"angular":0}"#) == nil)
        precondition(FpvSignaling.decode(kind: "webrtc/request", payload: #"{"v":1,"action":"arm","session":"\#(first)"}"#) == nil)
        precondition(FpvSignaling.decode(kind: "webrtc/request", payload: #"{"v":2,"action":"start","session":"\#(first)"}"#) == nil)
        precondition(FpvSignaling.decode(kind: "webrtc/request", payload: #"{"v":true,"action":"start","session":"\#(first)"}"#) == nil)
        precondition(FpvSignaling.decode(kind: "webrtc/request", payload: #"{"v":"1","action":"start","session":"\#(first)"}"#) == nil)
        precondition(FpvSignaling.decode(kind: "webrtc/request", payload: #"{"v":1,"action":"start","session":"not-a-uuid"}"#) == nil)
        precondition(FpvSignaling.decode(kind: "webrtc/request", payload: String(repeating: "{", count: FpvSignaling.maxBytes + 1)) == nil)

        let start = FpvSignaling.decode(kind: "webrtc/request", payload: #"{"v":1,"action":"start","session":"\#(first)","note":"optional"}"#)
        precondition(start == .start(session: first))
        precondition(machine.receive(start!) == .start(previous: nil, session: first))
        precondition(machine.receive(start!) == .ignore)

        let staleAnswer = FpvSignaling.decode(kind: "webrtc/answer", payload: #"{"v":1,"session":"\#(second)","sdp":"v=0"}"#)
        precondition(machine.receive(staleAnswer!) == .ignore)
        let answer = FpvSignaling.decode(kind: "webrtc/answer", payload: #"{"v":1,"session":"\#(first.uppercased())","sdp":"v=0\r\n"}"#)
        precondition(machine.receive(answer!) == .answer(session: first, sdp: "v=0\r\n"))

        let ice = FpvSignaling.decode(kind: "webrtc/ice/dashboard", payload: #"{"v":1,"session":"\#(first)","candidate":"candidate:1","sdpMid":"0","sdpMLineIndex":0,"extra":true}"#)
        precondition(machine.receive(ice!) == .ice(session: first, candidate: "candidate:1", sdpMid: "0", sdpMLineIndex: 0))
        let end = FpvSignaling.decode(kind: "webrtc/ice/dashboard", payload: #"{"v":1,"session":"\#(first)","candidate":"","sdpMid":null,"sdpMLineIndex":0}"#)
        precondition(machine.receive(end!) == .endOfCandidates(session: first))
        precondition(FpvSignaling.decode(kind: "webrtc/ice/dashboard", payload: #"{"v":1,"session":"\#(first)","candidate":"x","sdpMid":"0","sdpMLineIndex":1.5}"#) == nil)

        let replacement = FpvInbound.start(session: second)
        precondition(machine.receive(replacement) == .start(previous: first, session: second))
        precondition(machine.receive(.answer(session: first, sdp: "old")) == .ignore)
        precondition(machine.receive(.stop(session: first)) == .ignore)
        precondition(machine.receive(.stop(session: second)) == .stop(session: second))
        precondition(machine.receive(.answer(session: second, sdp: "late")) == .ignore)
        precondition(!machine.hasSession)

        precondition(machine.receive(.start(session: first)) == .start(previous: nil, session: first))
        precondition(machine.noteFailed() == first)
        precondition(machine.noteFailed() == nil)
        precondition(machine.receive(.answer(session: first, sdp: "after-fail")) == .ignore)
        precondition(machine.receive(.ice(session: first, candidate: "late", sdpMid: "0", sdpMLineIndex: 0)) == .ignore)
        precondition(machine.receive(.start(session: first)) == .start(previous: first, session: first))
        precondition(machine.stopLocal() == first)
        precondition(machine.stopLocal() == nil)

        let offer = FpvSignaling.offerJSON(session: first, sdp: "v=0\r\no=- 1 1 IN IP4 127.0.0.1\r\n")!
        let offerObject = try! JSONSerialization.jsonObject(with: Data(offer.utf8)) as! [String: Any]
        precondition(Set(offerObject.keys) == Set(["v", "session", "sdp"]))
        let state = FpvSignaling.stateJSON(session: second, state: "connected")!
        let stateObject = try! JSONSerialization.jsonObject(with: Data(state.utf8)) as! [String: Any]
        precondition(stateObject["state"] as? String == "connected")
        precondition(FpvSignaling.stateJSON(session: second, state: "armed") == nil)
        let huge = FpvSignaling.offerJSON(session: first, sdp: String(repeating: "a", count: FpvSignaling.maxBytes))
        precondition(huge == nil)

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

        print("FPV signaling session, size cap, and ICE redaction checks passed")
    }
}
