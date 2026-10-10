import Foundation

@main struct ActuatorDriveProfileTests {
    static func main() throws {
        let json = """
        {"schema_version":1,"revision":7,"actuators":[
          {"id":0,"kind":"bidirectional_esc","limits":{"min":-0.8,"max":0.8},"route":{"type":"left_effort"},"safe":{"type":"zero"}},
          {"id":1,"kind":"dc_motor","limits":{"min":-1,"max":1},"route":{"type":"right_effort"},"safe":{"type":"zero"}},
          {"id":2,"kind":"positional_servo","limits":{"min":-0.5,"max":0.5},"route":{"type":"servo"},"safe":{"type":"position","value":0.25}}
        ]}
        """
        let profile = try ActuatorDriveProfileSet.decode(json: json)
        let fixed = json.replacingOccurrences(of: "\"min\":-0.8,\"max\":0.8", with: "\"min\":0,\"max\":0")
        let fixedProfile = try ActuatorDriveProfileSet.decode(json: fixed)
        precondition(fixedProfile.actuators[0].limits.max == 0)
        precondition(profile.supportsFeedback)
        precondition(profile.safeValues.map { $0["value"] as! Double } == [0, 0, 0.25])
        let input: [String: Any] = ["left_effort": 1.0, "right_effort": -0.6, "forward": 0.4, "turn": 0.2, "servo_positions": ["2": -0.8]]
        // Feedback routing has no manual command: its neutral values must remain
        // Double when placed in the heterogeneous actuator-input dictionary.
        let manualForward: Double? = nil
        let feedback = true
        let forward: Double = manualForward ?? (feedback ? 0 : 0.4)
        let turn: Double = manualForward ?? (feedback ? 0 : 0.2)
        let feedbackInput: [String: Any] = ["left_effort": 0.0, "right_effort": 0.0, "forward": forward, "turn": turn, "servo_positions": [String: Double]()]
        let neutralOutput = try profile.route(input: feedbackInput)
        precondition(neutralOutput.map { $0["value"] as! Double } == [0, 0, 0.25])
        let output = try profile.route(input: input)
        precondition(output.map { $0["value"] as! Double } == [0.8, -0.6, -0.5])
        func rejects(_ json: String) {
            do { _ = try ActuatorDriveProfileSet.decode(json: json); preconditionFailure("Invalid profile accepted") }
            catch { }
        }
        rejects(json.replacingOccurrences(of: "\"id\":1", with: "\"id\":0"))
        rejects(json.replacingOccurrences(of: "\"type\":\"zero\"", with: "\"type\":\"position\",\"value\":0.1"))
        rejects(json.replacingOccurrences(of: "\"type\":\"left_effort\"", with: "\"type\":\"servo\""))
        rejects(json.replacingOccurrences(of: "\"min\":-0.8", with: "\"min\":0.9"))
        let manual = json.replacingOccurrences(of: "\"type\":\"left_effort\"", with: "\"type\":\"manual\",\"forward_coefficient\":1,\"turn_coefficient\":-1")
        let manualProfile = try ActuatorDriveProfileSet.decode(json: manual)
        precondition(!manualProfile.supportsFeedback)
        let manualOutput = try manualProfile.route(input: input)
        precondition(abs((manualOutput[0]["value"] as! Double) - 0.2) < 0.00001)
        do { _ = try profile.route(input: ["left_effort": Double.nan]); preconditionFailure("Invalid input accepted") } catch { }
        var sync = ActuatorProfileSynchronization()
        try sync.begin(revision: 7, ids: [0, 1, 2])
        precondition(sync.completed == nil)
        try sync.accept(revision: 7, actuator: profile.actuators[0])
        do { try sync.accept(revision: 8, actuator: profile.actuators[1]); preconditionFailure("Mixed revisions accepted") } catch { }
        precondition(sync.completed == nil)
        try sync.accept(revision: 7, actuator: profile.actuators[1])
        try sync.accept(revision: 7, actuator: profile.actuators[2])
        precondition(sync.completed?.actuators.count == 3)
        sync.clear(); precondition(sync.completed == nil)
        print("Compact profiles: routing, safe values, validation and feedback passed")
    }
}
