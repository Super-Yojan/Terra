import Foundation

@main struct ActuatorConfigurationTests {
    static func main() throws {
        let disconnected = ActuatorConfigurationPolicy.blockingReason(ready: false, status: [:])
        precondition(disconnected != nil)
        let external: [String: Any] = ["armed": false, "arming": false, "configuration_allowed": true,
                                       "gate_mode": "external_power_cutoff"]
        precondition(ActuatorConfigurationPolicy.blockingReason(ready: true, status: external) == nil)
        var armed = external; armed["armed"] = true
        precondition(ActuatorConfigurationPolicy.blockingReason(ready: true, status: armed) != nil)
        var denied = external; denied["configuration_allowed"] = false
        precondition(ActuatorConfigurationPolicy.blockingReason(ready: true, status: denied) != nil)
        precondition(ActuatorConfigurationPolicy.blockingReason(ready: true, status: [:]) != nil)
        let legacy: [String: Any] = ["armed": false, "arming": false, "hardware_gate_open_confirmed": true]
        precondition(ActuatorConfigurationPolicy.blockingReason(ready: true, status: legacy) == nil)
        let layout = ActuatorLayoutDraft.rover1(revision: 7)
        precondition(layout.revision == 7 && layout.actuators.count == 2)
        precondition(layout.actuators.map(\.port) == ["P0", "P1"])
        precondition(layout.actuators.map(\.route.type) == ["left_effort", "right_effort"])
        precondition(layout.actuators.allSatisfy { $0.kind == "bidirectional_esc" && $0.calibration.neutral_us == 1500 && !$0.inverted })
        let roundTrip = try JSONDecoder().decode(ActuatorLayoutDraft.self, from: JSONEncoder().encode(layout))
        precondition(roundTrip == layout)
        print("Actuator configuration: policy and two-ESC preset checks passed")
    }
}
