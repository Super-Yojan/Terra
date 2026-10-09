import Foundation
@main struct PresentationTests {
    static func main() {
        var facts = TerraConnectionFacts()
        precondition(!facts.shouldTrackForFleet)
        facts.fleetConnected = true
        precondition(!facts.shouldTrackForFleet)
        facts.authenticated = true; facts.configurationAvailable = true
        precondition(facts.shouldTrackForFleet)
        // Every ARGOS autonomy mode shares this lifecycle; discovery preference is irrelevant.
        facts.automatic = false; facts.hardwareReady = false
        precondition(facts.shouldTrackForFleet)
        facts.fleetConnected = false
        precondition(!facts.shouldTrackForFleet)
        facts = TerraConnectionFacts()
        precondition(facts.roverState == .paused)
        facts.automatic = true
        precondition(facts.roverState == .searching)
        facts.radio = .off
        precondition(facts.roverState == .bluetoothOff)
        facts.radio = .unauthorized
        precondition(facts.roverState == .permissionRequired)
        facts.radio = .ready; facts.pairingOffered = true
        precondition(facts.roverState == .pairing)
        facts.pairingOffered = false; facts.attempting = true
        precondition(facts.roverState == .connecting)
        facts.attempting = false; facts.authenticated = true
        precondition(facts.roverState == .configuring)
        facts.hardwareReady = true
        precondition(facts.roverState == .configuring)
        facts.hardwareReady = false
        facts.configurationAvailable = true
        precondition(facts.roverState == .needsSetup)
        facts.hardwareReady = true
        precondition(facts.roverState == .connected)
        precondition(facts.fleetState == .setupRequired)
        facts.fleetConfigured = true
        precondition(facts.fleetState == .waiting)
        facts.fleetAttempting = true
        precondition(facts.fleetState == .connecting)
        facts.fleetAttempting = false; facts.fleetConnected = true
        precondition(facts.summary == "Connected to fleet")
        facts.hardwareFault = true
        precondition(facts.summary == "Rover needs attention")
        facts.hardwareFault = false; facts.fleetConnected = false; facts.fleetPaused = true
        precondition(facts.fleetState == .paused)
        facts.authenticated = false
        precondition(facts.fleetState == .waitingForRover)
        facts.roverFailed = true; facts.automatic = true
        precondition(facts.roverState == .retrying)
        facts.automatic = false
        precondition(facts.roverState == .failed)
        facts.authenticated = true; facts.fleetPaused = false; facts.fleetFailed = true
        precondition(facts.fleetState == .retrying)
        facts.fleetConnected = true; facts.radio = .off
        precondition(facts.fleetState == .connected)
        facts.simulator = true
        precondition(facts.roverState == .simulator)
        print("Connection presentation: checks passed")
    }
}
