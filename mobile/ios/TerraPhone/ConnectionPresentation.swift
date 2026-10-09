import Foundation

struct TerraConnectionFacts {
    enum Radio { case unknown, ready, off, unauthorized }
    enum RoverState { case simulator, searching, pairing, connecting, configuring, connected, needsSetup, paused, bluetoothOff, permissionRequired, retrying, failed }
    enum FleetState { case waitingForRover, setupRequired, connecting, connected, paused, waiting, retrying }
    var radio: Radio = .unknown
    var automatic = false
    var pairingOffered = false
    var attempting = false
    var authenticated = false
    var configurationAvailable = false
    var hardwareReady = false
    var hardwareFault = false
    var roverFailed = false
    var fleetConfigured = false
    var fleetAttempting = false
    var fleetConnected = false
    var fleetPaused = false
    var fleetFailed = false
    var simulator = false

    var shouldTrackForFleet: Bool { fleetConnected && authenticated && configurationAvailable }

    var roverState: RoverState {
        if simulator { return .simulator }
        if radio == .unauthorized { return .permissionRequired }
        if radio == .off { return .bluetoothOff }
        if authenticated {
            if !configurationAvailable { return .configuring }
            if hardwareFault || (configurationAvailable && !hardwareReady) { return .needsSetup }
            return hardwareReady ? .connected : .configuring
        }
        if pairingOffered { return .pairing }
        if attempting { return .connecting }
        if roverFailed { return automatic ? .retrying : .failed }
        return automatic ? .searching : .paused
    }
    var fleetState: FleetState {
        if fleetConnected { return .connected }
        if !fleetConfigured { return .setupRequired }
        if !authenticated || !configurationAvailable { return .waitingForRover }
        if fleetAttempting { return .connecting }
        if fleetPaused { return .paused }
        if fleetFailed { return .retrying }
        return .waiting
    }
    var summary: String {
        if roverState == .needsSetup { return "Rover needs attention" }
        if !authenticated { return "Connect your rover" }
        if !configurationAvailable { return "Preparing your rover" }
        if fleetState == .setupRequired { return "Set up fleet connection" }
        if fleetConnected { return "Connected to fleet" }
        if fleetState == .paused { return "Fleet connection paused" }
        return "Connecting to fleet"
    }
}
