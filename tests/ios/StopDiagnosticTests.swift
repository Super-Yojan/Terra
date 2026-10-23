import Foundation

@main struct StopDiagnosticTests {
    static func main() {
        var diagnostic = TerraStopDiagnostic()
        precondition(TerraStopDiagnostic.autonomyReason("{\"status\":{\"reason\":\"obstacle_blocked\"}}") == "obstacle_blocked")
        precondition(TerraStopDiagnostic.autonomyReason("invalid") == "unavailable")
        diagnostic.record("AR tracking has too few visual features", at: 10)
        diagnostic.record("Bluetooth disconnected", at: 11)
        precondition(diagnostic.message == "AR tracking has too few visual features")
        precondition(diagnostic.timestamp == 10)
        diagnostic.clear()
        precondition(diagnostic.message == nil)
        let watchdog: [String: Any] = ["armed": false, "stop_reason": "watchdog"]
        precondition(TerraStopDiagnostic.piDisarmReason(wasArmed: true, isArmed: false, status: watchdog) == "Pi disarmed: watchdog")
        precondition(TerraStopDiagnostic.piDisarmReason(wasArmed: false, isArmed: false, status: watchdog) == nil)
        precondition(TerraStopDiagnostic.piDisarmReason(wasArmed: true, isArmed: false, status: ["armed": true, "stop_reason": "old status"]) == nil)
        precondition(TerraStopDiagnostic.piDisarmReason(wasArmed: true, isArmed: false, status: nil) == nil)
        diagnostic.record("Pi watchdog", at: 12)
        precondition(diagnostic.message == "Pi watchdog")
        diagnostic.clear()
        diagnostic.record("", at: 13)
        precondition(diagnostic.message == nil)
        print("First stop cause survives secondary disconnects and can be cleared")
    }
}
