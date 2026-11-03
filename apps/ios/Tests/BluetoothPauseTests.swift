import Foundation
// This test exercises session state only; wire encoding is covered by actuator tests.
func actuatorEncodeFrame(frameJson: String) throws -> Data { Data(frameJson.utf8) }
@main struct BluetoothPauseTests {
    static func main() throws {
        let session = BluetoothSession()
        var status: [String: Any] = ["type":"status", "schema_version":1,
            "session":UInt32(7), "active_revision":UInt32(1), "layout_available":true,
            "armed":false, "arming":false]
        _ = session.status(status, now: 1)
        session.synchronized(revision: 1)
        _ = try session.arm(now: 1.01)
        status["armed"] = true
        status["stop_reason"] = "watchdog_paused"
        _ = session.status(status, now: 1.02)
        precondition(session.armed && session.motionPaused)
        // An explicit request does not fabricate the remote recovery acknowledgment.
        _ = try session.arm(now: 1.03)
        precondition(session.motionPaused)
        status["stop_reason"] = "awaiting_command"
        _ = session.status(status, now: 1.04)
        precondition(session.armed && !session.motionPaused)
        session.stop()
        _ = session.status(status, now: 1.05)
        precondition(!session.armed)
        print("Bluetooth pause state checks passed")
    }
}
