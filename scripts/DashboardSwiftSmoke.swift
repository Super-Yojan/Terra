import Foundation
import TerraCore
import ArgosCore

// Exercises real generated bindings and both Rust clients through a loopback router.
@main enum DashboardSwiftSmoke {
    static func main() throws {
        guard CommandLine.arguments.count == 2 else { fatalError("Pass the local router endpoint") }
        let endpoint = CommandLine.arguments[1]
        let phone = try MobileController(settings: defaultControlSettings())
        let operatorClient = ArgosClient()
        try operatorClient.connect(config: ConnectionConfig(endpoint: endpoint, prefix: "phone-smoke"))
        try phone.connectDashboard(endpoint: endpoint, prefix: "phone-smoke", roverId: 7)
        defer { try? phone.disconnectDashboard(); operatorClient.disconnect() }
        func tick() throws {
            let t = ProcessInfo.processInfo.systemUptime
            try phone.pushImu(sample: ImuReading(timestamp: t, accelerationForward: 0, accelerationLeft: 0, accelerationUp: 0, gyroRoll: 0, gyroPitch: 0, gyroYaw: 0))
            try phone.pushVio(sample: VioReading(timestamp: t, positionX: 2, positionY: -3, positionZ: 0, quaternionX: 0, quaternionY: 0, quaternionZ: 0, quaternionW: 1, velocityX: 0, velocityY: 0, velocityZ: 0, tracked: true))
            try phone.autonomyMap(grid: OccupancyGrid(width: 40, height: 40, resolution: 0.25, originX: -3, originY: -8, occupancy: [Int8](repeating: 0, count: 1600)), timestamp: t, revision: 1)
            _ = try phone.step(timestamp: t)
        }
        func waitFor(_ description: String, _ predicate: () -> Bool) throws {
            let deadline = Date().addingTimeInterval(5)
            while !predicate() {
                guard Date() < deadline else { fatalError("Timed out: \(description)") }
                try tick()
                Thread.sleep(forTimeInterval: 0.01)
            }
        }
        try waitFor("phone membership and pose") { operatorClient.snapshot().rovers.first?.pose != nil }
        let pose = operatorClient.snapshot().rovers[0].pose!
        precondition(pose.x == 2 && pose.y == -3)
        try phone.autonomyRequest(kind: "autonomy", payload: "{\"level\":\"waypoint\",\"token\":\"mode\"}", timestamp: ProcessInfo.processInfo.systemUptime)
        let token = try operatorClient.sendGoal(roverId: 7, waypoint: .local(x: 3, y: -3, yaw: nil))
        try waitFor("matching phone goal acknowledgement") {
            let rover = operatorClient.snapshot().rovers.first
            return rover?.goal?.token == token && rover?.commandPhase == "active"
        }
        try operatorClient.cancelGoal(roverId: 7)
        try waitFor("phone idle after cancel") { operatorClient.snapshot().rovers.first?.commandPhase == "cancelled" }
        try phone.disconnectDashboard()
        precondition(phone.dashboardStatus() == "disconnected")
        try phone.connectDashboard(endpoint: endpoint, prefix: "phone-smoke", roverId: 7)
        try tick()
        let state = try JSONSerialization.jsonObject(with: Data(phone.autonomyStatus().utf8)) as! [String: Any]
        precondition((state["goal"] as! [String: Any])["state"] as! String == "idle")
        precondition((state["status"] as! [String: Any])["requested_level"] as! String == "teleop")
        print("Phone → router → ARGOS: pose, matching goal acknowledgement, cancel and explicit reconnect passed through generated Swift bindings")
    }
}
