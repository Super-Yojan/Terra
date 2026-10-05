import Foundation
@main
struct SwiftSmoke {
    static func main() throws {
        let brain = try MobileController(settings: defaultControlSettings())
        try brain.pushImu(sample: ImuReading(timestamp: 0, accelerationForward: 0, accelerationLeft: 0, accelerationUp: 0, gyroRoll: 0, gyroPitch: 0, gyroYaw: 0))
        try brain.pushVio(sample: VioReading(timestamp: 0, positionX: 0, positionY: 0, positionZ: 0, quaternionX: 0, quaternionY: 0, quaternionZ: 0, quaternionW: 1, velocityX: 0, velocityY: 0, velocityZ: 0, tracked: true))
        try brain.setTarget(target: TwistSetpoint(timestamp: 0, forward: 1, yawRate: 0.3))
        let output = try brain.step(timestamp: 0)
        precondition(output.safety == .active && output.rightEffort > output.leftEffort)
        let stale = try brain.step(timestamp: 1)
        precondition(stale.leftEffort == 0 && stale.rightEffort == 0)
        let map = try MobileOccupancyMap(settings: defaultOccupancySettings())
        try map.integrateDepth(frame: MappingDepthFrame(timestamp: 0, width: 1, height: 1, fx: 1, fy: 1, cx: 0, cy: 0, cameraX: 0, cameraY: 0, cameraZ: 0.5, quaternionX: -0.5, quaternionY: 0.5, quaternionZ: -0.5, quaternionW: 0.5, depthMetres: [2]))
        let grid = try map.snapshot()
        precondition(grid.occupancy.contains { $0 > 50 } && grid.occupancy.contains { $0 < 50 && $0 >= 0 })
        try map.recenter(x: 1, y: 0)
        try map.clear()
        let cleared = try map.snapshot()
        precondition(cleared.occupancy.allSatisfy { $0 == -1 })
        if let endpoint = ProcessInfo.processInfo.environment["TERRA_ZENOH_SMOKE_ENDPOINT"] {
            let remote = try MobileZenohClient(endpoint: endpoint, prefix: "terra/rover", roverId: 9)
            for _ in 0..<20 {
                try remote.setTarget(linear: 0.5, angular: 0.2)
                Thread.sleep(forTimeInterval: 0.05)
            }
            // Stop refreshing: the Rust lease must send zero while the connection stays open.
            Thread.sleep(forTimeInterval: 0.4)
            remote.disconnect()
            precondition(remote.status() == "Disconnected")
            print("Swift → UniFFI → Zenoh network path passed")
        }
        let report = try runVelocityBenchmark()
        precondition(report.passed)
        print(String(format: "Swift → UniFFI → Rust passed: v=%.3f m/s, yaw=%.3f rad/s", report.finalForward, report.finalYawRate))
    }
}
