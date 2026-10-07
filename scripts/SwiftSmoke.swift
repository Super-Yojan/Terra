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
            // Live depth and cmd_vel come from Zorvane (`cargo run -p zorvane`).
            // This smoke still speaks terra/rover/9 against the Python peer.
            let remote = try MobileZenohClient(endpoint: endpoint, prefix: "terra/rover", roverId: 9)
            let deadline = Date().addingTimeInterval(3)
            var mapped = false
            while Date() < deadline {
                if let frame = remote.takeDepth() {
                    precondition(abs(frame.cameraZ - 0.5) < 1e-6 && abs(frame.bodyYaw) < 1e-6)
                    precondition(frame.width == 1 && frame.depthMetres.count == 1)
                    let remoteMap = try MobileOccupancyMap(settings: defaultOccupancySettings())
                    try remoteMap.recenter(x: frame.bodyX, y: frame.bodyY)
                    try remoteMap.integrateDepth(frame: MappingDepthFrame(timestamp: frame.timestamp, width: frame.width, height: frame.height, fx: frame.fx, fy: frame.fy, cx: frame.cx, cy: frame.cy, cameraX: frame.cameraX, cameraY: frame.cameraY, cameraZ: frame.cameraZ, quaternionX: frame.quaternionX, quaternionY: frame.quaternionY, quaternionZ: frame.quaternionZ, quaternionW: frame.quaternionW, depthMetres: frame.depthMetres))
                    let remoteGrid = try remoteMap.snapshot()
                    precondition(remoteGrid.occupancy.contains { $0 > 50 })
                    mapped = true
                    break
                }
                Thread.sleep(forTimeInterval: 0.05)
            }
            precondition(mapped, "posed simulator depth did not integrate into an occupied cell")
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
        let authority = try MobileController(settings: defaultControlSettings())
        func feed(_ t:Double) throws {
            try authority.pushImu(sample:ImuReading(timestamp:t,accelerationForward:0,accelerationLeft:0,accelerationUp:0,gyroRoll:0,gyroPitch:0,gyroYaw:0))
            try authority.pushVio(sample:VioReading(timestamp:t,positionX:0,positionY:0,positionZ:0,quaternionX:0,quaternionY:0,quaternionZ:0,quaternionW:1,velocityX:0,velocityY:0,velocityZ:0,tracked:true))
            var cells=Array(repeating:Int8(-1),count:1600)
            for y in 12..<28 {for x in 12..<28 {cells[y*40+x]=0}}
            try authority.autonomyMap(grid:OccupancyGrid(width:40,height:40,resolution:0.25,originX:-5,originY:-5,occupancy:cells),timestamp:t,revision:UInt64(t*10))
        }
        for (index,level) in ["teleop","assisted_teleop","waypoint","supervised"].enumerated() {
            let t=Double(index)*0.1
            try feed(t)
            try authority.autonomyRequest(kind:"autonomy",payload:"{\"level\":\"\(level)\",\"token\":\"mode-\(index)\"}",timestamp:t)
            if index<2 {try authority.setTarget(target:TwistSetpoint(timestamp:t,forward:1,yawRate:0))}
            if index==2 {try authority.autonomyRequest(kind:"goal",payload:"{\"frame\":\"local\",\"x\":1.5,\"y\":0,\"token\":\"goal\"}",timestamp:t)}
            _=try authority.step(timestamp:t)
            let state=try JSONSerialization.jsonObject(with:Data(authority.autonomyStatus().utf8)) as! [String:Any]
            precondition((state["status"] as! [String:Any])["requested_level"] as! String==level)
            if index==3 {
                let proposal=state["proposal"] as! [String:Any]
                let decision:[String:Any]=["decision":"approve","proposal_id":proposal["proposal_id"]!,"run_id":proposal["run_id"]!,"token":"approve"]
                try authority.autonomyRequest(kind:"goal/decision",payload:String(decoding:try JSONSerialization.data(withJSONObject:decision),as:UTF8.self),timestamp:t+0.01)
                _=try authority.step(timestamp:t+0.01)
            }
        }
        try authority.autonomyRequest(kind:"safety",payload:"{\"action\":\"stop\",\"token\":\"stop\"}",timestamp:0.4)
        let stopped=try authority.step(timestamp:0.4)
        precondition(stopped.leftEffort==0 && stopped.rightEffort==0)
        print("Four shared autonomy levels and stop passed through Swift bindings")
        let report = try runVelocityBenchmark()
        precondition(report.passed)
        print(String(format: "Swift → UniFFI → Rust passed: v=%.3f m/s, yaw=%.3f rad/s", report.finalForward, report.finalYawRate))
    }
}
