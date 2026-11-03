import OSLog
import ARKit
import AVFoundation
import Combine
import CoreMotion
import CoreLocation
import Foundation
import QuartzCore
import simd

// Occupancy mapping: ground detection, depth integration and map publication.
extension PhoneController {
    func clearMap() {
        controlQueue.async {
            do { try self.occupancyMap?.clear(); self.lastMapTime = -Double.infinity; self.floorEpoch &+= 1
                DispatchQueue.main.async { self.occupancy = nil } }
            catch { DispatchQueue.main.async { self.mapStatus = error.localizedDescription } }
        }
    }
    private func publishMap(x: Double, y: Double, yaw: Double, status: String) throws {
        guard let grid = try occupancyMap?.snapshot() else { return }
        occupancyPublicationRevision += 1
        try controller?.autonomyMap(grid:grid,timestamp:lastMapTime.isFinite ? lastMapTime:simulatedTime,revision:occupancyPublicationRevision)
        if let controller {
            observationCapture.captureMap(grid: grid, exposure: lastMapTime.isFinite ? lastMapTime : simulatedTime, frameID: localizationPolicy.identifier, revision: occupancyPublicationRevision, rover: UserDefaults.standard.string(forKey: "dashboardRoverID") ?? "_producer", controller: controller)
        }
        DispatchQueue.main.async { self.occupancy = grid; self.mapPose = SIMD3(x, y, yaw); self.mapStatus = status }
    }
    func updateSimulatedMap(x: Double, y: Double, yaw: Double, timestamp: Double) throws {
        guard let map = occupancyMap else { return }
        let frame = try simulatedRoomDepth(x: x, y: y, yaw: yaw)
        try map.recenter(x: x, y: y)
        try map.integrateDepth(frame: MappingDepthFrame(timestamp: timestamp, width: frame.width, height: frame.height, fx: frame.fx, fy: frame.fy, cx: frame.cx, cy: frame.cy, cameraX: frame.cameraX, cameraY: frame.cameraY, cameraZ: frame.cameraZ, quaternionX: frame.quaternionX, quaternionY: frame.quaternionY, quaternionZ: frame.quaternionZ, quaternionW: frame.quaternionW, depthMetres: frame.depthMetres))
        try publishMap(x: x, y: y, yaw: yaw, status: "Simulated depth · 10 Hz")
    }
    func updatePhoneGround(frame: ARFrame, position: SIMD3<Float>, bodyUp: SIMD3<Float>) {
        // ARKit world origin is the initial camera, not the floor. A measured mount
        // height fixes the floor directly, like a robot model's camera transform;
        // without one, use a stable classified floor to calibrate clearance.
        // The level supporting plane follows camera elevation during integration.
        if mapGroundOffset == nil, mountCalibration.isPlausible {
            mapGroundOffset = groundPolicy.mount(clearance: mountCalibration.cameraHeight, cameraHeight: Double(position.z))
            guard mapGroundOffset != nil else { return }
            try? occupancyMap?.clear(); lastFloorTime = -Double.infinity; floorEpoch &+= 1
        }
        if mapGroundOffset == nil {
            let floors = frame.anchors.compactMap { $0 as? ARPlaneAnchor }.filter {
                $0.alignment == .horizontal && ($0.classification == .floor || { if case .none = $0 { return true } else { return false } }($0.classification))
                && Double(position.z) - Double($0.transform.columns.3.y) >= 0.1
                && $0.planeExtent.width * $0.planeExtent.height >= 0.4
            }
            let classified = floors.filter { $0.classification == .floor }
            let floor = (classified.isEmpty ? floors : classified).min { $0.transform.columns.3.y < $1.transform.columns.3.y }
            var height = floor.map { Double(($0.transform * SIMD4<Float>($0.center.x, $0.center.y, $0.center.z, 1)).y) }
            var area = floor.map { Double($0.planeExtent.width * $0.planeExtent.height) } ?? 0
            // No floor plane yet: the classified mesh's floor near the camera is the next best measurement.
            if floor == nil, let mesh = occupancyMap?.floorEstimate(x: Double(position.x), y: Double(position.y), radius: 2.0) {
                height = mesh.height; area = mesh.area
            }
            mapGroundOffset = groundPolicy.observe(height: height, cameraHeight: Double(position.z), area: area, now: frame.timestamp, upZ: Double(bodyUp.z))
            // The level-band map reports its own "finding floor" status meanwhile.
            guard mapGroundOffset != nil else { return }
            try? occupancyMap?.clear(); lastFloorTime = -Double.infinity; floorEpoch &+= 1
        }
    }
    func updatePhoneMap(frame: ARFrame, position: SIMD3<Float>, roverPosition: SIMD3<Float>, bodyOrientation: simd_quatf) {
        guard frame.timestamp - lastMapTime >= 0.1 else { return }
        guard let depth = frame.sceneDepth, let map = occupancyMap else {
            observation.recordChanged(component: "mapping", kind: "mapping.health", attributes: ["reason": frame.sceneDepth == nil ? "depth_unavailable" : "map_unavailable"])
            return
        }
        if frame.timestamp - lastFloorTime >= 1, !floorUpdateInFlight {
            lastFloorTime = frame.timestamp
            floorUpdateInFlight = true
            let lift = mapGroundOffset ?? 0
            let anchors = frame.anchors.compactMap { $0 as? ARMeshAnchor }
            let worldFromAR = self.worldFromAR, epoch = floorEpoch, frameTime = frame.timestamp
            // The mesh walk can exceed 100 ms; on the control queue it stalled VIO
            // and IMU once a second, which tripped SensorNotReady holds. Handing
            // up to 600,000 floats to Rust is also slow (UniFFI encodes them one
            // by one), so that runs here too. The map has its own lock.
            floorQueue.async { [weak self] in
                let samples = Self.collectMeshSamples(anchors: anchors, worldFromAR: worldFromAR, lift: Float(lift))
                let cells = try? map.updateFloorSurface(cameraX: Double(position.x), cameraY: Double(position.y), cameraZ: Double(position.z) + lift, xyz: samples.floor)
                let barriers = try? map.updateBarrierSurface(cameraX: Double(position.x), cameraY: Double(position.y), xyz: samples.barrier)
                self?.controlQueue.async {
                    guard let self else { return }
                    let applyStarted = CACurrentMediaTime()
                    defer { SensorTimingProbe.shared.record(.block, sensor: frameTime, queue: applyStarted, done: CACurrentMediaTime(), a: 1, b: Double(samples.floor.count + samples.barrier.count)) }
                    self.floorUpdateInFlight = false
                    // A map or ground reset happened meanwhile: these samples use the
                    // old height frame, so turn the floor veto and barriers off again.
                    guard self.floorEpoch == epoch else {
                        _ = try? map.updateFloorSurface(cameraX: Double(position.x), cameraY: Double(position.y), cameraZ: Double(position.z), xyz: [])
                        _ = try? map.updateBarrierSurface(cameraX: Double(position.x), cameraY: Double(position.y), xyz: [])
                        return
                    }
                    self.observation.recordChanged(component: "mapping", kind: "mapping.floor_mesh", attributes: ["available": (cells ?? 0) > 0])
                    self.observation.recordChanged(component: "mapping", kind: "mapping.barrier_mesh", attributes: ["available": (barriers ?? 0) > 0])
                }
            }
        }
        let up = bodyOrientation.act(SIMD3<Float>(0, 0, 1))
        // Nil until ARKit finds the floor; the level band maps in the meantime.
        let ground = groundPolicy.groundPlane(
            cameraPosition: SIMD3(Double(position.x), Double(position.y), Double(position.z)),
            bodyUp: SIMD3(Double(up.x), Double(up.y), Double(up.z)))
        let buffer = depth.depthMap
        guard CVPixelBufferGetPixelFormatType(buffer) == kCVPixelFormatType_DepthFloat32,
              CVPixelBufferLockBaseAddress(buffer, .readOnly) == kCVReturnSuccess else {
            observation.recordChanged(component: "mapping", kind: "mapping.health", attributes: ["reason": "depth_buffer_unavailable"])
            return
        }
        defer { CVPixelBufferUnlockBaseAddress(buffer, .readOnly) }
        let width = CVPixelBufferGetWidth(buffer), height = CVPixelBufferGetHeight(buffer)
        guard let base = CVPixelBufferGetBaseAddress(buffer) else { return }
        let stride = CVPixelBufferGetBytesPerRow(buffer)
        var values = [Float](repeating: .nan, count: width * height)
        for row in 0..<height {
            let pixels = base.advanced(by: row * stride).assumingMemoryBound(to: Float.self)
            for col in 0..<width { values[row * width + col] = pixels[col] }
        }
        if let confidence = depth.confidenceMap,
           CVPixelBufferGetWidth(confidence) == width, CVPixelBufferGetHeight(confidence) == height,
           CVPixelBufferLockBaseAddress(confidence, .readOnly) == kCVReturnSuccess {
            defer { CVPixelBufferUnlockBaseAddress(confidence, .readOnly) }
            if let cbase = CVPixelBufferGetBaseAddress(confidence) {
                let cstride = CVPixelBufferGetBytesPerRow(confidence)
                for row in 0..<height { let pixels = cbase.advanced(by: row * cstride).assumingMemoryBound(to: UInt8.self)
                    for col in 0..<width where pixels[col] == 0 { values[row * width + col] = .nan }
                }
            }
        }
        let k = frame.camera.intrinsics
        let sx = Double(width) / Double(frame.camera.imageResolution.width)
        let sy = Double(height) / Double(frame.camera.imageResolution.height)
        // AR camera axes right/up/back -> optical right/down/forward.
        let q = (simd_quatf(worldFromAR) * simd_quatf(frame.camera.transform) * simd_quatf(angle: .pi, axis: SIMD3(1, 0, 0))).vector
        guard let ground, mapGroundOffset != nil else {
            integrateHorizonBand(frame: frame, map: map, values: values, width: width, height: height, sx: sx, sy: sy, q: q, position: position, roverPosition: roverPosition, bodyOrientation: bodyOrientation)
            return
        }
        do {
            try map.recenter(x: Double(roverPosition.x), y: Double(roverPosition.y))
            try map.integrateDepthWithGroundPlane(frame: MappingDepthFrame(timestamp: frame.timestamp, width: UInt32(width), height: UInt32(height), fx: Double(k.columns.0.x) * sx, fy: Double(k.columns.1.y) * sy, cx: (Double(k.columns.2.x) + 0.5) * sx - 0.5, cy: (Double(k.columns.2.y) + 0.5) * sy - 0.5, cameraX: Double(position.x), cameraY: Double(position.y), cameraZ: Double(position.z) + (mapGroundOffset ?? 0), quaternionX: Double(q.x), quaternionY: Double(q.y), quaternionZ: Double(q.z), quaternionW: Double(q.w), depthMetres: values), ground: MappingGroundPlane(normalX: ground.normal.x, normalY: ground.normal.y, normalZ: ground.normal.z, offset: ground.offset))
            lastMapTime = frame.timestamp
            observation.recordChanged(component: "mapping", kind: "mapping.health", attributes: ["reason": "accepted"])
            observation.record(component: "mapping", kind: "mapping.integrated", attributes: ["exposure_ns": UInt64(frame.timestamp * 1e9), "valid_depth_samples": values.reduce(0) { $0 + ($1.isFinite && $1 > 0 ? 1 : 0) }, "total_depth_samples": values.count, "ground_normal": [ground.normal.x, ground.normal.y, ground.normal.z], "ground_offset": ground.offset])
            let forward = bodyOrientation.act(SIMD3<Float>(1, 0, 0))
            try publishMap(x: Double(roverPosition.x), y: Double(roverPosition.y), yaw: atan2(Double(forward.y), Double(forward.x)), status: "ARKit scene depth · terrain-relative ground · 10 Hz")
            if let controller {
                let bodyRelative = bodyOrientation.inverse.act(position - roverPosition)
                let mountQ = (bodyOrientation.inverse * simd_quatf(vector: q)).vector
                let run = controller.observationRunId()
                let k = frame.camera.intrinsics
                let metadata: [String: Any] = [
                    "rover_id": UserDefaults.standard.string(forKey: "dashboardRoverID") ?? "_producer", "session_id": run,
                    "exposure_ns": UInt64(frame.timestamp * 1e9), "frame_id": localizationPolicy.identifier,
                    "depth_intrinsics": ["width": width, "height": height, "fx": Double(k.columns.0.x) * sx, "fy": Double(k.columns.1.y) * sy, "cx": (Double(k.columns.2.x) + 0.5) * sx - 0.5, "cy": (Double(k.columns.2.y) + 0.5) * sy - 0.5],
                    "rgb_intrinsics": ["width": Int(frame.camera.imageResolution.width), "height": Int(frame.camera.imageResolution.height), "fx": Double(k.columns.0.x), "fy": Double(k.columns.1.y), "cx": Double(k.columns.2.x), "cy": Double(k.columns.2.y)],
                    "image_orientation": "ARKit native landscape pixels; optical right/down/forward",
                    "camera_to_world": ["position": [Double(position.x), Double(position.y), Double(position.z) + (mapGroundOffset ?? 0)], "quaternion_xyzw": [q.x, q.y, q.z, q.w]],
                    "camera_to_body": ["position": [bodyRelative.x, bodyRelative.y, bodyRelative.z], "quaternion_xyzw": [mountQ.x, mountQ.y, mountQ.z, mountQ.w]],
                    "body_pose": ["position": [Double(roverPosition.x), Double(roverPosition.y), Double(roverPosition.z) + (mapGroundOffset ?? 0)], "quaternion_xyzw": [bodyOrientation.vector.x, bodyOrientation.vector.y, bodyOrientation.vector.z, bodyOrientation.vector.w]],
                    "ground_plane": ["normal": [ground.normal.x, ground.normal.y, ground.normal.z], "offset": ground.offset],
                    "clearance": groundPolicy.clearance ?? 0, "world_offset": mapGroundOffset ?? 0,
                    "thresholds": ["min_obstacle_height": occupancySettings.minObstacleHeight, "max_obstacle_height": occupancySettings.maxObstacleHeight, "max_range": occupancySettings.maxRange, "pixel_stride": occupancySettings.pixelStride],
                    "map_revision": occupancyPublicationRevision, "integration_result": "accepted"]
                observationCapture.capture(image: frame.capturedImage, exposure: frame.timestamp, depth: values, width: width, height: height, grid: try? map.snapshot(), manifest: metadata, controller: controller)
            }
        } catch { observation.recordChanged(component: "mapping", kind: "mapping.health", attributes: ["reason": "integration_failed", "error": error.localizedDescription]); DispatchQueue.main.async { self.mapStatus = "Map: " + error.localizedDescription } }
    }
    /// Centres of mesh faces ARKit classifies as floor, and as wall, window or
    /// door, as world x, y, z triples lifted by the same ground offset as depth
    /// frames. Rust decides what floor vetoes and where barriers stop rays.
    private static func collectMeshSamples(anchors: [ARMeshAnchor], worldFromAR: simd_float3x3, lift: Float) -> (floor: [Float], barrier: [Float]) {
        var floor: [Float] = [], barrier: [Float] = []
        let floorClass = UInt8(ARMeshClassification.floor.rawValue)
        let barrierClasses = Set([ARMeshClassification.wall, .window, .door].map { UInt8($0.rawValue) })
        let cap = 300_000
        for anchor in anchors {
            let geometry = anchor.geometry
            let faces = geometry.faces, vertices = geometry.vertices
            guard let classes = geometry.classification, classes.count == faces.count,
                  faces.indexCountPerPrimitive == 3, vertices.format == .float3 else { continue }
            let faceBase = faces.buffer.contents(), vertexBase = vertices.buffer.contents(), classBase = classes.buffer.contents()
            let transform = anchor.transform
            for face in 0..<faces.count {
                let kind = classBase.load(fromByteOffset: classes.offset + classes.stride * face, as: UInt8.self)
                let isFloor = kind == floorClass, isBarrier = barrierClasses.contains(kind)
                guard (isFloor && floor.count < cap) || (isBarrier && barrier.count < cap) else { continue }
                var centre = SIMD3<Float>(repeating: 0)
                for corner in 0..<3 {
                    let at = (face * 3 + corner) * faces.bytesPerIndex
                    let index = faces.bytesPerIndex == 4 ? Int(faceBase.load(fromByteOffset: at, as: UInt32.self)) : Int(faceBase.load(fromByteOffset: at, as: UInt16.self))
                    let v = vertexBase.advanced(by: vertices.offset + vertices.stride * index).assumingMemoryBound(to: Float.self)
                    centre += SIMD3(v[0], v[1], v[2])
                }
                let ar = transform * SIMD4(centre / 3, 1)
                let p = worldFromAR * SIMD3(ar.x, ar.y, ar.z)
                if isFloor { floor.append(contentsOf: [p.x, p.y, p.z + lift]) } else { barrier.append(contentsOf: [p.x, p.y, p.z + lift]) }
                if floor.count >= cap && barrier.count >= cap { return (floor, barrier) }
            }
        }
        return (floor, barrier)
    }
    /// Half-height of the level depth band used before the floor is found, degrees.
    private static let horizonBandHalfAngleDegrees = 10.0
    /// Map from a level slice of the depth view when no floor reference exists yet.
    /// Every return in the slice is an obstacle, so returns are cut off where the
    /// slice's lower edge meets the floor at the measured (or nominal 0.5 m) camera height.
    private func integrateHorizonBand(frame: ARFrame, map: MobileOccupancyMap, values: [Float], width: Int, height: Int, sx: Double, sy: Double, q: SIMD4<Float>, position: SIMD3<Float>, roverPosition: SIMD3<Float>, bodyOrientation: simd_quatf) {
        let k = frame.camera.intrinsics
        let cameraHeight = mountCalibration.cameraHeight > 0 ? mountCalibration.cameraHeight : 0.5
        let halfAngle = Self.horizonBandHalfAngleDegrees
        let maxRange = cameraHeight / tan(halfAngle * .pi / 180)
        do {
            try map.recenter(x: Double(roverPosition.x), y: Double(roverPosition.y))
            try map.integrateDepthInHorizonBand(frame: MappingDepthFrame(timestamp: frame.timestamp, width: UInt32(width), height: UInt32(height), fx: Double(k.columns.0.x) * sx, fy: Double(k.columns.1.y) * sy, cx: (Double(k.columns.2.x) + 0.5) * sx - 0.5, cy: (Double(k.columns.2.y) + 0.5) * sy - 0.5, cameraX: Double(position.x), cameraY: Double(position.y), cameraZ: Double(position.z) + (mapGroundOffset ?? 0), quaternionX: Double(q.x), quaternionY: Double(q.y), quaternionZ: Double(q.z), quaternionW: Double(q.w), depthMetres: values), halfAngleDegrees: halfAngle, maxRange: maxRange)
            lastMapTime = frame.timestamp
            observation.recordChanged(component: "mapping", kind: "mapping.health", attributes: ["reason": "accepted_horizon_band"])
            let forward = bodyOrientation.act(SIMD3<Float>(1, 0, 0))
            // With a floor offset already known, the band means the ground plane refused
            // the body's up axis (over 60° from vertical): name the tilt so a bad mount
            // roll/pitch is visible instead of looking like "floor not found".
            let up = bodyOrientation.act(SIMD3<Float>(0, 0, 1))
            let tilt = acos(Double(max(-1, min(1, up.z / max(simd_length(up), 1e-6))))) * 180 / .pi
            let reason = mapGroundOffset == nil ? "finding floor" : String(format: "floor known but body tilted %.0f° · check mount roll/pitch", tilt)
            try publishMap(x: Double(roverPosition.x), y: Double(roverPosition.y), yaw: atan2(Double(forward.y), Double(forward.x)), status: String(format: "ARKit scene depth · level band ±%.0f° to %.1f m · %@ · 10 Hz", halfAngle, maxRange, reason))
        } catch { observation.recordChanged(component: "mapping", kind: "mapping.health", attributes: ["reason": "integration_failed", "error": error.localizedDescription]); DispatchQueue.main.async { self.mapStatus = "Map: " + error.localizedDescription } }
    }
}
