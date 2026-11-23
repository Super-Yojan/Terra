import ARKit
import CoreLocation
import CoreMotion
import Foundation
import simd

/// Stage 1, Sensors. Reads ARKit, Core Motion, Core Location and LiDAR depth
/// and keeps the latest value of each. No filtering, no frames changed:
/// mount and body-frame work happen in terra-av.
///
/// Inherits `NSObject` because Apple's Objective-C delegate protocols
/// (`ARSessionDelegate`, `CLLocationManagerDelegate`) require it.
final class PhoneSensors: NSObject, ARSessionDelegate, CLLocationManagerDelegate {
    /// ARKit visual-inertial odometry (camera pose + LiDAR depth + ARWorldMap).
    private let session = ARSession()
    /// Dedicated serial queue for ARKit callbacks and `vertexColorCache`.
    private let arQueue = DispatchQueue(label: "terra.sensors.ar")
    /// Spatial 5 cm voxel cache of packed `0xRRGGBB` vertex colors so vertices outside
    /// the current camera frustum keep their observed color across mesh updates.
    private var vertexColorCache: [Int64: UInt32] = [:]
    /// Core Motion IMU manager (gyroscope / accelerometer).
    private let motion = CMMotionManager()
    /// Core Location GPS manager.
    private let location = CLLocationManager()
    /// Mutex protecting `vio`, `gyro`, `gps`, and `depth`, which are written from
    /// different background queues (ARKit, Core Motion, Core Location) and read
    /// from the planner queue.
    private let lock = NSLock()
    private var vio: VioPose?
    private var gyro: Vec3?
    private var gps: GpsFix?
    private var depth: [Vec3] = []

    /// Keep about this many depth points per frame.
    private let depthTarget = 600

    /// Called on `terra.sensors.ar` whenever ARKit adds, updates, or removes an
    /// `ARMeshAnchor`. Empty `vertices`/`indices` means the chunk `id` was removed.
    /// `colors` holds flat `[r, g, b, ...]` sRGB bytes (3 per vertex, or empty).
    var onMeshChunk: ((_ id: String, _ vertices: [Vec3], _ indices: [UInt32], _ classes: Data, _ colors: Data) -> Void)?

    /// Called on `terra.sensors.ar` with each rear-camera pixel buffer for WebRTC FPV.
    var onCameraFrame: ((CVPixelBuffer, TimeInterval) -> Void)?

    func start() {
        // ARWorldTrackingConfiguration tracks 6-DoF camera pose using camera + IMU.
        let config = ARWorldTrackingConfiguration()
        // `.gravity` aligns ARKit's +Y axis straight up (opposite gravity),
        // with heading initialized to wherever the phone points at start.
        config.worldAlignment = .gravity
        // Enable LiDAR per-pixel depth map on Pro iPhones/iPads that support it.
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            config.frameSemantics.insert(.sceneDepth)
        }
        if ARWorldTrackingConfiguration.supportsSceneReconstruction(.meshWithClassification) {
            config.sceneReconstruction.insert(.meshWithClassification)
        }

        session.delegate = self
        // Run ARKit callbacks on a dedicated serial background queue instead of
        // the main UI thread so depth processing never stutters SwiftUI.
        session.delegateQueue = arQueue
        arQueue.async { [weak self] in self?.vertexColorCache.removeAll() }
        // Start fresh: discard any previous tracking origin or anchors.
        session.run(config, options: [.resetTracking, .removeExistingAnchors])

        // Poll IMU at 100 Hz (0.01 s) on a background OperationQueue.
        // `[weak self]` avoids a retain cycle if PhoneSensors is deallocated.
        motion.deviceMotionUpdateInterval = 0.01
        motion.startDeviceMotionUpdates(to: OperationQueue()) { [weak self] data, _ in
            guard let r = data?.rotationRate else { return }
            self?.store { $0.gyro = Vec3(x: r.x, y: r.y, z: r.z) }
        }

        location.delegate = self
        location.desiredAccuracy = kCLLocationAccuracyBest
        // Prompts the user for location permission if not already granted.
        location.requestWhenInUseAuthorization()
        location.startUpdatingLocation()
    }

    func stop() {
        session.pause()
        motion.stopDeviceMotionUpdates()
        location.stopUpdatingLocation()
        arQueue.async { [weak self] in self?.vertexColorCache.removeAll() }
        store { $0.vio = nil; $0.gyro = nil; $0.gps = nil; $0.depth = [] }
    }

    /// The latest reading of every sensor, stamped now.
    func frame(estop: Bool) -> SensorFrame {
        // `defer` guarantees `lock.unlock()` runs right when this function returns.
        lock.lock(); defer { lock.unlock() }
        // `systemUptime` is a monotonic clock (seconds since boot) — never jumps
        // backwards on NTP/timezone changes, unlike `Date()`.
        return SensorFrame(t: ProcessInfo.processInfo.systemUptime, estop: estop,
                           vio: vio, gyro: gyro, gps: gps, depth: depth)
    }

    /// Snapshot the current ARKit `ARWorldMap` (`NSKeyedArchiver` bytes + 3D
    /// `rawFeaturePoints` in ARKit world frame) to send to the base station.
    ///
    /// `@escaping` means the `completion` closure is called asynchronously later
    /// (after `captureWorldMap` has already returned) once ARKit finishes building the map.
    func captureWorldMap(_ completion: @escaping (Result<(points: [Vec3], data: Data), Error>) -> Void) {
        // Asynchronously asks ARKit to snapshot its current visual map (keyframes,
        // feature descriptors, anchors, and 3D point cloud).
        session.getCurrentWorldMap { worldMap, error in
            if let error {
                completion(.failure(error))
                return
            }
            guard let worldMap else {
                completion(.failure(NSError(domain: "terra.worldmap", code: 1,
                                            userInfo: [NSLocalizedDescriptionKey: "No ARWorldMap available yet"])))
                return
            }
            // Plain 3D coordinates of ARKit's visual landmarks (ARKit world A, metres),
            // used for rendering on macOS/Rust since the archive below is Apple-private.
            let points = worldMap.rawFeaturePoints.points.map { Vec3($0) }
            do {
                // Serialize the full `ARWorldMap` (including visual descriptors) into a
                // binary `Data` blob so another iPhone can unarchive it into `initialWorldMap`.
                let data = try NSKeyedArchiver.archivedData(withRootObject: worldMap, requiringSecureCoding: true)
                completion(.success((points: points, data: data)))
            } catch {
                completion(.failure(error))
            }
        }
    }

    // MARK: ARKit

    /// Called by ARKit at ~60 Hz on `terra.sensors.ar` with each new camera frame.
    func session(_: ARSession, didUpdate frame: ARFrame) {
        // Swift pattern matching: only mark `tracked = true` when ARKit is in `.normal`
        // state (not `.notAvailable` or `.limited(.initializing / .relocalizing / ...)`).
        var tracked = false
        if case .normal = frame.camera.trackingState { tracked = true }
        // `frame.camera.transform` is a 4×4 column-major matrix (camera → ARKit world).
        let pose = VioPose(arkit: frame.camera.transform, tracked: tracked)
        // `Optional.map` runs `depthPoints(...)` only when `sceneDepth` is non-nil;
        // `?? []` falls back to an empty array on devices without LiDAR.
        let points = frame.sceneDepth.map { depthPoints(from: $0, intrinsics: frame.camera.intrinsics,
                                                        imageSize: frame.camera.imageResolution) } ?? []
        store {
            $0.vio = pose
            $0.depth = points
        }
        onCameraFrame?(frame.capturedImage, frame.timestamp)
    }

    /// Called by ARKit when new mesh chunks are first created.
    func session(_: ARSession, didAdd anchors: [ARAnchor]) {
        emitMeshChunks(from: anchors)
    }

    /// Called by ARKit when existing mesh chunks are refined.
    func session(_: ARSession, didUpdate anchors: [ARAnchor]) {
        emitMeshChunks(from: anchors)
    }

    /// Called by ARKit when mesh chunks are discarded; sends an empty chunk so the
    /// base station removes that chunk ID from its map.
    func session(_: ARSession, didRemove anchors: [ARAnchor]) {
        guard let onMeshChunk else { return }
        for case let mesh as ARMeshAnchor in anchors {
            onMeshChunk(mesh.identifier.uuidString, [], [], Data(), Data())
        }
    }

    /// Extract vertices (in ARKit world `A`), triangle indices, per-face
    /// classifications, and per-vertex sRGB colors sampled from `capturedImage`.
    private func emitMeshChunks(from anchors: [ARAnchor]) {
        guard let onMeshChunk else { return }
        let meshAnchors = anchors.compactMap { $0 as? ARMeshAnchor }
        guard !meshAnchors.isEmpty else { return }

        // Lock the latest ARKit RGB frame (and optional LiDAR depth map for occlusion
        // rejection) once for the whole batch of mesh anchor updates.
        let sampler = session.currentFrame.flatMap(FrameColorSampler.init)
        defer { sampler?.unlock() }

        for mesh in meshAnchors {
            let geom = mesh.geometry
            let transform = mesh.transform

            // 1. Vertices: local (x, y, z) Float32s → ARKit world A via `anchor.transform`.
            // Note: `stride` is 12 bytes (4-byte aligned), while `SIMD3<Float>` requires
            // 16-byte alignment, so read three `Float`s instead of `.load(as: SIMD3<Float>.self)`.
            let vSource = geom.vertices
            let vBase = vSource.buffer.contents()
            var vertices: [Vec3] = []
            vertices.reserveCapacity(vSource.count)
            var colors = Data()
            if sampler != nil {
                colors.reserveCapacity(vSource.count * 3)
            }
            for i in 0 ..< vSource.count {
                let ptr = vBase
                    .advanced(by: vSource.offset + i * vSource.stride)
                    .assumingMemoryBound(to: Float.self)
                let local = SIMD3<Float>(ptr[0], ptr[1], ptr[2])
                let world4 = transform * SIMD4<Float>(local, 1.0)
                let world = SIMD3<Float>(world4.x, world4.y, world4.z)
                vertices.append(Vec3(world))

                if let sampler {
                    let key = Self.colorVoxelKey(world)
                    let rgb: (UInt8, UInt8, UInt8)
                    if let sampled = sampler.sampleRGB(worldPoint: world) {
                        rgb = sampled
                        let packed = (UInt32(sampled.0) << 16) | (UInt32(sampled.1) << 8) | UInt32(sampled.2)
                        if vertexColorCache.count < 200_000 || vertexColorCache[key] != nil {
                            vertexColorCache[key] = packed
                        }
                    } else if let cached = vertexColorCache[key] {
                        rgb = (
                            UInt8((cached >> 16) & 0xFF),
                            UInt8((cached >> 8) & 0xFF),
                            UInt8(cached & 0xFF)
                        )
                    } else {
                        rgb = (128, 128, 128)
                    }
                    colors.append(rgb.0)
                    colors.append(rgb.1)
                    colors.append(rgb.2)
                }
            }

            // 2. Faces: 3 vertex indices per triangle (`UInt32` or `UInt16`).
            let faces = geom.faces
            let fBase = faces.buffer.contents()
            let totalIndices = faces.count * faces.indexCountPerPrimitive
            var indices: [UInt32] = []
            indices.reserveCapacity(totalIndices)
            if faces.bytesPerIndex == 4 {
                let ptr = fBase.assumingMemoryBound(to: UInt32.self)
                for i in 0 ..< totalIndices {
                    indices.append(ptr[i])
                }
            } else if faces.bytesPerIndex == 2 {
                let ptr = fBase.assumingMemoryBound(to: UInt16.self)
                for i in 0 ..< totalIndices {
                    indices.append(UInt32(ptr[i]))
                }
            }

            // 3. Per-face classification: 1 UInt8 per triangle (`ARMeshClassification.rawValue`).
            var classes = Data()
            if let cls = geom.classification {
                let cBase = cls.buffer.contents()
                classes.reserveCapacity(cls.count)
                for i in 0 ..< cls.count {
                    let raw = cBase
                        .advanced(by: cls.offset + i * cls.stride)
                        .load(as: UInt8.self)
                    classes.append(raw)
                }
            }

            onMeshChunk(mesh.identifier.uuidString, vertices, indices, classes, colors)
        }
    }

    /// Quantize an ARKit world point into a 5 cm voxel key packed into 63 bits.
    private static func colorVoxelKey(_ p: SIMD3<Float>) -> Int64 {
        let scale: Float = 20.0
        let mask: Int64 = (1 << 21) - 1
        let x = Int64(max(-1_048_575, min(1_048_575, Int(floor(p.x * scale))))) & mask
        let y = Int64(max(-1_048_575, min(1_048_575, Int(floor(p.y * scale))))) & mask
        let z = Int64(max(-1_048_575, min(1_048_575, Int(floor(p.z * scale))))) & mask
        return (x << 42) | (y << 21) | z
    }

    func session(_: ARSession, didFailWithError _: Error) {
        store { $0.vio = nil }
    }

    func sessionWasInterrupted(_: ARSession) {
        store { $0.vio = nil }
    }

    /// Back-project a thinned grid of LiDAR depth pixels into camera-frame
    /// points (ARKit camera axes, metres). Low-confidence pixels are skipped.
    private func depthPoints(from depth: ARDepthData, intrinsics: simd_float3x3, imageSize: CGSize) -> [Vec3] {
        // `depthMap` is a CoreVideo `CVPixelBuffer` (2D grid of Float32 distances in metres).
        // You must lock its base address before reading raw memory, and unlock it via `defer`.
        let map = depth.depthMap
        CVPixelBufferLockBaseAddress(map, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(map, .readOnly) }
        let width = CVPixelBufferGetWidth(map), height = CVPixelBufferGetHeight(map)
        guard let base = CVPixelBufferGetBaseAddress(map) else { return [] }
        // `rowBytes` (stride) can include padding bytes at the end of each row,
        // so always step rows by `v * rowBytes`, not `v * width * 4`.
        let rowBytes = CVPixelBufferGetBytesPerRow(map)

        // Optional per-pixel confidence buffer (UInt8: 0=low, 1=medium, 2=high).
        var confidenceBase: UnsafeMutableRawPointer?
        var confidenceRow = 0
        if let confidence = depth.confidenceMap {
            CVPixelBufferLockBaseAddress(confidence, .readOnly)
            confidenceBase = CVPixelBufferGetBaseAddress(confidence)
            confidenceRow = CVPixelBufferGetBytesPerRow(confidence)
        }
        defer {
            if let c = depth.confidenceMap {
                CVPixelBufferUnlockBaseAddress(c, .readOnly)
            }
        }

        // Pinhole camera intrinsics matrix K (column-major in simd_float3x3):
        //   [ fx  0  cx ]
        //   [  0 fy  cy ]   where (fx, fy) = focal length in px, (cx, cy) = principal point.
        //   [  0  0   1 ]
        // Intrinsics match the full RGB `imageSize`, so scale them down to the LiDAR depth map size.
        let sx = Float(width) / Float(imageSize.width), sy = Float(height) / Float(imageSize.height)
        let fx = intrinsics[0][0] * sx, fy = intrinsics[1][1] * sy
        let cx = intrinsics[2][0] * sx, cy = intrinsics[2][1] * sy
        // Subsample stride so we emit roughly `depthTarget` (~600) evenly spaced points.
        let step = max(1, Int((Float(width * height) / Float(depthTarget)).squareRoot()))

        var points: [Vec3] = []
        points.reserveCapacity(depthTarget)
        for v in stride(from: 0, to: height, by: step) {
            // Byte-offset to row `v`, then view that row as an array of `Float32` depths.
            let row = base.advanced(by: v * rowBytes).assumingMemoryBound(to: Float32.self)
            for u in stride(from: 0, to: width, by: step) {
                if let c = confidenceBase,
                   c.advanced(by: v * confidenceRow + u).load(as: UInt8.self) < UInt8(ARConfidenceLevel.medium.rawValue)
                {
                    continue
                }
                let z = row[u] // depth along the lens axis in metres
                guard z.isFinite, z > 0.1, z < 5 else { continue }
                // Pinhole back-projection into ARKit camera frame (+X right, +Y up, -Z forward):
                // image `v` grows downward, so Y is negated, and forward distance `z` becomes `-z`.
                points.append(Vec3(SIMD3((Float(u) - cx) * z / fx, -(Float(v) - cy) * z / fy, -z)))
            }
        }
        return points
    }

    // MARK: Core Location

    /// Called by Core Location when a new GPS fix arrives.
    /// Negative `horizontalAccuracy` means Apple considers the fix invalid.
    func locationManager(_: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        guard let fix = locations.last, fix.horizontalAccuracy >= 0 else { return }
        store { $0.gps = GpsFix(latitude: fix.coordinate.latitude, longitude: fix.coordinate.longitude,
                                accuracy: fix.horizontalAccuracy) }
    }

    /// Helper to mutate cached sensor fields under `lock`.
    private func store(_ update: (PhoneSensors) -> Void) {
        lock.lock(); update(self); lock.unlock()
    }
}

/// Locks one `ARFrame`'s bi-planar NV12 `capturedImage` (and optional LiDAR
/// `sceneDepth`) to project ARKit world vertices into sRGB colors.
private struct FrameColorSampler {
    private let pixelBuffer: CVPixelBuffer
    private let yBase: UnsafePointer<UInt8>
    private let yWidth: Int
    private let yHeight: Int
    private let yRowBytes: Int
    private let cbcrBase: UnsafePointer<UInt8>
    private let cbcrWidth: Int
    private let cbcrHeight: Int
    private let cbcrRowBytes: Int

    private let depthBuffer: CVPixelBuffer?
    private let depthBase: UnsafeRawPointer?
    private let depthWidth: Int
    private let depthHeight: Int
    private let depthRowBytes: Int

    private let worldToCamera: simd_float4x4
    private let fx: Float
    private let fy: Float
    private let cx: Float
    private let cy: Float

    init?(_ frame: ARFrame) {
        let pb = frame.capturedImage
        guard CVPixelBufferIsPlanar(pb), CVPixelBufferGetPlaneCount(pb) >= 2 else { return nil }
        CVPixelBufferLockBaseAddress(pb, .readOnly)
        guard let yPtr = CVPixelBufferGetBaseAddressOfPlane(pb, 0),
              let cbcrPtr = CVPixelBufferGetBaseAddressOfPlane(pb, 1)
        else {
            CVPixelBufferUnlockBaseAddress(pb, .readOnly)
            return nil
        }

        pixelBuffer = pb
        yBase = UnsafePointer(yPtr.assumingMemoryBound(to: UInt8.self))
        yWidth = CVPixelBufferGetWidthOfPlane(pb, 0)
        yHeight = CVPixelBufferGetHeightOfPlane(pb, 0)
        yRowBytes = CVPixelBufferGetBytesPerRowOfPlane(pb, 0)
        cbcrBase = UnsafePointer(cbcrPtr.assumingMemoryBound(to: UInt8.self))
        cbcrWidth = CVPixelBufferGetWidthOfPlane(pb, 1)
        cbcrHeight = CVPixelBufferGetHeightOfPlane(pb, 1)
        cbcrRowBytes = CVPixelBufferGetBytesPerRowOfPlane(pb, 1)

        if let depthMap = frame.sceneDepth?.depthMap {
            CVPixelBufferLockBaseAddress(depthMap, .readOnly)
            depthBuffer = depthMap
            depthBase = UnsafeRawPointer(CVPixelBufferGetBaseAddress(depthMap))
            depthWidth = CVPixelBufferGetWidth(depthMap)
            depthHeight = CVPixelBufferGetHeight(depthMap)
            depthRowBytes = CVPixelBufferGetBytesPerRow(depthMap)
        } else {
            depthBuffer = nil
            depthBase = nil
            depthWidth = 0
            depthHeight = 0
            depthRowBytes = 0
        }

        worldToCamera = frame.camera.transform.inverse
        let imageSize = frame.camera.imageResolution
        let sx = Float(yWidth) / Float(imageSize.width)
        let sy = Float(yHeight) / Float(imageSize.height)
        let k = frame.camera.intrinsics
        fx = k[0][0] * sx
        fy = k[1][1] * sy
        cx = k[2][0] * sx
        cy = k[2][1] * sy
    }

    func unlock() {
        if let depthBuffer {
            CVPixelBufferUnlockBaseAddress(depthBuffer, .readOnly)
        }
        CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly)
    }

    /// Project a world-space point (ARKit world `A`) into `capturedImage` and return
    /// its sRGB color, or `nil` if behind the camera, outside the frame, or occluded.
    func sampleRGB(worldPoint: SIMD3<Float>) -> (UInt8, UInt8, UInt8)? {
        let cam = worldToCamera * SIMD4<Float>(worldPoint, 1.0)
        let depth = -cam.z
        guard depth > 0.05 else { return nil }

        let u = Int((fx * (cam.x / depth) + cx).rounded())
        let v = Int((fy * (-cam.y / depth) + cy).rounded())
        guard u >= 0, u < yWidth, v >= 0, v < yHeight else { return nil }

        // Reject vertices occluded behind a closer foreground surface in LiDAR depth.
        if let depthBase, depthWidth > 0, depthHeight > 0 {
            let du = min(depthWidth - 1, u * depthWidth / yWidth)
            let dv = min(depthHeight - 1, v * depthHeight / yHeight)
            let zMeasured = depthBase
                .advanced(by: dv * depthRowBytes)
                .assumingMemoryBound(to: Float32.self)[du]
            if zMeasured.isFinite, zMeasured > 0.05, abs(depth - zMeasured) > 0.35 {
                return nil
            }
        }

        let yVal = Float(yBase[v * yRowBytes + u])
        let cu = min(cbcrWidth - 1, u >> 1)
        let cv = min(cbcrHeight - 1, v >> 1)
        let cbcrOffset = cv * cbcrRowBytes + cu * 2
        let cb = Float(cbcrBase[cbcrOffset]) - 128.0
        let cr = Float(cbcrBase[cbcrOffset + 1]) - 128.0

        let r = UInt8(clamping: Int((yVal + 1.402 * cr).rounded()))
        let g = UInt8(clamping: Int((yVal - 0.344136 * cb - 0.714136 * cr).rounded()))
        let b = UInt8(clamping: Int((yVal + 1.772 * cb).rounded()))
        return (r, g, b)
    }
}
