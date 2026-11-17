import ARKit
import CoreLocation
import CoreMotion
import Foundation
import simd

/// Stage 1, Sensors. Reads ARKit, Core Motion, Core Location and LiDAR depth
/// and keeps the latest value of each. No filtering, no frames changed:
/// mount and body-frame work happen in terra-av.
final class PhoneSensors: NSObject, ARSessionDelegate, CLLocationManagerDelegate {
    private let session = ARSession()
    private let motion = CMMotionManager()
    private let location = CLLocationManager()
    private let lock = NSLock()
    private var vio: VioPose?
    private var gyro: SIMD3<Double>?
    private var gps: GpsFix?
    private var depth: [SIMD3<Float>] = []

    /// Keep about this many depth points per frame.
    private let depthTarget = 600

    func start() {
        let config = ARWorldTrackingConfiguration()
        config.worldAlignment = .gravity
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            config.frameSemantics.insert(.sceneDepth)
        }
        session.delegate = self
        session.delegateQueue = DispatchQueue(label: "terra.sensors.ar")
        session.run(config, options: [.resetTracking, .removeExistingAnchors])

        motion.deviceMotionUpdateInterval = 0.01
        motion.startDeviceMotionUpdates(to: OperationQueue()) { [weak self] data, _ in
            guard let r = data?.rotationRate else { return }
            self?.store { $0.gyro = SIMD3(r.x, r.y, r.z) }
        }

        location.delegate = self
        location.desiredAccuracy = kCLLocationAccuracyBest
        location.requestWhenInUseAuthorization()
        location.startUpdatingLocation()
    }

    func stop() {
        session.pause()
        motion.stopDeviceMotionUpdates()
        location.stopUpdatingLocation()
        store { $0.vio = nil; $0.gyro = nil; $0.gps = nil; $0.depth = [] }
    }

    /// The latest reading of every sensor, stamped now.
    func frame(estop: Bool) -> SensorFrame {
        lock.lock(); defer { lock.unlock() }
        return SensorFrame(t: ProcessInfo.processInfo.systemUptime, estop: estop,
                           vio: vio, gyro: gyro, gps: gps, depth: depth)
    }

    // MARK: ARKit

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        let m = frame.camera.transform
        let rotation = simd_double3x3(
            SIMD3(Double(m.columns.0.x), Double(m.columns.0.y), Double(m.columns.0.z)),
            SIMD3(Double(m.columns.1.x), Double(m.columns.1.y), Double(m.columns.1.z)),
            SIMD3(Double(m.columns.2.x), Double(m.columns.2.y), Double(m.columns.2.z)))
        let position = SIMD3(Double(m.columns.3.x), Double(m.columns.3.y), Double(m.columns.3.z))
        var tracked = false
        if case .normal = frame.camera.trackingState { tracked = true }
        let points = frame.sceneDepth.map { depthPoints(from: $0, intrinsics: frame.camera.intrinsics,
                                                         imageSize: frame.camera.imageResolution) } ?? []
        store {
            $0.vio = VioPose(position: position, rotation: rotation, tracked: tracked)
            $0.depth = points
        }
    }

    func session(_ session: ARSession, didFailWithError error: Error) { store { $0.vio = nil } }
    func sessionWasInterrupted(_ session: ARSession) { store { $0.vio = nil } }

    /// Back-project a thinned grid of LiDAR depth pixels into camera-frame
    /// points (ARKit camera axes, metres). Low-confidence pixels are skipped.
    private func depthPoints(from depth: ARDepthData, intrinsics: simd_float3x3, imageSize: CGSize) -> [SIMD3<Float>] {
        let map = depth.depthMap
        CVPixelBufferLockBaseAddress(map, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(map, .readOnly) }
        let width = CVPixelBufferGetWidth(map), height = CVPixelBufferGetHeight(map)
        guard let base = CVPixelBufferGetBaseAddress(map) else { return [] }
        let rowBytes = CVPixelBufferGetBytesPerRow(map)
        var confidenceBase: UnsafeMutableRawPointer?
        var confidenceRow = 0
        if let confidence = depth.confidenceMap {
            CVPixelBufferLockBaseAddress(confidence, .readOnly)
            confidenceBase = CVPixelBufferGetBaseAddress(confidence)
            confidenceRow = CVPixelBufferGetBytesPerRow(confidence)
        }
        defer { if let c = depth.confidenceMap { CVPixelBufferUnlockBaseAddress(c, .readOnly) } }

        // Intrinsics are for the camera image; scale to the depth map.
        let sx = Float(width) / Float(imageSize.width), sy = Float(height) / Float(imageSize.height)
        let fx = intrinsics[0][0] * sx, fy = intrinsics[1][1] * sy
        let cx = intrinsics[2][0] * sx, cy = intrinsics[2][1] * sy
        let step = max(1, Int((Float(width * height) / Float(depthTarget)).squareRoot()))

        var points: [SIMD3<Float>] = []
        points.reserveCapacity(depthTarget)
        for v in stride(from: 0, to: height, by: step) {
            let row = base.advanced(by: v * rowBytes).assumingMemoryBound(to: Float32.self)
            for u in stride(from: 0, to: width, by: step) {
                if let c = confidenceBase,
                   c.advanced(by: v * confidenceRow + u).load(as: UInt8.self) < UInt8(ARConfidenceLevel.medium.rawValue) { continue }
                let z = row[u]
                guard z.isFinite, z > 0.1, z < 5 else { continue }
                // ARKit camera axes: +X right, +Y up, -Z forward (image v grows down).
                points.append(SIMD3((Float(u) - cx) * z / fx, -(Float(v) - cy) * z / fy, -z))
            }
        }
        return points
    }

    // MARK: Core Location

    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        guard let fix = locations.last, fix.horizontalAccuracy >= 0 else { return }
        store { $0.gps = GpsFix(latitude: fix.coordinate.latitude, longitude: fix.coordinate.longitude,
                                accuracy: fix.horizontalAccuracy) }
    }

    private func store(_ update: (PhoneSensors) -> Void) {
        lock.lock(); update(self); lock.unlock()
    }
}
