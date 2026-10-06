import ARKit
import Combine
import CoreMotion
import Foundation
import QuartzCore
import simd

// Sensor input, target updates and Rust ticks are serialized on controlQueue.
// Observable properties are published only on the main queue.
final class PhoneController: NSObject, ObservableObject, ARSessionDelegate, @unchecked Sendable {
    @Published private(set) var source = "Stopped"
    @Published private(set) var status = "Motor output disabled"
    @Published private(set) var measuredForward = 0.0
    @Published private(set) var measuredYaw = 0.0
    @Published private(set) var leftEffort = 0.0
    @Published private(set) var rightEffort = 0.0
    @Published private(set) var benchmark = "Tests feedback control under load and saturation."
    @Published private(set) var occupancy: OccupancyGrid?
    @Published private(set) var mapStatus = "Start Simulated rover to build a map"
    @Published private(set) var mapPose = SIMD3<Double>.zero
    @Published private(set) var calibrationStatus = "Mount calibration required"
    @Published private(set) var offsetFitStatus = "Offset fit not started"
    @Published private(set) var fittedCameraOffset: SIMD2<Double>?
    @Published private(set) var offsetFitInProgress = false
    private(set) var mountCalibration = MountCalibration.load()
    private var occupancyMap: MobileOccupancyMap?
    private var simulatedPosition = SIMD2<Double>.zero
    private var lastMapTime = -Double.infinity
    private enum Mode { case stopped, simulation, phone }
    private let controlQueue = DispatchQueue(label: "terra.control", qos: .userInteractive)
    private let motionQueue = OperationQueue()
    private let motion = CMMotionManager()
    private let session = ARSession()
    private var controller: MobileController?
    private var timer: DispatchSourceTimer?
    private var mode: Mode = .stopped
    private var target = (forward: 0.0, yaw: 0.0)
    private var tick = 0
    private var simulatedTime = 0.0
    private var simulatedForward = 0.0
    private var simulatedYawRate = 0.0
    private var simulatedYaw = 0.0
    private var simulatedAcceleration = 0.0
    private var previousPosition: SIMD3<Float>?
    private var previousPoseTime: Double?
    private var filteredVelocity = SIMD3<Float>.zero
    private var latestAngularVelocityBody = SIMD3<Float>.zero
    private var isCollectingOffsetFit = false
    private var offsetFitSamples: [RotationOffsetSample] = []
    private var firstTurnFit: RotationOffsetFit?
    // ARKit world +Y up -> robotics world +Z up, preserving right handedness.
    private let worldFromAR = simd_float3x3(columns: (SIMD3(0, -1, 0), SIMD3(0, 0, 1), SIMD3(-1, 0, 0)))
    override init() {
        super.init()
        calibrationStatus = mountCalibration.isPlausible
            ? (mountCalibration.reviewed ? "Measured calibration saved · reviewed" : "Measured calibration saved · review required")
            : "Mount calibration required"
        motionQueue.maxConcurrentOperationCount = 1
        motionQueue.qualityOfService = .userInteractive
        session.delegate = self
        session.delegateQueue = controlQueue
    }
    func setTarget(forward: Double, yaw: Double) {
        controlQueue.async { self.target = (forward, yaw) }
    }
    func startSimulation() {
        stop()
        controlQueue.async {
            do {
                try self.prepare(.simulation)
                self.startTimer()
                DispatchQueue.main.async { self.source = "Simulated motor plant" }
            } catch { self.fail(error) }
        }
    }
    func startPhone() {
        stop()
        guard ARWorldTrackingConfiguration.isSupported, motion.isDeviceMotionAvailable else {
            status = "Phone tracking is unavailable; use Simulated rover."
            return
        }
        controlQueue.async {
            do { try self.prepare(.phone); self.startTimer() }
            catch { self.fail(error) }
        }
        let configuration = ARWorldTrackingConfiguration()
        configuration.worldAlignment = .gravity
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) { configuration.frameSemantics.insert(.sceneDepth) }
        session.run(configuration, options: [.resetTracking, .removeExistingAnchors])
        motion.deviceMotionUpdateInterval = 0.01
        motion.startDeviceMotionUpdates(using: .xArbitraryZVertical, to: motionQueue) { [weak self] data, error in
            guard let self else { return }
            if let error { self.controlQueue.async { self.fail(error) }; return }
            guard let data else { return }
            let phoneToBody = self.mountCalibration.deviceToBody
            let acceleration = phoneToBody.act(SIMD3(Float(data.userAcceleration.x), Float(data.userAcceleration.y), Float(data.userAcceleration.z))) * 9.80665
            let gyro = phoneToBody.act(SIMD3(Float(data.rotationRate.x), Float(data.rotationRate.y), Float(data.rotationRate.z)))
            let sample = ImuReading(timestamp: data.timestamp, accelerationForward: Double(acceleration.x), accelerationLeft: Double(acceleration.y), accelerationUp: Double(acceleration.z), gyroRoll: Double(gyro.x), gyroPitch: Double(gyro.y), gyroYaw: Double(gyro.z))
            self.controlQueue.async {
                guard self.mode == .phone else { return }
                self.latestAngularVelocityBody = SIMD3(Float(sample.gyroRoll), Float(sample.gyroPitch), Float(sample.gyroYaw))
                do { try self.controller?.pushImu(sample: sample) } catch { self.fail(error) }
            }
        }
        source = "Core Motion + ARKit"
    }
    func stop() {
        motion.stopDeviceMotionUpdates()
        session.pause()
        controlQueue.async {
            self.timer?.cancel(); self.timer = nil
            self.mode = .stopped
            self.isCollectingOffsetFit = false
            self.offsetFitSamples.removeAll(keepingCapacity: true)
            self.firstTurnFit = nil
            try? self.controller?.reset()
            self.controller = nil
            DispatchQueue.main.async {
                self.source = "Stopped"; self.status = "Motor output disabled"
                self.offsetFitInProgress = false
                self.mapStatus = "Map paused"
                self.leftEffort = 0; self.rightEffort = 0
                self.measuredForward = 0; self.measuredYaw = 0
            }
        }
    }
    func runBenchmark() {
        controlQueue.async {
            do {
                let result = try runVelocityBenchmark()
                DispatchQueue.main.async {
                    self.benchmark = String(format: "%@ · speed error %.3f m/s · turn error %.3f rad/s", result.passed ? "Passed" : "Failed", result.forwardError, result.yawError)
                }
            } catch { self.fail(error) }
        }
    }
    private func prepare(_ mode: Mode) throws {
        self.controller = try MobileController(settings: defaultControlSettings())
        self.occupancyMap = try MobileOccupancyMap(settings: defaultOccupancySettings())
        simulatedPosition = .zero; lastMapTime = -Double.infinity
        DispatchQueue.main.async { self.occupancy = nil; self.mapPose = .zero; self.mapStatus = mode == .simulation ? "Simulated depth · 10 Hz" : (ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) ? "Waiting for scene depth" : "Scene depth unavailable on this device") }
        self.mode = mode
        tick = 0; simulatedTime = 0; simulatedForward = 0; simulatedYawRate = 0; simulatedYaw = 0; simulatedAcceleration = 0
        previousPosition = nil; previousPoseTime = nil; filteredVelocity = .zero
    }
    private func startTimer() {
        let timer = DispatchSource.makeTimerSource(queue: controlQueue)
        timer.schedule(deadline: .now(), repeating: .milliseconds(10), leeway: .milliseconds(1))
        timer.setEventHandler { [weak self] in self?.update() }
        self.timer = timer
        timer.resume()
    }
    private func update() {
        guard let controller, mode != .stopped else { return }
        let now = mode == .simulation ? simulatedTime : CACurrentMediaTime()
        do {
            if mode == .simulation {
                try controller.pushImu(sample: ImuReading(timestamp: now, accelerationForward: simulatedAcceleration, accelerationLeft: simulatedForward * simulatedYawRate, accelerationUp: 0, gyroRoll: 0, gyroPitch: 0, gyroYaw: simulatedYawRate))
                if tick % 5 == 0 {
                    try controller.pushVio(sample: VioReading(timestamp: now, positionX: simulatedPosition.x, positionY: simulatedPosition.y, positionZ: 0, quaternionX: 0, quaternionY: 0, quaternionZ: sin(simulatedYaw / 2), quaternionW: cos(simulatedYaw / 2), velocityX: simulatedForward * cos(simulatedYaw), velocityY: simulatedForward * sin(simulatedYaw), velocityZ: 0, tracked: true))
                }
            }
            try controller.setTarget(target: TwistSetpoint(timestamp: now, forward: target.forward, yawRate: target.yaw))
            let output = try controller.step(timestamp: now)
            if mode == .simulation {
                simulatedAcceleration = 3 * (output.leftEffort + output.rightEffort) / 2 - simulatedForward
                simulatedForward += simulatedAcceleration * 0.01
                simulatedYawRate += (5 * (output.rightEffort - output.leftEffort) / 2 - simulatedYawRate) * 0.01
                if tick % 10 == 0 { try updateSimulatedMap(timestamp: now) }
                simulatedPosition += SIMD2(cos(simulatedYaw), sin(simulatedYaw)) * simulatedForward * 0.01
                simulatedYaw += simulatedYawRate * 0.01
                simulatedTime += 0.01
            }
            tick += 1
            if tick % 10 == 0 { publish(output) }
        } catch { fail(error) }
    }
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        guard mode == .phone, let controller else { return }
        let tracked: Bool
        if case .normal = frame.camera.trackingState { tracked = true } else { tracked = false }
        let transform = frame.camera.transform
        let arPosition = SIMD3(transform.columns.3.x, transform.columns.3.y, transform.columns.3.z)
        let position = worldFromAR * arPosition
        let cameraOpticalToWorld = simd_quatf(worldFromAR) * simd_quatf(transform) * simd_quatf(angle: .pi, axis: SIMD3(1, 0, 0))
        let orientation = cameraOpticalToWorld * mountCalibration.opticalCameraToBody.inverse
        if isCollectingOffsetFit {
            if !tracked {
                isCollectingOffsetFit = false
                offsetFitSamples.removeAll(keepingCapacity: true)
                DispatchQueue.main.async {
                    self.offsetFitInProgress = false
                    self.offsetFitStatus = "Fit cancelled because ARKit tracking was interrupted"
                }
            } else {
                let roverForward = orientation.act(SIMD3<Float>(1, 0, 0))
                offsetFitSamples.append(RotationOffsetSample(
                    timestamp: frame.timestamp,
                    cameraPositionWorld: SIMD2(Double(position.x), Double(position.y)),
                    roverHeadingWorld: atan2(Double(roverForward.y), Double(roverForward.x))
                ))
                if offsetFitSamples.count > 12000 {
                    isCollectingOffsetFit = false
                    offsetFitSamples.removeAll(keepingCapacity: true)
                    DispatchQueue.main.async {
                        self.offsetFitInProgress = false
                        self.offsetFitStatus = "Fit cancelled: turn exceeded the capture duration. Start a new pass."
                    }
                }
            }
        }
        var velocityReady = false
        if tracked, let last = previousPosition, let previousTime = previousPoseTime {
            let dt = frame.timestamp - previousTime
            if dt > 0 && dt < 0.2 {
                let raw = (position - last) / Float(dt)
                let alpha = Float(dt / (0.03 + dt))
                filteredVelocity += alpha * (raw - filteredVelocity)
                velocityReady = true
            }
        }
        previousPosition = tracked ? position : nil
        previousPoseTime = tracked ? frame.timestamp : nil
        if !tracked { filteredVelocity = .zero }
        let q = orientation.vector
        // The configured rover origin is the ground projection below the drive-axle midpoint.
        let cameraOffsetBody = SIMD3(Float(mountCalibration.cameraOffsetForward), Float(mountCalibration.cameraOffsetLeft), Float(mountCalibration.cameraHeight))
        let roverPosition = position - orientation.act(cameraOffsetBody)
        let cameraVelocityBody = orientation.inverse.act(filteredVelocity)
        let roverVelocityWorld = orientation.act(cameraVelocityBody - latestAngularVelocityBody.cross(cameraOffsetBody))
        do {
            try controller.pushVio(sample: VioReading(timestamp: frame.timestamp, positionX: Double(roverPosition.x), positionY: Double(roverPosition.y), positionZ: Double(roverPosition.z), quaternionX: Double(q.x), quaternionY: Double(q.y), quaternionZ: Double(q.z), quaternionW: Double(q.w), velocityX: Double(roverVelocityWorld.x), velocityY: Double(roverVelocityWorld.y), velocityZ: Double(roverVelocityWorld.z), tracked: tracked && velocityReady && mountCalibration.isPlausible))
            if tracked { updatePhoneMap(frame: frame, position: position, roverPosition: roverPosition, bodyOrientation: orientation) }
            else { DispatchQueue.main.async { self.mapStatus = "Tracking lost · map paused" } }
        } catch { fail(error) }
    }
    func clearMap() {
        controlQueue.async {
            do { try self.occupancyMap?.clear(); self.lastMapTime = -Double.infinity
                DispatchQueue.main.async { self.occupancy = nil } }
            catch { DispatchQueue.main.async { self.mapStatus = error.localizedDescription } }
        }
    }
    func saveMountCalibration(_ value: MountCalibration) {
        controlQueue.async {
            do {
                guard value.isPlausible else { throw CalibrationSaveError.invalidValues }
                try value.save()
                self.mountCalibration = value
                DispatchQueue.main.async {
                    self.calibrationStatus = value.reviewed ? "Measured calibration saved · reviewed" : "Measured calibration saved · review required"
                }
            } catch {
                DispatchQueue.main.async { self.calibrationStatus = "Calibration not saved: \(error.localizedDescription)" }
            }
        }
    }
    func beginAutomaticOffsetFit() {
        controlQueue.async {
            guard !self.isCollectingOffsetFit else { return }
            guard self.mode == .phone else {
                DispatchQueue.main.async { self.offsetFitStatus = "Start Phone IMU + VIO before fitting the offset" }
                return
            }
            guard self.mountCalibration.cameraHeight > 0 else {
                DispatchQueue.main.async { self.offsetFitStatus = "Enter and save camera height before fitting" }
                return
            }
            if self.firstTurnFit == nil {
                DispatchQueue.main.async { self.fittedCameraOffset = nil }
            }
            self.offsetFitSamples.removeAll(keepingCapacity: true)
            self.isCollectingOffsetFit = true
            DispatchQueue.main.async {
                self.offsetFitInProgress = true
                self.offsetFitStatus = "Turn the rover at least 90° in one direction around its marked reference point"
            }
        }
    }
    func finishAutomaticOffsetFit() {
        controlQueue.async {
            guard self.isCollectingOffsetFit else { return }
            self.isCollectingOffsetFit = false
            DispatchQueue.main.async { self.offsetFitInProgress = false }
            let samples = self.offsetFitSamples
            self.offsetFitSamples.removeAll(keepingCapacity: true)
            guard let fit = RotationOffsetFitter.fit(samples) else {
                DispatchQueue.main.async {
                    self.offsetFitInProgress = false
                    self.offsetFitStatus = "Fit rejected: need stable tracking, ≥90° turn, and ≤5 cm path error. Retry slowly."
                }
                return
            }
            if let first = self.firstTurnFit {
                guard first.turnDirection != fit.turnDirection,
                      hypot(first.forward - fit.forward, first.left - fit.left) <= 0.05 else {
                    self.firstTurnFit = nil
                    DispatchQueue.main.async {
                        self.offsetFitInProgress = false
                        self.offsetFitStatus = "Reverse-turn result disagreed by over 5 cm. Recheck the pivot and tracking, then repeat both turns."
                    }
                    return
                }
                self.firstTurnFit = nil
                let forward = (first.forward + fit.forward) * 0.5
                let left = (first.left + fit.left) * 0.5
                DispatchQueue.main.async {
                    self.fittedCameraOffset = SIMD2(forward, left)
                    self.offsetFitStatus = String(format: "Two-way fit accepted · forward %+.3f m · left %+.3f m · RMS %.1f / %.1f cm",
                                                  forward, left, first.rmsError * 100, fit.rmsError * 100)
                }
            } else {
                self.firstTurnFit = fit
                DispatchQueue.main.async {
                    self.offsetFitStatus = String(format: "First pass accepted · %.1f cm RMS. Now start and perform a turn in the opposite direction.", fit.rmsError * 100)
                }
            }
        }
    }
    private func publishMap(x: Double, y: Double, yaw: Double, status: String) throws {
        guard let grid = try occupancyMap?.snapshot() else { return }
        DispatchQueue.main.async { self.occupancy = grid; self.mapPose = SIMD3(x, y, yaw); self.mapStatus = status }
    }
    private func updateSimulatedMap(timestamp: Double) throws {
        guard let map = occupancyMap else { return }
        let x = simulatedPosition.x, y = simulatedPosition.y
        try map.recenter(x: x, y: y)
        // Room walls at ±6 m, optical camera facing along rover heading.
        var depths = [Float](repeating: .nan, count: 81)
        for u in depths.indices {
            let right = (Double(u) - 40) / 60
            let dx = cos(simulatedYaw) + right * sin(simulatedYaw)
            let dy = sin(simulatedYaw) - right * cos(simulatedYaw)
            var nearest = Double.infinity
            for wall in [-6.0, 6.0] {
                if abs(dx) > 1e-9 { let t = (wall - x) / dx
                    if t > 0 && abs(y + t * dy) <= 6 { nearest = min(nearest, t) } }
                if abs(dy) > 1e-9 { let t = (wall - y) / dy
                    if t > 0 && abs(x + t * dx) <= 6 { nearest = min(nearest, t) } }
            }
            if nearest.isFinite { depths[u] = Float(nearest) }
        }
        let opticalToBody = simd_quatd(ix: -0.5, iy: 0.5, iz: -0.5, r: 0.5)
        let q = (simd_quatd(angle: simulatedYaw, axis: SIMD3(0, 0, 1)) * opticalToBody).vector
        try map.integrateDepth(frame: MappingDepthFrame(timestamp: timestamp, width: 81, height: 1, fx: 60, fy: 60, cx: 40, cy: 0, cameraX: x, cameraY: y, cameraZ: 0.5, quaternionX: q.x, quaternionY: q.y, quaternionZ: q.z, quaternionW: q.w, depthMetres: depths))
        try publishMap(x: x, y: y, yaw: simulatedYaw, status: "Simulated depth · 10 Hz")
    }
    private func updatePhoneMap(frame: ARFrame, position: SIMD3<Float>, roverPosition: SIMD3<Float>, bodyOrientation: simd_quatf) {
        guard frame.timestamp - lastMapTime >= 0.1, let depth = frame.sceneDepth, let map = occupancyMap else { return }
        let buffer = depth.depthMap
        guard CVPixelBufferGetPixelFormatType(buffer) == kCVPixelFormatType_DepthFloat32,
              CVPixelBufferLockBaseAddress(buffer, .readOnly) == kCVReturnSuccess else { return }
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
        // Use the measured camera height to establish the local ground plane.
        guard mountCalibration.isPlausible else {
            DispatchQueue.main.async { self.mapStatus = "Enter a valid measured camera height to map" }
            return
        }
        do {
            try map.recenter(x: Double(roverPosition.x), y: Double(roverPosition.y))
            try map.integrateDepth(frame: MappingDepthFrame(timestamp: frame.timestamp, width: UInt32(width), height: UInt32(height), fx: Double(k.columns.0.x) * sx, fy: Double(k.columns.1.y) * sy, cx: (Double(k.columns.2.x) + 0.5) * sx - 0.5, cy: (Double(k.columns.2.y) + 0.5) * sy - 0.5, cameraX: Double(position.x), cameraY: Double(position.y), cameraZ: mountCalibration.cameraHeight, quaternionX: Double(q.x), quaternionY: Double(q.y), quaternionZ: Double(q.z), quaternionW: Double(q.w), depthMetres: values))
            lastMapTime = frame.timestamp
            let forward = bodyOrientation.act(SIMD3<Float>(1, 0, 0))
            try publishMap(x: Double(roverPosition.x), y: Double(roverPosition.y), yaw: atan2(Double(forward.y), Double(forward.x)), status: "ARKit scene depth · 10 Hz")
        } catch { DispatchQueue.main.async { self.mapStatus = "Map: " + error.localizedDescription } }
    }
    func session(_ session: ARSession, didFailWithError error: Error) { fail(error) }
    func sessionWasInterrupted(_ session: ARSession) { fail(NSError(domain: "Terra", code: 1, userInfo: [NSLocalizedDescriptionKey: "AR session interrupted"])) }
    private func publish(_ output: ControlOutput) {
        DispatchQueue.main.async {
            self.measuredForward = output.estimatedForward; self.measuredYaw = output.estimatedYawRate
            self.leftEffort = output.leftEffort; self.rightEffort = output.rightEffort
            switch output.safety {
            case .active: self.status = "Feedback control active"
            case .sensorNotReady: self.status = "Waiting for IMU and VIO"
            case .trackingLost: self.status = "Tracking lost · neutral output"
            case .staleSensors: self.status = "Stale sensors · neutral output"
            case .staleTarget: self.status = "Target expired · neutral output"
            case .invalidTime: self.status = "Clock mismatch · neutral output"
            }
        }
    }
    private func fail(_ error: Error) {
        mode = .stopped; timer?.cancel(); timer = nil; try? controller?.reset()
        DispatchQueue.main.async {
            self.motion.stopDeviceMotionUpdates(); self.session.pause()
            self.status = error.localizedDescription; self.leftEffort = 0; self.rightEffort = 0
            self.mapStatus = "Map paused"
        }
    }
}

private enum CalibrationSaveError: LocalizedError {
    case invalidValues
    var errorDescription: String? { "Check the measured values and camera height." }
}
