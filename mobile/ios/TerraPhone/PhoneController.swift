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
    // Phone vector -> rover vector: x_body=y_phone, y_body=-x_phone, z_body=z_phone.
    private let phoneToBody = simd_quatf(angle: -.pi / 2, axis: SIMD3(0, 0, 1))
    // ARKit world +Y up -> robotics world +Z up, preserving right handedness.
    private let worldFromAR = simd_float3x3(columns: (SIMD3(0, -1, 0), SIMD3(0, 0, 1), SIMD3(-1, 0, 0)))
    override init() {
        super.init()
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
        session.run(configuration, options: [.resetTracking, .removeExistingAnchors])
        motion.deviceMotionUpdateInterval = 0.01
        motion.startDeviceMotionUpdates(using: .xArbitraryZVertical, to: motionQueue) { [weak self] data, error in
            guard let self else { return }
            if let error { self.controlQueue.async { self.fail(error) }; return }
            guard let data else { return }
            let acceleration = self.phoneToBody.act(SIMD3(Float(data.userAcceleration.x), Float(data.userAcceleration.y), Float(data.userAcceleration.z))) * 9.80665
            let gyro = self.phoneToBody.act(SIMD3(Float(data.rotationRate.x), Float(data.rotationRate.y), Float(data.rotationRate.z)))
            let sample = ImuReading(timestamp: data.timestamp, accelerationForward: Double(acceleration.x), accelerationLeft: Double(acceleration.y), accelerationUp: Double(acceleration.z), gyroRoll: Double(gyro.x), gyroPitch: Double(gyro.y), gyroYaw: Double(gyro.z))
            self.controlQueue.async {
                guard self.mode == .phone else { return }
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
            try? self.controller?.reset()
            self.controller = nil
            DispatchQueue.main.async {
                self.source = "Stopped"; self.status = "Motor output disabled"
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
                    try controller.pushVio(sample: VioReading(timestamp: now, positionX: 0, positionY: 0, positionZ: 0, quaternionX: 0, quaternionY: 0, quaternionZ: sin(simulatedYaw / 2), quaternionW: cos(simulatedYaw / 2), velocityX: simulatedForward * cos(simulatedYaw), velocityY: simulatedForward * sin(simulatedYaw), velocityZ: 0, tracked: true))
                }
            }
            try controller.setTarget(target: TwistSetpoint(timestamp: now, forward: target.forward, yawRate: target.yaw))
            let output = try controller.step(timestamp: now)
            if mode == .simulation {
                simulatedAcceleration = 3 * (output.leftEffort + output.rightEffort) / 2 - simulatedForward
                simulatedForward += simulatedAcceleration * 0.01
                simulatedYawRate += (5 * (output.rightEffort - output.leftEffort) / 2 - simulatedYawRate) * 0.01
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
        let orientation = simd_quatf(worldFromAR) * simd_quatf(transform) * phoneToBody.inverse
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
        do {
            try controller.pushVio(sample: VioReading(timestamp: frame.timestamp, positionX: Double(position.x), positionY: Double(position.y), positionZ: Double(position.z), quaternionX: Double(q.x), quaternionY: Double(q.y), quaternionZ: Double(q.z), quaternionW: Double(q.w), velocityX: Double(filteredVelocity.x), velocityY: Double(filteredVelocity.y), velocityZ: Double(filteredVelocity.z), tracked: tracked && velocityReady))
        } catch { fail(error) }
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
        }
    }
}
