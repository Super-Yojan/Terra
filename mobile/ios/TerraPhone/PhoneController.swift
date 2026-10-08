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
    private var occupancyMap: MobileOccupancyMap?
    private var simulatedPosition = SIMD2<Double>.zero
    private var mapGroundOffset: Double?
    private var lastMapTime = -Double.infinity
    @Published private(set) var zenohStatus = "Disconnected"
    @Published private(set) var zenohConnecting = false
    @Published private(set) var waypointActive = false
    @Published private(set) var waypointStatus = "No goal"
    @Published private(set) var waypointDistance = 0.0
    private var zenoh: MobileZenohClient?
    @Published private(set) var autonomyLevel="teleop"
    @Published private(set) var autonomyReason="Idle"
    @Published private(set) var explorationSummary=""
    @Published var runLog:URL?
    @Published private(set) var proposedGoal:UInt64?
    private var proposalRun=""
    @Published private(set) var proposalText=""
    private var goalLatched = false
    private var remotePose: (x: Double, y: Double, yaw: Double)?
    private var phonePose: (x: Double, y: Double, yaw: Double)?
    private enum Mode { case stopped, simulation, phone, remote, bluetoothManual }
    let bluetooth = BluetoothLink()
    @Published private(set) var hardwareActive = false
    @Published private(set) var hardwareFeedback = false
    @Published private(set) var hardwareReady = false
    @Published private(set) var hardwareConfigurationReady = false
    @Published private(set) var activeLayoutRevision: UInt32 = 0
    @Published private(set) var hardwareFault: String?
    private var autoConnectionGeneration: UInt64 = 0
    private var autoConnectionPending = false
    private var configurationGeneration: UInt64 = 0
    private var resetFaultRequest: UInt32?
    @Published private(set) var hardwareArmed = false
    @Published private(set) var hardwareArming = false
    @Published private(set) var hardwareBenchMode = false
    @Published private(set) var hardwareBenchEnabled = false
    @Published private(set) var hardwareStatus = "Disconnected"
    @Published private(set) var capabilitiesJSON = "{}"
    @Published private(set) var committedLayoutJSON = "{}"
    @Published private(set) var configurationStatus = "No staged layout"
    @Published private(set) var discoveredRovers: [RoverPeripheral] = []
    @Published private(set) var servoPositions: [UInt8: Double] = [:]
    @Published private(set) var feedbackCompatible = false
    @Published private(set) var acknowledgedCommitRevision: UInt32?
    private var requestedHardwareFeedback: UUID?
    private var feedbackConnectionSawUnready = false
    private var subscriptions = Set<AnyCancellable>()
    private var bleActive = false
    private var bleFeedback = false
    private var bleLayout = "{}"
    private var servoTargets: [String: Double] = [:]
    private var staged: (request: UInt32, revision: UInt32)?
    private var stageRequest: UInt32?
    private var commitRequest: UInt32?
    private var nextRequest: UInt32 = max(1000, UInt32(clamping: UserDefaults.standard.integer(forKey: "hardwareConfigurationRequestID")))
    private var sensorEpoch = 0
    private let sensorEpochLock = NSLock()
    private var hardwareMotionRevoked = true
    private var feedbackTrackingHealthy = false
    private var bleCompatible = false
    private var phoneStartedAt = 0.0
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
        observeBluetooth()
    }
    func setTarget(forward: Double, yaw: Double) {
        controlQueue.async { self.target = (forward, yaw) }
    }
    func engageWaypoint(originLatitude: Double, originLongitude: Double, latitude: Double, longitude: Double, token: String, halfExtent: Double) {
        let token = token.trimmingCharacters(in: .whitespacesAndNewlines)
        controlQueue.async {
            guard self.mode != .stopped, !self.bleActive || self.bleFeedback else {
                DispatchQueue.main.async {
                    #if targetEnvironment(simulator)
                    self.waypointStatus = "Start a rover or connect to Bevy, then go"
                    #else
                    self.waypointStatus = "Start a rover or connect over Bluetooth, then go"
                    #endif
                }
                return
            }
            do {
                let now=self.mode == .simulation ? self.simulatedTime:CACurrentMediaTime()
                try self.controller?.autonomyOrigin(latitude:originLatitude,longitude:originLongitude)
                let values:[String:Any]=["frame":"wgs84","latitude":latitude,"longitude":longitude,"token":token.isEmpty ? UUID().uuidString:token]
                let payload=String(decoding:try JSONSerialization.data(withJSONObject:values),as:UTF8.self)
                if self.mode == .remote {try self.zenoh?.sendAction(kind:"goal",payload:payload)}else{try self.controller?.autonomyRequest(kind:"goal",payload:payload,timestamp:now)}
                let accepted=true
                self.goalLatched = accepted
                DispatchQueue.main.async {
                    self.waypointActive = accepted
                    self.waypointDistance = 0
                    self.waypointStatus = accepted ? "Goal latched · waiting for a pose" : "That point is outside \(Int(halfExtent)) m of the origin"
                }
            } catch {
                self.goalLatched = false
                DispatchQueue.main.async { self.waypointActive = false; self.waypointStatus = error.localizedDescription }
            }
        }
    }
    func cancelWaypoint() {
        controlQueue.async {
            let now=self.mode == .simulation ? self.simulatedTime:CACurrentMediaTime()
            if self.mode == .remote {try? self.zenoh?.sendAction(kind:"goal",payload:"{\"cancel\":true}")}else{try? self.controller?.autonomyRequest(kind:"goal",payload:"{\"cancel\":true}",timestamp:now)}
            self.goalLatched = false
            DispatchQueue.main.async {
                self.waypointActive = false
                self.waypointDistance = 0
                self.waypointStatus = "Teleop"
            }
        }
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
    func startBevy(endpoint: String, roverID: String) {
        // Device builds have no Bevy connection UI. Ignore the call so a saved
        // endpoint cannot open Zenoh; Bluetooth stays the link.
        #if targetEnvironment(simulator)
        connectBevySimulator(endpoint: endpoint, roverID: roverID)
        #else
        _ = (endpoint, roverID)
        #endif
    }
    #if targetEnvironment(simulator)
    private func connectBevySimulator(endpoint: String, roverID: String) {
        guard !zenohConnecting else { return }
        guard let id = UInt64(roverID.trimmingCharacters(in: .whitespacesAndNewlines)) else {
            zenohStatus = "Rover ID must be a nonnegative integer"; return
        }
        stop()
        zenohConnecting = true; zenohStatus = "Connecting…"
        let endpoint = endpoint.trimmingCharacters(in: .whitespacesAndNewlines)
        controlQueue.async {
            do {
                self.occupancyMap = try MobileOccupancyMap(settings: defaultOccupancySettings())
                self.simulatedPosition = .zero
                self.mapGroundOffset = nil
                self.lastMapTime = -Double.infinity
                self.zenoh = try MobileZenohClient(endpoint: endpoint, prefix: "terra/rover", roverId: id)
                self.target = (0, 0); self.tick = 0; self.mode = .remote
                self.startTimer()
                DispatchQueue.main.async {
                    self.zenohConnecting = false
                    self.source = "Bevy rover \(id) over Zenoh"
                    self.status = "Remote velocity targets · local feedback unavailable"
                    self.zenohStatus = "Zenoh session open · waiting for depth"
                    self.occupancy = nil
                    self.mapPose = .zero
                    self.mapStatus = "Waiting for simulator depth and exposure pose"
                }
            } catch { self.fail(error) }
        }
    }
    #endif
    func startPhone() {
        stop()
        startPhoneSensors()
    }
    private func startPhoneSensors() {
        sensorEpochLock.lock(); sensorEpoch += 1; let epoch = sensorEpoch; sensorEpochLock.unlock()
        guard ARWorldTrackingConfiguration.isSupported, motion.isDeviceMotionAvailable else {
            status = "Phone tracking is unavailable; use Simulated rover."
            return
        }
        controlQueue.async {
            guard self.sensorEpochMatches(epoch) else { return }
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
            if let error { self.controlQueue.async { if self.sensorEpochMatches(epoch) { self.fail(error) } }; return }
            guard let data else { return }
            let acceleration = self.phoneToBody.act(SIMD3(Float(data.userAcceleration.x), Float(data.userAcceleration.y), Float(data.userAcceleration.z))) * 9.80665
            let gyro = self.phoneToBody.act(SIMD3(Float(data.rotationRate.x), Float(data.rotationRate.y), Float(data.rotationRate.z)))
            let sample = ImuReading(timestamp: data.timestamp, accelerationForward: Double(acceleration.x), accelerationLeft: Double(acceleration.y), accelerationUp: Double(acceleration.z), gyroRoll: Double(gyro.x), gyroPitch: Double(gyro.y), gyroYaw: Double(gyro.z))
            self.controlQueue.async {
                guard self.mode == .phone, self.sensorEpochMatches(epoch) else { return }
                do { try self.controller?.pushImu(sample: sample) } catch { self.fail(error) }
            }
        }
        source = "Core Motion + ARKit"
    }
    /// keepBluetooth leaves the rover link up for the actuator layout screen,
    /// which is pushed from this root and would otherwise drop on navigation.
    func stop(keepBluetooth: Bool = false) {
        autoConnectionGeneration &+= 1; autoConnectionPending = false
        bluetooth.cancelAutomaticConnection()
        sensorEpochLock.lock(); sensorEpoch += 1; sensorEpochLock.unlock()
        bluetooth.disarm()
        if !keepBluetooth { bluetooth.disconnect() }
        hardwareActive = false; requestedHardwareFeedback = nil
        motion.stopDeviceMotionUpdates()
        session.pause()
        controlQueue.async {
            self.timer?.cancel(); self.timer = nil
            self.mode = .stopped
            self.bleActive = false; self.bleFeedback = false; self.hardwareMotionRevoked = true; self.feedbackTrackingHealthy = false; self.target = (0, 0); self.servoTargets = [:]
            self.zenoh?.disconnect(); self.zenoh = nil
            self.goalLatched = false
            self.remotePose = nil; self.phonePose = nil
            try? self.controller?.reset()
            self.controller = nil
            DispatchQueue.main.async {
                self.source = "Stopped"; self.status = "Motor output disabled"
                self.mapStatus = "Map paused"
                self.waypointActive = false; self.waypointDistance = 0; self.waypointStatus = "No goal"
                self.zenohStatus = "Disconnected"
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
                let log=FileManager.default.urls(for:.documentDirectory,in:.userDomainMask)[0].appendingPathComponent("Terra-run-\(UUID().uuidString).jsonl")
                try self.controller?.beginRecording(path:log.path,runId:UUID().uuidString)
                DispatchQueue.main.async {self.runLog=log}
        self.occupancyMap = try MobileOccupancyMap(settings: defaultOccupancySettings())
        simulatedPosition = .zero; mapGroundOffset = nil; lastMapTime = -Double.infinity
        DispatchQueue.main.async { self.occupancy = nil; self.mapPose = .zero; self.mapStatus = mode == .simulation ? "Simulated depth · 10 Hz" : (ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) ? "Waiting for scene depth" : "Scene depth unavailable on this device") }
        self.mode = mode
        phoneStartedAt = ProcessInfo.processInfo.systemUptime
        target = (0, 0)
        tick = 0; simulatedTime = 0; simulatedForward = 0; simulatedYawRate = 0; simulatedYaw = 0; simulatedAcceleration = 0
        previousPosition = nil; previousPoseTime = nil; filteredVelocity = .zero
    }
    private func startTimer() {
        self.timer?.cancel(); self.timer = nil
        let timer = DispatchSource.makeTimerSource(queue: controlQueue)
        timer.schedule(deadline: .now(), repeating: .milliseconds(10), leeway: .milliseconds(1))
        timer.setEventHandler { [weak self] in self?.update() }
        self.timer = timer
        timer.resume()
    }
    private func update() {
        if mode == .bluetoothManual {
            tick += 1
            if tick % 5 == 0 {
                let left = target.forward - target.yaw, right = target.forward + target.yaw
                let scale = max(1, max(abs(left), abs(right)))
                routeHardware(left: left / scale, right: right / scale, now: ProcessInfo.processInfo.systemUptime)
            }
            return
        }
        if mode == .remote, let zenoh {
            do {
                if let frame = zenoh.takeDepth() {
                    remotePose = (frame.bodyX, frame.bodyY, frame.bodyYaw)
                    self.integrateRemoteDepth(frame)
                }
                let command = target
                try zenoh.setTarget(linear: command.forward, angular: command.yaw)
                tick += 1
                if tick % 10 == 0 { updateAuthority();let status = zenoh.status(); DispatchQueue.main.async { self.zenohStatus = status } }
            } catch { fail(error) }
            return
        }
        guard let controller, mode != .stopped else { return }
        let now = mode == .simulation ? simulatedTime : CACurrentMediaTime()
        do {
            if mode == .simulation {
                try controller.pushImu(sample: ImuReading(timestamp: now, accelerationForward: simulatedAcceleration, accelerationLeft: simulatedForward * simulatedYawRate, accelerationUp: 0, gyroRoll: 0, gyroPitch: 0, gyroYaw: simulatedYawRate))
                if tick % 5 == 0 {
                    try controller.pushVio(sample: VioReading(timestamp: now, positionX: simulatedPosition.x, positionY: simulatedPosition.y, positionZ: 0, quaternionX: 0, quaternionY: 0, quaternionZ: sin(simulatedYaw / 2), quaternionW: cos(simulatedYaw / 2), velocityX: simulatedForward * cos(simulatedYaw), velocityY: simulatedForward * sin(simulatedYaw), velocityZ: 0, tracked: true))
                }
            }
            try controller.setTarget(target: TwistSetpoint(timestamp:now,forward:target.forward,yawRate:target.yaw))
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
            if bleActive && bleFeedback {
                feedbackTrackingHealthy = output.safety == .active
                if !feedbackTrackingHealthy { revokeHardwareMotion() }
                if tick % 5 == 0 { routeHardware(left: output.leftEffort, right: output.rightEffort, now: ProcessInfo.processInfo.systemUptime) }
            }
            if tick % 10 == 0 { publish(output);updateAuthority() }
        } catch { fail(error) }
    }
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        guard mode == .phone, frame.timestamp >= phoneStartedAt, let controller else { return }
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
        if !tracked { filteredVelocity = .zero; if bleActive { feedbackTrackingHealthy = false; revokeHardwareMotion() } }
        let q = orientation.vector
        let bodyForward = orientation.act(SIMD3<Float>(1, 0, 0))
        phonePose = (Double(position.x), Double(position.y), atan2(Double(bodyForward.y), Double(bodyForward.x)))
        do {
            try controller.pushVio(sample: VioReading(timestamp: frame.timestamp, positionX: Double(position.x), positionY: Double(position.y), positionZ: Double(position.z), quaternionX: Double(q.x), quaternionY: Double(q.y), quaternionZ: Double(q.z), quaternionW: Double(q.w), velocityX: Double(filteredVelocity.x), velocityY: Double(filteredVelocity.y), velocityZ: Double(filteredVelocity.z), tracked: tracked && velocityReady))
            if tracked { updatePhoneMap(frame: frame, position: position, bodyOrientation: orientation) }
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
    private func publishMap(x: Double, y: Double, yaw: Double, status: String) throws {
        guard let grid = try occupancyMap?.snapshot() else { return }
        if mode != .remote {try controller?.autonomyMap(grid:grid,timestamp:lastMapTime.isFinite ? lastMapTime:simulatedTime,revision:UInt64(tick))}
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
    private func integrateRemoteDepth(_ frame: MobileDepthFrame) {
        guard let map = occupancyMap else { return }
        do {
            try map.recenter(x: frame.bodyX, y: frame.bodyY)
            try map.integrateDepth(frame: MappingDepthFrame(timestamp: frame.timestamp, width: frame.width, height: frame.height, fx: frame.fx, fy: frame.fy, cx: frame.cx, cy: frame.cy, cameraX: frame.cameraX, cameraY: frame.cameraY, cameraZ: frame.cameraZ, quaternionX: frame.quaternionX, quaternionY: frame.quaternionY, quaternionZ: frame.quaternionZ, quaternionW: frame.quaternionW, depthMetres: frame.depthMetres))
            try publishMap(x: frame.bodyX, y: frame.bodyY, yaw: frame.bodyYaw, status: "Simulator depth · exposure pose")
        } catch {
            DispatchQueue.main.async { self.mapStatus = "Map: " + error.localizedDescription }
        }
    }
    private func updatePhoneMap(frame: ARFrame, position: SIMD3<Float>, bodyOrientation: simd_quatf) {
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
        // Initial camera height is assumed 0.5 m; calibrate against the ground before physical use.
        if mapGroundOffset == nil { mapGroundOffset = 0.5 - Double(position.z) }
        do {
            try map.recenter(x: Double(position.x), y: Double(position.y))
            try map.integrateDepth(frame: MappingDepthFrame(timestamp: frame.timestamp, width: UInt32(width), height: UInt32(height), fx: Double(k.columns.0.x) * sx, fy: Double(k.columns.1.y) * sy, cx: (Double(k.columns.2.x) + 0.5) * sx - 0.5, cy: (Double(k.columns.2.y) + 0.5) * sy - 0.5, cameraX: Double(position.x), cameraY: Double(position.y), cameraZ: Double(position.z) + (mapGroundOffset ?? 0), quaternionX: Double(q.x), quaternionY: Double(q.y), quaternionZ: Double(q.z), quaternionW: Double(q.w), depthMetres: values))
            lastMapTime = frame.timestamp
            let forward = bodyOrientation.act(SIMD3<Float>(1, 0, 0))
            try publishMap(x: Double(position.x), y: Double(position.y), yaw: atan2(Double(forward.y), Double(forward.x)), status: "ARKit scene depth · 10 Hz")
        } catch { DispatchQueue.main.async { self.mapStatus = "Map: " + error.localizedDescription } }
    }
    func session(_ session: ARSession, didFailWithError error: Error) { fail(error) }
    func sessionWasInterrupted(_ session: ARSession) { fail(NSError(domain: "Terra", code: 1, userInfo: [NSLocalizedDescriptionKey: "AR session interrupted"])) }
    func setAutonomy(_ level:String, budgetSeconds: Double? = nil) {
        controlQueue.async {
            guard !self.bleActive || self.bleFeedback else { return }
            self.target=(0,0)
            do {
                var body: [String: Any] = ["level": level, "token": UUID().uuidString]
                if level == "explore" {
                    let budget = budgetSeconds ?? 120
                    if budget.isFinite, budget > 0 { body["budget_seconds"] = budget }
                }
                let payload=String(decoding:try JSONSerialization.data(withJSONObject:body),as:UTF8.self)
                if self.mode == .remote {try self.zenoh?.sendAction(kind:"autonomy",payload:payload)}else{try self.controller?.autonomyRequest(kind:"autonomy",payload:payload,timestamp:self.mode == .simulation ? self.simulatedTime:CACurrentMediaTime())}
            }catch{DispatchQueue.main.async{self.autonomyReason=error.localizedDescription}}
        }
    }
    func emergencyStop(reset:Bool=false) {
        // Stop/reset must also reach a connected rover after navigation has
        // stopped the motion timer while retaining its synchronized BLE link.
        let hardwareConnected = hardwareReady || hardwareConfigurationReady || bluetooth.connectedIdentifier != nil
        controlQueue.async {
            if self.bleActive || hardwareConnected {
                self.target = (0, 0); self.resetServoTargets()
                if reset { DispatchQueue.main.async { self.sendConfiguration("reset_emergency_stop", payload: [:]) } } else { self.bluetooth.emergencyStop() }
                return
            }
            self.target=(0,0);let payload="{\"action\":\"\(reset ? "reset":"stop")\",\"token\":\"\(UUID().uuidString)\"}";if self.mode == .remote {try? self.zenoh?.sendAction(kind:"safety",payload:payload)}else{try? self.controller?.autonomyRequest(kind:"safety",payload:payload,timestamp:self.mode == .simulation ? self.simulatedTime:CACurrentMediaTime())}}
    }
    func exportRun() {
        controlQueue.async {do {try self.controller?.endRecording()}catch{DispatchQueue.main.async{self.autonomyReason=error.localizedDescription}}}
    }
    func decideProposal(_ id:UInt64,approve:Bool) {
        let run=proposalRun
        controlQueue.async {do {let payload=String(decoding:try JSONSerialization.data(withJSONObject:["run_id":run,"proposal_id":id,"decision":approve ? "approve":"reject","token":UUID().uuidString]),as:UTF8.self);if self.mode == .remote{try self.zenoh?.sendAction(kind:"goal/decision",payload:payload)}else{try self.controller?.autonomyRequest(kind:"goal/decision",payload:payload,timestamp:self.mode == .simulation ? self.simulatedTime:CACurrentMediaTime())}}catch{DispatchQueue.main.async{self.autonomyReason=error.localizedDescription}}}
    }
    private func updateAuthority() {
        let raw=mode == .remote ? zenoh?.autonomyStatus():controller?.autonomyStatus()
        guard let raw,let data=raw.data(using:.utf8),let object=(try? JSONSerialization.jsonObject(with:data)) as? [String:Any] else{return}
        let status=(object["status"] as? [String:Any]) ?? object
        let level=status["requested_level"] as? String ?? "teleop", reason=status["reason"] as? String ?? "Unknown"
        let goal=object["goal"] as? [String:Any]
        let proposal=object["proposal"] as? [String:Any]
        let exploration=(object["exploration"] as? [String:Any]) ?? (status["exploration"] as? [String:Any])
        let summary: String = {
            guard let exploration, let phase=exploration["phase"] as? String, phase != "idle" else { return "" }
            let remaining=exploration["remaining_seconds"] as? Double ?? 0
            let coverage=exploration["coverage_m2"] as? Double ?? 0
            if let end=exploration["end_reason"] as? String {
                return String(format:"Explore ended · %@ · %.1f m² observed", end.replacingOccurrences(of:"_", with:" "), coverage)
            }
            if let target=exploration["target"] as? [String:Any] {
                let x=target["x"] as? Double ?? 0, y=target["y"] as? Double ?? 0
                return String(format:"Exploring · %.0f s left · %.1f m² · target %.1f, %.1f m", remaining, coverage, x, y)
            }
            return String(format:"Exploring · %.0f s left · %.1f m²", remaining, coverage)
        }()
        DispatchQueue.main.async {self.autonomyLevel=level;self.autonomyReason=reason;self.explorationSummary=summary;self.proposalRun=proposal?["run_id"] as? String ?? "";self.proposedGoal=(proposal?["proposal_id"] as? NSNumber)?.uint64Value;self.proposalText=proposal.map{String(format:"Search target %.1f, %.1f m",$0["x"] as? Double ?? 0,$0["y"] as? Double ?? 0)} ?? "";if let goal{self.waypointActive=(goal["state"] as? String)=="active";self.waypointDistance=goal["distance"] as? Double ?? 0;self.waypointStatus=goal["state"] as? String ?? "Unknown"}}
    }
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
        bluetooth.disarm(); bluetooth.disconnect(); bleActive = false
        mode = .stopped; timer?.cancel(); timer = nil; try? controller?.reset()
        zenoh?.disconnect(); zenoh = nil
        goalLatched = false
        remotePose = nil; phonePose = nil
        DispatchQueue.main.async {
            self.waypointActive = false; self.waypointDistance = 0; self.waypointStatus = "No goal"
            self.motion.stopDeviceMotionUpdates(); self.session.pause()
            self.status = error.localizedDescription; self.leftEffort = 0; self.rightEffort = 0
            self.mapStatus = "Map paused"
            self.zenohConnecting = false; self.zenohStatus = error.localizedDescription
        }
    }
}

struct RoverPeripheral: Identifiable, Decodable {
    let identifier: String
    let name: String
    var id: String { identifier }
}

extension PhoneController {
    private func sensorEpochMatches(_ epoch: Int) -> Bool {
        sensorEpochLock.lock(); defer { sensorEpochLock.unlock() }; return sensorEpoch == epoch
    }
    private func revokeHardwareMotion() {
        target = (0, 0); resetServoTargets()
        if !hardwareMotionRevoked { bluetooth.disarm(); hardwareMotionRevoked = true }
    }
    private func observeBluetooth() {
        bluetooth.objectWillChange.sink { [weak self] _ in
            DispatchQueue.main.async { self?.refreshBluetooth() }
        }.store(in: &subscriptions)
        bluetooth.$replyJSON.sink { [weak self] text in self?.receiveConfiguration(text) }.store(in: &subscriptions)
    }
    private func refreshBluetooth() {
        if configurationGeneration != bluetooth.connectionGeneration {
            configurationGeneration = bluetooth.connectionGeneration
            staged = nil; stageRequest = nil; commitRequest = nil; resetFaultRequest = nil
            acknowledgedCommitRevision = nil
            configurationStatus = "Connection changed · stage draft again"
        }
        hardwareStatus = bluetooth.status; hardwareReady = bluetooth.isReady
        if bluetooth.isReady, let identifier = bluetooth.connectedIdentifier {
            UserDefaults.standard.set(identifier.uuidString, forKey: "preferredHardwareRover")
        }
        hardwareConfigurationReady = bluetooth.configurationReady
        let remoteStatus = (try? JSONSerialization.jsonObject(with: Data(bluetooth.statusJSON.utf8))) as? [String: Any]
        hardwareBenchMode = remoteStatus?["gate_mode"] as? String == "bench"
        hardwareBenchEnabled = remoteStatus?["bench_enabled"] as? Bool == true
        activeLayoutRevision = remoteStatus?["active_revision"] as? UInt32 ?? 0
        hardwareFault = remoteStatus?["fault"] as? String
        if let staged, staged.revision != activeLayoutRevision { self.staged = nil; stageRequest = nil }

        hardwareArmed = bluetooth.armed; hardwareArming = bluetooth.arming
        capabilitiesJSON = bluetooth.capabilitiesJSON; committedLayoutJSON = bluetooth.layoutJSON
        discoveredRovers = (try? JSONDecoder().decode([RoverPeripheral].self, from: Data(bluetooth.peripheralsJSON.utf8))) ?? []
        feedbackCompatible = (try? actuatorSupportsFeedback(layoutJson: committedLayoutJSON)) ?? false
        let layout = committedLayoutJSON
        let ready = hardwareReady
        let compatible = feedbackCompatible
        if requestedHardwareFeedback != nil && !ready { feedbackConnectionSawUnready = true }
        if let requested = requestedHardwareFeedback, feedbackConnectionSawUnready, bluetooth.connectedIdentifier == requested, ready {
            requestedHardwareFeedback = nil
            if compatible && ARWorldTrackingConfiguration.isSupported && motion.isDeviceMotionAvailable {
                startPhoneSensors()
                hardwareFeedback = true
                controlQueue.async { self.bleFeedback = true; self.target = (0, 0); self.resetServoTargets() }
            } else {
                configurationStatus = "Feedback requires a compatible fresh rover layout and available phone tracking"
                disarmHardware()
            }
        }
        controlQueue.async {
            self.bleCompatible = compatible
            if self.bleLayout != layout { self.bleLayout = layout; self.target = (0, 0); self.resetServoTargets() }
            if self.bleActive && (!ready || (self.bleFeedback && !compatible)) { self.revokeHardwareMotion() }
        }
    }
    func autoConnectHardware() {
        #if !targetEnvironment(simulator)
        guard UserDefaults.standard.object(forKey: "autoConnectHardware") as? Bool != false,
              !hardwareActive, source == "Stopped", !autoConnectionPending else { return }
        autoConnectionPending = true
        let generation = autoConnectionGeneration
        let preferred = UserDefaults.standard.string(forKey: "preferredHardwareRover").flatMap(UUID.init(uuidString:))
        bluetooth.scanForAutomaticConnection(preferred: preferred) { [weak self] identifier in
            DispatchQueue.main.async {
                guard let self, self.autoConnectionGeneration == generation,
                      UserDefaults.standard.object(forKey: "autoConnectHardware") as? Bool != false,
                      !self.hardwareActive, self.source == "Stopped" else { return }
                self.autoConnectionPending = false
                self.startBluetooth(identifier: identifier, feedback: false)
            }
        }
        #endif
    }
    func cancelAutoConnection() {
        autoConnectionGeneration &+= 1; autoConnectionPending = false
        bluetooth.cancelAutomaticConnection()
    }
    func scanBluetooth() { cancelAutoConnection(); bluetooth.scan() }
    func startBluetooth(identifier: UUID, feedback: Bool) {
        stop()
        requestedHardwareFeedback = feedback ? identifier : nil
        feedbackConnectionSawUnready = false
        sensorEpochLock.lock(); let epoch = sensorEpoch; sensorEpochLock.unlock()
        hardwareActive = true; hardwareFeedback = false
        controlQueue.async {
            guard self.sensorEpochMatches(epoch) else { return }
            self.bleActive = true; self.bleFeedback = false; self.hardwareMotionRevoked = true
            self.target = (0, 0); self.resetServoTargets()
            self.mode = .bluetoothManual; self.tick = 0; self.startTimer()
            self.bluetooth.connect(identifier: identifier)
            DispatchQueue.main.async { self.source = feedback ? "Bluetooth · waiting for fresh feedback-compatible layout" : "Bluetooth · normalized manual effort" }
        }
    }
    func armHardware() {
        guard requestedHardwareFeedback == nil, hardwareReady, !hardwareArmed, !hardwareArming else { return }
        sensorEpochLock.lock(); let epoch = sensorEpoch; sensorEpochLock.unlock()
        controlQueue.async {
            guard self.sensorEpochMatches(epoch) else { return }
            // Navigation stops output while retaining the BLE link. An explicit
            // Arm can resume manual output on that already synchronized connection.
            if !self.bleActive {
                guard self.mode == .stopped else { return }
                self.bleActive = true; self.bleFeedback = false; self.mode = .bluetoothManual
                self.tick = 0; self.startTimer()
                DispatchQueue.main.async {
                    guard self.sensorEpochMatches(epoch) else { return }
                    self.hardwareActive = true; self.hardwareFeedback = false
                    self.source = "Bluetooth · normalized manual effort"
                }
            }
            guard !self.bleFeedback || (self.feedbackTrackingHealthy && self.bleCompatible) else { return }
            self.target = (0, 0); self.resetServoTargets(); self.hardwareMotionRevoked = false
            self.bluetooth.arm()
        }
    }
    func disarmHardware() {
        controlQueue.async { self.revokeHardwareMotion() }
    }
    func setServoTarget(id: UInt8, position: Double) {
        guard position.isFinite, hardwareArmed else { return }
        controlQueue.async {
            guard let layout = try? JSONDecoder().decode(ActuatorLayoutDraft.self, from: Data(self.bleLayout.utf8)),
                  let actuator = layout.actuators.first(where: { $0.id == Int(id) && $0.kind == "positional_servo" }) else { return }
            let value = min(actuator.limits.max, max(actuator.limits.min, position))
            self.servoTargets[String(id)] = value
            DispatchQueue.main.async { self.servoPositions[id] = value }
        }
    }
    private func resetServoTargets() {
        servoTargets = [:]
        if let layout = try? JSONDecoder().decode(ActuatorLayoutDraft.self, from: Data(bleLayout.utf8)) {
            for a in layout.actuators where a.kind == "positional_servo" {
                servoTargets[String(a.id)] = min(a.limits.max, max(a.limits.min, a.safe.value ?? 0))
            }
        }
        let values = servoTargets.reduce(into: [UInt8: Double]()) { result, entry in if let id = UInt8(entry.key) { result[id] = entry.value } }
        DispatchQueue.main.async { self.servoPositions = values }
    }
    private func routeHardware(left: Double, right: Double, now: TimeInterval) {
        guard bleActive, !bleFeedback || bleCompatible else { return }
        do {
            let input: [String: Any] = ["left_effort": left, "right_effort": right, "forward": bleFeedback ? 0 : target.forward, "turn": bleFeedback ? 0 : target.yaw, "servo_positions": servoTargets]
            let json = String(decoding: try JSONSerialization.data(withJSONObject: input), as: UTF8.self)
            let values = try actuatorRoute(layoutJson: bleLayout, inputJson: json)
            bluetooth.drive(valuesJSON: values, producedAt: now)
        } catch { revokeHardwareMotion() }
    }
    var configurationAllowed: Bool {
        configurationBlockingReason == nil
    }
    var configurationBlockingReason: String? {
        let status = (try? JSONSerialization.jsonObject(with: Data(bluetooth.statusJSON.utf8))) as? [String: Any] ?? [:]
        return ActuatorConfigurationPolicy.blockingReason(ready: hardwareConfigurationReady, status: status)
    }
    @discardableResult private func sendConfiguration(_ operation: String, payload: [String: Any]) -> UInt32? {
        guard nextRequest < 0x7fffffff else { return nil }
        nextRequest += 1
        UserDefaults.standard.set(Int(nextRequest), forKey: "hardwareConfigurationRequestID")
        guard let data = try? JSONSerialization.data(withJSONObject: ["schema_version": 1, "request_id": nextRequest, "operation": operation, "payload": payload]) else { return nil }
        bluetooth.request(envelope: data, generation: bluetooth.connectionGeneration); return nextRequest
    }
    func setBenchEnabled(_ enabled: Bool) {
        guard hardwareBenchMode, hardwareConfigurationReady else { return }
        if !enabled { setTarget(forward: 0, yaw: 0); disarmHardware() }
        sendConfiguration("set_bench_enabled", payload: ["enabled": enabled])
    }
    func resetHardwareFault() {
        guard configurationAllowed else { configurationStatus = configurationBlockingReason ?? "Configuration unavailable"; return }
        resetFaultRequest = sendConfiguration("reset_fault", payload: [:])
        configurationStatus = "Waiting for fault reset acknowledgement · remains disarmed"
    }
    func stageActuatorLayout(json: String) {
        staged = nil; stageRequest = nil
        guard configurationAllowed else { configurationStatus = configurationBlockingReason ?? "Configuration unavailable"; return }
        do {
            let errors = try actuatorValidateLayout(layoutJson: json, capabilitiesJson: capabilitiesJSON)
            guard errors == "[]" else { configurationStatus = errors; return }
            guard let object = try JSONSerialization.jsonObject(with: Data(json.utf8)) as? [String: Any] else { return }
            guard object["revision"] as? UInt32 == activeLayoutRevision else {
                configurationStatus = "Draft revision is stale · use the current revision before staging"; return
            }
            stageRequest = sendConfiguration("stage_layout", payload: ["layout": object]); configurationStatus = "Waiting for stage acknowledgement"
        } catch { configurationStatus = error.localizedDescription }
    }
    func invalidateStagedLayout() { staged = nil; stageRequest = nil; configurationStatus = "Draft changed · stage again" }
    var hasStagedLayout: Bool { staged != nil }
    func commitActuatorLayout() {
        guard configurationAllowed, let staged else { configurationStatus = configurationBlockingReason ?? "Validate and stage the draft first"; return }
        commitRequest = sendConfiguration("commit_layout", payload: ["staged_revision": staged.revision, "staged_request_id": staged.request])
        configurationStatus = "Waiting for commit acknowledgement"
    }
    private func receiveConfiguration(_ text: String) {
        guard let object = (try? JSONSerialization.jsonObject(with: Data(text.utf8))) as? [String: Any], let id = object["request_id"] as? UInt32 else { return }
        if id == resetFaultRequest {
            resetFaultRequest = nil
            configurationStatus = object["result"] as? String == "ok" ? "Fault reset acknowledged · arm explicitly when ready" : "Fault reset rejected: \(object["errors"] ?? [])"
            return
        }
        guard id == stageRequest || id == commitRequest else { return }
        guard object["result"] as? String == "ok" else { staged = nil; configurationStatus = "Rejected · draft retained: \(object["errors"] ?? [])"; return }
        if id == stageRequest {
            let payload = object["payload"] as? [String: Any]
            if let revision = payload?["staged_revision"] as? UInt32, revision == activeLayoutRevision, payload?["staged_request_id"] as? UInt32 == id { staged = (id, revision); configurationStatus = "Staged revision \(revision) · commit explicitly" }
        } else {
            staged = nil; stageRequest = nil; commitRequest = nil
            acknowledgedCommitRevision = object["active_revision"] as? UInt32
            configurationStatus = "Commit acknowledged · refreshing active layout"
            sendConfiguration("capabilities", payload: [:]); sendConfiguration("read_layout", payload: [:])
        }
    }
}
