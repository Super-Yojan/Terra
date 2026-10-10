import OSLog
import ARKit
import AVFoundation
import Combine
import CoreMotion
import CoreLocation
import Foundation
import QuartzCore
import simd

// Sensor input, target updates and Rust ticks are serialized on controlQueue.
// Observable properties are published only on the main queue.
final class PhoneController: NSObject, ObservableObject, ARSessionDelegate, CLLocationManagerDelegate, @unchecked Sendable {
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
    private var groundPolicy = PhoneGroundPolicy()
    private var lastMapTime = -Double.infinity
    @Published private(set) var zenohStatus = "Disconnected"
    @Published private(set) var zenohConnecting = false
    @Published private(set) var dashboardStatus = "Disconnected"
    @Published private(set) var dashboardConnecting = false
    @Published private(set) var dashboardConnected = false
    @Published private(set) var waypointActive = false
    @Published private(set) var waypointStatus = "No goal"
    @Published private(set) var waypointDistance = 0.0
    private var zenoh: MobileZenohClient?
    @Published private(set) var searchAvailable=false
    @Published private(set) var searchPhase=""
    @Published private(set) var searchReportText=""
    @Published private(set) var searchReportHistory:[String]=[]
    private var searchReportReceipts:[String:Double]=[:]
    @Published private(set) var searchTerminal=true
    @Published private(set) var autonomyLevel="teleop"
    @Published private(set) var autonomyReason="Idle"
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
    @Published private(set) var phoneTrackingActive = false
    @Published private(set) var phoneTrackingStatus = "Tracking starts after ARGOS connects"
    private var fleetTrackingManaged = false
    private var fleetTrackingFailed = false
    @Published private(set) var hardwareReady = false
    @Published private(set) var hardwareConfigurationReady = false
    @Published private(set) var activeLayoutRevision: UInt32 = 0
    @Published private(set) var hardwareFault: String?
    @Published private(set) var connectionFacts = TerraConnectionFacts()
    @Published private(set) var roverDisplayName = UserDefaults.standard.string(forKey: "preferredHardwareRoverName") ?? "Your rover"
    private var roverNames: [UUID: String] = [:]
    private var roverConnectionFailed = false
    @Published private(set) var pairingCandidate: TerraDiscoveredRover?
    private var connectionPolicy = TerraConnectionPolicy()
    private var connectionTimer: DispatchSourceTimer?
    private var nearbyPairing = TerraPairingCandidates()
    private var routerRetryAt: TimeInterval?
    private var routerSuppressed = false
    private let dashboardGenerationLock = NSLock()
    private var dashboardGeneration: UInt64 = 0
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
    private let routerQueue = DispatchQueue(label: "terra.router.connection")
    private var dashboardHardwareToken = ""
    private var dashboardHardwareResult = ""
    private var bleActive = false
    private var bleFeedback = false
    private var bleLayout = "{}"
    private var servoTargets: [String: Double] = [:]
    @Published private(set) var actuatorEditor = ActuatorPaginationState()
    @Published private(set) var actuatorPageLoading = false
    private typealias EditorRequest = (id: UInt32, operation: String, generation: UInt64, actuator: ActuatorDraft?, actuatorID: Int?)
    private var editorRequest: EditorRequest?
    private var retryableEditorRequest: EditorRequest?
    @Published private(set) var hasRetryableActuatorRequest = false
    private var presetUpload: ActuatorPresetUpload?
    private var beginReplacement = false
    private var removeAfterBegin: Int?
    private var nextRequest: UInt32 = max(1000, UInt32(clamping: UserDefaults.standard.integer(forKey: "hardwareConfigurationRequestID")))
    private var sensorEpoch = 0
    private let sensorEpochLock = NSLock()
    private var dashboardEmergencyStop = false
    private var emergencyResetRequest: UInt32?
    private var hardwareMotionRevoked = true
    private var feedbackTrackingHealthy = false
    private var bleCompatible = false
    private var phoneStartedAt = 0.0
    private let controlQueue = DispatchQueue(label: "terra.control", qos: .userInteractive)
    private let motionQueue = OperationQueue()
    private let motion = CMMotionManager()
    private let session = ARSession()
    private let locationManager = CLLocationManager()
    private var locationFix: CLLocation?
    private var locationHeading: CLHeading?
    private var localizationPolicy = PhoneLocalizationPolicy()
    private var localizationTracked = false
    private var phoneTopYaw: Double?
    private var lastLocalizationPublish = -Double.infinity
    private var controller: MobileController?
    private var timer: DispatchSourceTimer?
    private var mode: Mode = .stopped
    private var target = (forward: 0.0, yaw: 0.0)
    private var lastLocalTarget = (forward: 0.0, yaw: 0.0)
    private var lastCloudTime = -Double.infinity
    private var tick = 0
    private var simulatedTime = 0.0
    private var simulatedForward = 0.0
    private var simulatedYawRate = 0.0
    private var simulatedYaw = 0.0
    private var simulatedAcceleration = 0.0
    private var previousPosition: SIMD3<Float>?
    private var previousPoseTime: Double?
    private var filteredVelocity = SIMD3<Float>.zero
    // Mounted with the left edge forward: x_body=-x_phone,
    // y_body=-y_phone, z_body=z_phone. Use the same mount for IMU and AR pose.
    private let phoneToBody = simd_quatf(angle: .pi, axis: SIMD3(0, 0, 1))
    // ARKit world +Y up -> robotics world +Z up, preserving right handedness.
    private let worldFromAR = simd_float3x3(columns: (SIMD3(0, -1, 0), SIMD3(0, 0, 1), SIMD3(-1, 0, 0)))
    override init() {
        super.init()
        locationManager.delegate = self
        locationManager.desiredAccuracy = kCLLocationAccuracyBest
        locationManager.distanceFilter = 1
        locationManager.headingFilter = 2
        locationManager.headingOrientation = .portrait
        motionQueue.maxConcurrentOperationCount = 1
        motionQueue.qualityOfService = .userInteractive
        session.delegate = self
        session.delegateQueue = controlQueue
        observeBluetooth()
    }
    func setTarget(forward: Double, yaw: Double) {
        controlQueue.async { self.target = self.dashboardEmergencyStop ? (0, 0) : (forward, yaw) }
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
    /// Dashboard transport is independent of the Simulator-only Bevy hardware link.
    func connectDashboard(endpoint: String, prefix: String, roverID: String) {
        guard !dashboardConnecting, !dashboardConnected else { return }
        let attempt = dashboardAttemptToken()
        guard let id = UInt64(roverID.trimmingCharacters(in: .whitespacesAndNewlines)) else {
            dashboardStatus = "Enter a non-negative rover ID"; routerSuppressed = true; return
        }
        guard let settings = TerraRouterSettings.parse(endpoint: endpoint, prefix: prefix, roverID: roverID) else {
            dashboardStatus = "Enter a TCP router endpoint and valid topic prefix"; routerSuppressed = true; return
        }
        let endpoint = settings.endpoint, prefix = settings.prefix
        dashboardConnecting = true; dashboardStatus = "Connecting…"
        sensorEpochLock.lock(); let epoch = sensorEpoch; sensorEpochLock.unlock()
        controlQueue.async {
            guard self.sensorEpochMatches(epoch), self.dashboardAttemptMatches(attempt) else { return }
            if self.mode == .bluetoothManual, self.controller == nil {
                do { self.controller = try MobileController(settings: defaultControlSettings()) }
                catch {
                    DispatchQueue.main.async {
                        guard self.dashboardAttemptMatches(attempt) else { return }
                        self.dashboardConnecting = false; self.dashboardStatus = error.localizedDescription
                        self.routerRetryAt = self.connectionNow + self.connectionPolicy.nextRouterDelay()
                    }
                    return
                }
            }
            guard (self.mode == .phone || self.mode == .simulation || self.mode == .bluetoothManual), let controller = self.controller else {
                DispatchQueue.main.async {
                    guard self.sensorEpochMatches(epoch), self.dashboardAttemptMatches(attempt) else { return }
                    self.dashboardConnecting = false
                    self.dashboardStatus = "Start Phone IMU + VIO or Bluetooth feedback control first"
                }
                return
            }
            let continuingWaypoint = TerraFleetManualDrive.localWaypointActive(controller.autonomyStatus())
            // Connecting the router preserves the current local drive session.
            // Network setup must not block local AR and control updates.
            self.routerQueue.async {
            do {
                if continuingWaypoint {
                    try controller.reconnectDashboard(endpoint: endpoint, prefix: prefix, roverId: id)
                } else {
                    try controller.connectDashboard(endpoint: endpoint, prefix: prefix, roverId: id)
                }
                guard self.sensorEpochMatches(epoch), self.dashboardAttemptMatches(attempt) else {
                    try? controller.disconnectDashboard(); return
                }
                DispatchQueue.main.async {
                    guard self.sensorEpochMatches(epoch), self.dashboardAttemptMatches(attempt) else { return }
                    self.connectionPolicy.routerConnected(); self.routerRetryAt = nil
                    self.dashboardConnecting = false; self.dashboardConnected = true
                    self.dashboardStatus = continuingWaypoint ? "Router restored · local waypoint retained" : "Router connected · rover \(id) · hardware remains disarmed"
                    self.fleetTrackingFailed = false
                    self.refreshConnectionPresentation()
                }
            } catch {
                DispatchQueue.main.async {
                    guard self.sensorEpochMatches(epoch), self.dashboardAttemptMatches(attempt) else { return }
                    self.dashboardConnecting = false; self.dashboardConnected = false
                    self.dashboardStatus = error.localizedDescription
                    self.routerRetryAt = self.connectionNow + self.connectionPolicy.nextRouterDelay()
                }
            }
            }
        }
    }
    func disconnectDashboard() {
        routerSuppressed = true; routerRetryAt = nil; invalidateDashboardAttempt()
        controlQueue.async {
            self.target = (0, 0); self.revokeHardwareMotion()
            try? self.controller?.disconnectDashboard()
            self.goalLatched = false
            DispatchQueue.main.async {
                self.dashboardConnecting = false; self.dashboardConnected = false
                self.dashboardStatus = "Disconnected · remote intent cleared"
                self.waypointActive = false; self.waypointDistance = 0; self.waypointStatus = "No goal"
            }
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
                self.mapGroundOffset = nil; self.groundPolicy.reset()
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
        invalidateDashboardAttempt(); dashboardConnecting = false
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
        configuration.planeDetection = [.horizontal]
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) { configuration.frameSemantics.insert(.sceneDepth) }
        locationManager.requestWhenInUseAuthorization()
        locationManager.startUpdatingLocation()
        if CLLocationManager.headingAvailable() { locationManager.startUpdatingHeading() }
        session.run(configuration, options: [.resetTracking, .removeExistingAnchors])
        motion.deviceMotionUpdateInterval = 0.01
        motion.startDeviceMotionUpdates(using: .xArbitraryZVertical, to: motionQueue) { [weak self] data, error in
            guard let self else { return }
            if let error { self.controlQueue.async { if self.sensorEpochMatches(epoch) { self.handleTrackingFailure(error) } }; return }
            guard let data else { return }
            let acceleration = self.phoneToBody.act(SIMD3(Float(data.userAcceleration.x), Float(data.userAcceleration.y), Float(data.userAcceleration.z))) * 9.80665
            let gyro = self.phoneToBody.act(SIMD3(Float(data.rotationRate.x), Float(data.rotationRate.y), Float(data.rotationRate.z)))
            let sample = ImuReading(timestamp: data.timestamp, accelerationForward: Double(acceleration.x), accelerationLeft: Double(acceleration.y), accelerationUp: Double(acceleration.z), gyroRoll: Double(gyro.x), gyroPitch: Double(gyro.y), gyroYaw: Double(gyro.z))
            self.controlQueue.async {
                guard self.mode == .phone, self.sensorEpochMatches(epoch) else { return }
                do { try self.controller?.pushImu(sample: sample) } catch { self.fail(error, context: "IMU sample at \(sample.timestamp)") }
            }
        }
        phoneTrackingActive = true
        source = "Core Motion + ARKit"
    }
    /// keepBluetooth leaves the rover link up for the actuator layout screen,
    /// which is pushed from this root and would otherwise drop on navigation.
    func stop(keepBluetooth: Bool = false) {
        connectionPolicy.stop(); pairingCandidate = nil; nearbyPairing.removeAll()
        routerSuppressed = true; routerRetryAt = nil; invalidateDashboardAttempt()
        dashboardConnecting = false
        bluetooth.cancelAutomaticConnection()
        sensorEpochLock.lock(); sensorEpoch += 1; sensorEpochLock.unlock()
        bluetooth.disarm()
        if !keepBluetooth { bluetooth.disconnect() }
        hardwareActive = false; requestedHardwareFeedback = nil
        phoneTrackingActive = false; hardwareFeedback = false
        fleetTrackingManaged = false; fleetTrackingFailed = false
        phoneTrackingStatus = "Tracking starts after ARGOS connects"
        motion.stopDeviceMotionUpdates()
        locationManager.stopUpdatingLocation(); locationManager.stopUpdatingHeading()
        session.pause()
        controlQueue.async {
            self.timer?.cancel(); self.timer = nil
            self.mode = .stopped
            self.dashboardEmergencyStop = false
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
                self.dashboardConnecting = false; self.dashboardConnected = false; self.dashboardStatus = "Disconnected"
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
        let preserveRuntime = mode == .phone && self.mode == .bluetoothManual && self.controller != nil
        if !preserveRuntime {
            try? self.controller?.reset()
            self.controller = try MobileController(settings: defaultControlSettings())
            DispatchQueue.main.async {
                self.dashboardConnecting = false; self.dashboardConnected = false; self.dashboardStatus = "Disconnected"
            }
        }
        if !preserveRuntime {
                let log=FileManager.default.urls(for:.documentDirectory,in:.userDomainMask)[0].appendingPathComponent("Terra-run-\(UUID().uuidString).jsonl")
                try self.controller?.beginRecording(path:log.path,runId:UUID().uuidString)
                DispatchQueue.main.async {self.runLog=log}
        }
        self.occupancyMap = try MobileOccupancyMap(settings: defaultOccupancySettings())
        simulatedPosition = .zero; mapGroundOffset = nil; groundPolicy.reset(); lastMapTime = -Double.infinity
        DispatchQueue.main.async { self.occupancy = nil; self.mapPose = .zero; self.mapStatus = mode == .simulation ? "Simulated depth · 10 Hz" : (ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) ? "Waiting for scene depth" : "Scene depth unavailable on this device") }
        self.mode = mode
        phoneStartedAt = ProcessInfo.processInfo.systemUptime
        target = (0, 0)
        tick = 0; simulatedTime = 0; simulatedForward = 0; simulatedYawRate = 0; simulatedYaw = 0; simulatedAcceleration = 0
        previousPosition = nil; previousPoseTime = nil; filteredVelocity = .zero
        localizationPolicy.reset(); locationFix = nil; locationHeading = nil; localizationTracked = false; phoneTopYaw = nil; lastLocalizationPublish = -Double.infinity
    }
    private func startTimer() {
        self.timer?.cancel(); self.timer = nil
        let timer = DispatchSource.makeTimerSource(queue: controlQueue)
        timer.schedule(deadline: .now(), repeating: .milliseconds(10), leeway: .milliseconds(1))
        timer.setEventHandler { [weak self] in self?.update() }
        self.timer = timer
        timer.resume()
    }
    private func serviceManualDashboard(now: TimeInterval) {
        guard let controller else { return }
        let attempt = dashboardAttemptToken()
        // Preserve the AR/controller clock when switching to manual hardware.
        // Do not repeatedly detach/reset authority once the router is disconnected.
        if controller.dashboardStatus() == "connected" {
            do { _ = try controller.step(timestamp: now) }
            catch {
                target = (0, 0); revokeHardwareMotion()
                try? controller.disconnectDashboard()
                DispatchQueue.main.async { self.status = "Dashboard control update: \(error.localizedDescription)" }
            }
        }
        if TerraManualRouterSafety.emergencyStopped(statusJSON: controller.autonomyStatus()), !dashboardEmergencyStop {
            dashboardEmergencyStop = true; target = (0, 0); resetServoTargets()
            bluetooth.emergencyStop(); hardwareMotionRevoked = true
            DispatchQueue.main.async { self.autonomyReason = "Router emergency stop · reset explicitly" }
        }
        if controller.dashboardStatus() != "connected" {
            if controller.dashboardStatus() == "failed" { try? controller.disconnectDashboard() }
            DispatchQueue.main.async {
                guard self.dashboardAttemptMatches(attempt), self.dashboardConnected else { return }
                self.dashboardConnected = false; self.dashboardConnecting = false
                self.dashboardStatus = "Router connection lost · retrying"
                self.routerRetryAt = self.connectionNow + self.connectionPolicy.nextRouterDelay()
            }
        }
    }
    private func update() {
        if mode == .bluetoothManual {
            if tick % 10 == 0 { serviceManualDashboard(now: CACurrentMediaTime()); updateAuthority() }
            tick += 1
            if tick % 5 == 0 {
                let command = controller?.dashboardStatus() == "connected"
                    ? TerraFleetManualDrive.command(controller?.autonomyStatus() ?? "{}")
                    : DriveJoystickCommand(forward: target.forward, yaw: target.yaw)
                let wheels = command.wheelEfforts
                routeHardware(left: wheels.left, right: wheels.right, now: ProcessInfo.processInfo.systemUptime, manualTarget: command)
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
            // An attached fleet owns velocity intent; idle phone input must not
            // overwrite ARGOS teleop commands between incoming router messages.
            if !(bleActive && controller.dashboardStatus() == "connected") {
                // Idle phone controls must not overwrite remote operator intent every tick.
            // A local held input has priority; its transition to neutral is still sent once.
            if target.forward != 0 || target.yaw != 0 || target != lastLocalTarget {
                try controller.setTarget(target: TwistSetpoint(timestamp:now,forward:target.forward,yawRate:target.yaw))
                lastLocalTarget = target
            }
            }
            let output = try controller.step(timestamp: now)
            if tick % 10 == 0 && controller.dashboardStatus() == "failed" {
                let continuingWaypoint = TerraFleetManualDrive.localWaypointActive(controller.autonomyStatus())
                if continuingWaypoint { try controller.detachDashboardLink() }
                else {
                    self.target = (0, 0); self.revokeHardwareMotion()
                    try controller.disconnectDashboard()
                }
                DispatchQueue.main.async {
                    self.dashboardConnecting = false; self.dashboardConnected = false
                    self.dashboardStatus = continuingWaypoint ? "Router interrupted · continuing local waypoint" : "Router connection lost · retrying"
                    self.routerRetryAt = self.connectionNow + self.connectionPolicy.nextRouterDelay()
                }
            }
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
            let autonomy = controller.autonomyStatus()
            let fullyManual = TerraFleetManualDrive.isFullyManual(autonomy)
            if bleActive && bleFeedback { feedbackTrackingHealthy = output.safety == .active }
            if bleActive && fullyManual && tick % 5 == 0 {
                // L0 has no sensor-driven motor correction. The arbiter still
                // gates the operator command by authority, freshness and safety.
                let command = TerraFleetManualDrive.command(autonomy)
                let wheels = command.wheelEfforts
                routeHardware(left: wheels.left, right: wheels.right, now: ProcessInfo.processInfo.systemUptime, manualTarget: command)
            }
            if bleActive && bleFeedback && !fullyManual {
                if !feedbackTrackingHealthy { revokeHardwareMotion() }
                if tick % 5 == 0 { routeHardware(left: output.leftEffort, right: output.rightEffort, now: ProcessInfo.processInfo.systemUptime) }
            }
            if bleActive && !bleFeedback && !fullyManual && tick % 5 == 0 && controller.dashboardStatus() == "connected" {
                let command = TerraFleetManualDrive.command(autonomy)
                let wheels = command.wheelEfforts
                routeHardware(left: wheels.left, right: wheels.right, now: ProcessInfo.processInfo.systemUptime, manualTarget: command)
            }
            if tick % 10 == 0 { publish(output);updateAuthority() }
            if mode == .phone { publishLocalization(controller) }
        } catch { fail(error, context: "Control update in \(mode) at \(now)") }
    }
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        guard mode == .phone, frame.timestamp >= phoneStartedAt, let controller else { return }
        let tracked: Bool
        if case .normal = frame.camera.trackingState { tracked = true } else { tracked = false }
        if localizationTracked != tracked {
            TerraLog.tracking.notice("Camera tracking changed; healthy=\(tracked) state=\(String(describing: frame.camera.trackingState), privacy: .public)")
        }
        localizationTracked = tracked
        sensorEpochLock.lock(); let trackingEpoch = sensorEpoch; sensorEpochLock.unlock()
        DispatchQueue.main.async {
            guard self.sensorEpochMatches(trackingEpoch), self.phoneTrackingActive else { return }
            self.phoneTrackingStatus = tracked ? (self.fleetTrackingManaged ? "Tracking active · managed by ARGOS in every mode" : "Phone tracking active") : "Tracking initializing · keep the phone steady"
        }
        let transform = frame.camera.transform
        let top = worldFromAR * SIMD3(transform.columns.1.x, transform.columns.1.y, transform.columns.1.z)
        phoneTopYaw = hypot(top.x, top.y) >= 0.3 ? atan2(Double(top.y), Double(top.x)) : nil
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
        if !tracked {
            filteredVelocity = .zero
            if bleActive {
                feedbackTrackingHealthy = false
                let fullyManual = TerraFleetManualDrive.isFullyManual(controller.autonomyStatus())
                if DriveJoystickSafety.trackingLossRequiresDisarm(feedback: bleFeedback, fullyManual: fullyManual) {
                    revokeHardwareMotion(reason: "camera tracking lost in tracking-dependent mode")
                }
            }
        }
        let q = orientation.vector
        let bodyForward = orientation.act(SIMD3<Float>(1, 0, 0))
        phonePose = (Double(position.x), Double(position.y), atan2(Double(bodyForward.y), Double(bodyForward.x)))
        do {
            try controller.pushVio(sample: VioReading(timestamp: frame.timestamp, positionX: Double(position.x), positionY: Double(position.y), positionZ: Double(position.z), quaternionX: Double(q.x), quaternionY: Double(q.y), quaternionZ: Double(q.z), quaternionW: Double(q.w), velocityX: Double(filteredVelocity.x), velocityY: Double(filteredVelocity.y), velocityZ: Double(filteredVelocity.z), tracked: tracked && velocityReady))
            if tracked { updatePhoneGround(frame: frame, position: position); updatePhoneMap(frame: frame, position: position, bodyOrientation: orientation); publishPhoneCloud(frame: frame, position: position) }
            else { DispatchQueue.main.async { self.mapStatus = "Tracking lost · map paused" } }
        } catch { fail(error, context: "AR frame at \(frame.timestamp)") }
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
    private func updatePhoneGround(frame: ARFrame, position: SIMD3<Float>) {
        // ARKit world origin is the initial camera, not the floor. Use a stable
        // classified floor and keep its reference fixed until tracking restarts.
        if mapGroundOffset == nil {
            let floors = frame.anchors.compactMap { $0 as? ARPlaneAnchor }.filter {
                $0.alignment == .horizontal && ($0.classification == .floor || $0.classification == .none)
                && Double(position.z) - Double($0.transform.columns.3.y) >= 0.1
                && $0.planeExtent.width * $0.planeExtent.height >= 0.4
            }
            let classified = floors.filter { $0.classification == .floor }
            let floor = (classified.isEmpty ? floors : classified).min { $0.transform.columns.3.y < $1.transform.columns.3.y }
            let height = floor.map { Double(($0.transform * SIMD4<Float>($0.center.x, $0.center.y, $0.center.z, 1)).y) }
            let area = floor.map { Double($0.planeExtent.width * $0.planeExtent.height) } ?? 0
            mapGroundOffset = groundPolicy.observe(height: height, cameraHeight: Double(position.z), area: area, now: frame.timestamp)
            guard mapGroundOffset != nil else {
                DispatchQueue.main.async { self.mapStatus = "Finding floor · point the camera toward the ground" }
                return
            }
            try? occupancyMap?.clear()
        }
    }
    private func publishPhoneCloud(frame: ARFrame, position: SIMD3<Float>) {
        guard frame.timestamp - lastCloudTime >= 0.3, let controller else { return }
        lastCloudTime = frame.timestamp
        var points: [[Double]] = []
        var source = "features"
        guard let offset = mapGroundOffset else { return }
        if let depth = frame.sceneDepth {
            source = "lidar"
            let buffer = depth.depthMap
            if CVPixelBufferGetPixelFormatType(buffer) == kCVPixelFormatType_DepthFloat32,
               CVPixelBufferLockBaseAddress(buffer, .readOnly) == kCVReturnSuccess {
                defer { CVPixelBufferUnlockBaseAddress(buffer, .readOnly) }
                let width = CVPixelBufferGetWidth(buffer), height = CVPixelBufferGetHeight(buffer)
                let stride = CVPixelBufferGetBytesPerRow(buffer)
                let k = frame.camera.intrinsics
                let sx = Double(width) / Double(frame.camera.imageResolution.width), sy = Double(height) / Double(frame.camera.imageResolution.height)
                let fx = Double(k.columns.0.x)*sx, fy = Double(k.columns.1.y)*sy
                let cx = (Double(k.columns.2.x)+0.5)*sx-0.5, cy = (Double(k.columns.2.y)+0.5)*sy-0.5
                let orientation = simd_quatf(worldFromAR) * simd_quatf(frame.camera.transform) * simd_quatf(angle: .pi, axis: SIMD3(1,0,0))
                let step = max(1, Int(ceil(sqrt(Double(width*height)/1800))))
                if let base = CVPixelBufferGetBaseAddress(buffer) {
                    for v in Swift.stride(from: 0, to: height, by: step) {
                        let row = base.advanced(by: v*stride).assumingMemoryBound(to: Float.self)
                        for u in Swift.stride(from: 0, to: width, by: step) {
                            let d = row[u]
                            guard d.isFinite, d >= 0.15, d <= 8 else { continue }
                            let optical = SIMD3<Float>(Float((Double(u)-cx)/fx)*d, Float((Double(v)-cy)/fy)*d, d)
                            let p = position + orientation.act(optical)
                            points.append([Double(p.x), Double(p.y), Double(p.z)+offset])
                        }
                    }
                }
            }
        } else if let features = frame.rawFeaturePoints {
            let step = max(1, Int(ceil(Double(features.points.count)/1800)))
            for i in Swift.stride(from: 0, to: features.points.count, by: step) {
                let p = worldFromAR * features.points[i]
                if p.x.isFinite && p.y.isFinite && p.z.isFinite { points.append([Double(p.x),Double(p.y),Double(p.z)+offset]) }
            }
        }
        let value: [String: Any] = ["version":1,"sequence":UInt64(frame.timestamp*1000),"frameID":localizationPolicy.identifier,"source":source,"points":points]
        if let data = try? JSONSerialization.data(withJSONObject: value) {
            try? controller.publishPointCloud(payload: String(decoding: data, as: UTF8.self))
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
        guard mapGroundOffset != nil else { return }
        do {
            try map.recenter(x: Double(position.x), y: Double(position.y))
            try map.integrateDepth(frame: MappingDepthFrame(timestamp: frame.timestamp, width: UInt32(width), height: UInt32(height), fx: Double(k.columns.0.x) * sx, fy: Double(k.columns.1.y) * sy, cx: (Double(k.columns.2.x) + 0.5) * sx - 0.5, cy: (Double(k.columns.2.y) + 0.5) * sy - 0.5, cameraX: Double(position.x), cameraY: Double(position.y), cameraZ: Double(position.z) + (mapGroundOffset ?? 0), quaternionX: Double(q.x), quaternionY: Double(q.y), quaternionZ: Double(q.z), quaternionW: Double(q.w), depthMetres: values))
            lastMapTime = frame.timestamp
            let forward = bodyOrientation.act(SIMD3<Float>(1, 0, 0))
            try publishMap(x: Double(position.x), y: Double(position.y), yaw: atan2(Double(forward.y), Double(forward.x)), status: "ARKit scene depth · measured floor · 10 Hz")
        } catch { DispatchQueue.main.async { self.mapStatus = "Map: " + error.localizedDescription } }
    }
    private func handleTrackingFailure(_ error: Error) {
        sensorEpochLock.lock(); let epoch = sensorEpoch; sensorEpochLock.unlock()
        DispatchQueue.main.async {
            guard self.sensorEpochMatches(epoch), self.phoneTrackingActive else { return }
            if self.fleetTrackingManaged {
                self.fleetTrackingFailed = true
                self.setHardwareFeedback(enabled: false)
                self.phoneTrackingStatus = "Tracking unavailable: " + error.localizedDescription
            } else { self.controlQueue.async { self.fail(error) } }
        }
    }
    func session(_ session: ARSession, didFailWithError error: Error) { handleTrackingFailure(error) }
    func sessionWasInterrupted(_ session: ARSession) { controlQueue.async { self.localizationPolicy.reset(); self.localizationTracked = false; self.phoneTopYaw = nil }; handleTrackingFailure(NSError(domain: "Terra", code: 1, userInfo: [NSLocalizedDescriptionKey: "AR session interrupted"])) }
    func startTargetSearch(targetClass:String,minX:Double,minY:Double,maxX:Double,maxY:Double,budget:Double) {
        controlQueue.async {
            do {
                let raw=self.mode == .remote ? self.zenoh?.autonomyStatus():self.controller?.autonomyStatus()
                let object=(try? JSONSerialization.jsonObject(with:Data((raw ?? "{}").utf8))) as? [String:Any] ?? [:]
                let status=object["status"] as? [String:Any] ?? object
                guard let run=status["run_id"] as? String,status["requested_level"] as? String=="target_search",[minX,minY,maxX,maxY,budget].allSatisfy({$0.isFinite}),minX<maxX,minY<maxY,maxX-minX<=100,maxY-minY<=100,budget>0,budget<=3600 else {throw NSError(domain:"TerraSearch",code:1,userInfo:[NSLocalizedDescriptionKey:"Select target search and enter valid local bounds and budget"])}
                let payload=String(decoding:try JSONSerialization.data(withJSONObject:["version":1,"run_id":run,"search_id":UUID().uuidString,"token":UUID().uuidString,"target_class":targetClass,"bounds":["min_x":minX,"min_y":minY,"max_x":maxX,"max_y":maxY],"time_budget_s":budget]),as:UTF8.self)
                if self.mode == .remote {try self.zenoh?.sendAction(kind:"search",payload:payload)}else{try self.controller?.autonomyRequest(kind:"search",payload:payload,timestamp:self.mode == .simulation ? self.simulatedTime:CACurrentMediaTime())}
            }catch{DispatchQueue.main.async{self.autonomyReason=error.localizedDescription}}
        }
    }
    func targetSearchAction(_ action:String) {
        controlQueue.async {
            do {
                let raw=self.mode == .remote ? self.zenoh?.autonomyStatus():self.controller?.autonomyStatus()
                let object=(try? JSONSerialization.jsonObject(with:Data((raw ?? "{}").utf8))) as? [String:Any] ?? [:]
                guard let search=TerraTargetSearchPresentation.currentSearch(object,remote:self.mode == .remote),let run=search["run_id"] as? String,let id=search["search_id"] as? String else {return}
                let payload=String(decoding:try JSONSerialization.data(withJSONObject:["version":1,"run_id":run,"search_id":id,"token":UUID().uuidString,"action":action]),as:UTF8.self)
                if self.mode == .remote {try self.zenoh?.sendAction(kind:"search/action",payload:payload)}else{try self.controller?.autonomyRequest(kind:"search/action",payload:payload,timestamp:self.mode == .simulation ? self.simulatedTime:CACurrentMediaTime())}
            }catch{DispatchQueue.main.async{self.autonomyReason=error.localizedDescription}}
        }
    }
    func setAutonomy(_ level:String) {
        controlQueue.async {
            guard !self.bleActive || self.bleFeedback else { return }
            self.target=(0,0)
            do {let payload=String(decoding:try JSONSerialization.data(withJSONObject:["level":level,"token":UUID().uuidString]),as:UTF8.self)
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
                if reset { DispatchQueue.main.async { self.emergencyResetRequest = self.sendConfiguration("reset_emergency_stop", payload: [:]) } } else { self.bluetooth.emergencyStop() }
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
        if let controller {
            let request = controller.takeHardwareRequest()
            if let r = (try? JSONSerialization.jsonObject(with: Data(request.utf8))) as? [String: Any],
               let action = r["action"] as? String, let token = r["token"] as? String {
                let expiry = ProcessInfo.processInfo.systemUptime + (r["valid_for"] as? Double ?? 0)
                let run = r["run_id"] as? String
                let revision = (r["authority_revision"] as? NSNumber)?.uint64Value
                DispatchQueue.main.async {
                    self.dashboardHardwareToken = token
                    if action == "disarm" {
                        self.dashboardHardwareResult = "disarming"
                        self.disarmHardware()
                    } else if !self.hardwareReady || self.hardwareFault != nil || self.dashboardEmergencyStop {
                        self.dashboardHardwareResult = "rejected: rover not ready"
                    } else {
                        self.dashboardHardwareResult = "requested"
                        self.armHardware(expectedRun: run, expectedRevision: revision, expiresAt: expiry)
                    }
                }
            }
            DispatchQueue.main.async {
                let remote = (try? JSONSerialization.jsonObject(with: Data(self.bluetooth.statusJSON.utf8))) as? [String: Any] ?? [:]
                var state: [String: Any] = ["schema_version": 1, "ready": self.hardwareReady,
                    "armed": self.hardwareArmed, "arming": self.hardwareArming,
                    "token": self.dashboardHardwareToken, "result": self.dashboardHardwareResult,
                    "reason": self.hardwareStatus]
                state["stop_reason"] = remote["stop_reason"]
                if let fault = self.hardwareFault { state["fault"] = fault }
                if let payload = try? JSONSerialization.data(withJSONObject: state) {
                    try? controller.publishHardwareStatus(payload: String(decoding: payload, as: UTF8.self))
                }
            }
        }

        let raw=mode == .remote ? zenoh?.autonomyStatus():controller?.autonomyStatus()
        guard let raw,let data=raw.data(using:.utf8),let object=(try? JSONSerialization.jsonObject(with:data)) as? [String:Any] else{return}
        #if DEBUG
        if tick % 50 == 0 {
            let snapshot: [String: Any] = ["time": ISO8601DateFormatter().string(from: Date()), "mode": String(describing: mode), "feedback": bleFeedback, "feedbackTrackingHealthy": feedbackTrackingHealthy, "router": controller?.dashboardStatus() ?? "none", "autonomy": object]
            let file = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0].appendingPathComponent("ControlDiagnostics.json")
            if let bytes = try? JSONSerialization.data(withJSONObject: snapshot) { try? bytes.write(to: file, options: .atomic) }
        }
        #endif
        let status=(object["status"] as? [String:Any]) ?? object
        let level=status["requested_level"] as? String ?? "teleop", reason=status["reason"] as? String ?? "Unknown"
        let goal=object["goal"] as? [String:Any]
        let proposal=object["proposal"] as? [String:Any]
        let reports=(object[self.mode == .remote ? "reports":"pending_reports"] as? [[String:Any]]) ?? []
        for report in reports {
            guard let run=report["run_id"] as? String,let search=report["search_id"] as? String,let id=report["report_id"] as? String else {continue}
            let identity="\(run):\(search):\(id)"
            let now=CACurrentMediaTime()
            if searchReportReceipts[identity].map({now-$0<1}) == true {continue}
            if searchReportReceipts[identity]==nil {
                let line=String(format:"%@ · %.2f, %.2f m · %d frames",report["target_class"] as? String ?? "Target",report["world_x"] as? Double ?? 0,report["world_y"] as? Double ?? 0,(report["frame_ids"] as? [Any])?.count ?? 0)
                DispatchQueue.main.async {self.searchReportHistory=Array((self.searchReportHistory+[line]).suffix(128))}
            }
            do {
                let payload=String(decoding:try JSONSerialization.data(withJSONObject:["version":1,"run_id":run,"search_id":search,"report_id":id,"token":UUID().uuidString]),as:UTF8.self)
                if self.mode == .remote {try self.zenoh?.sendAction(kind:"search/report/ack",payload:payload)}else{try self.controller?.autonomyRequest(kind:"search/report/ack",payload:payload,timestamp:self.mode == .simulation ? self.simulatedTime:now)}
                searchReportReceipts[identity]=now
            }catch{DispatchQueue.main.async{self.autonomyReason="Report acknowledgement failed: \(error.localizedDescription)"}}
        }
        searchReportReceipts=searchReportReceipts.filter{CACurrentMediaTime()-$0.value<5}
        let search=TerraTargetSearchPresentation.currentSearch(object,remote:self.mode == .remote)
        let searchPhase=search?["phase"] as? String ?? ""
        let report=search?["report"] as? [String:Any]
        let age=status["age"] as? Double ?? 0
        let available=(status["supported_levels"] as? [String] ?? []).contains("target_search") && (self.mode != .remote || age<0.5)
        DispatchQueue.main.async {
            self.searchAvailable=available
            self.searchPhase=searchPhase
            self.searchTerminal=searchPhase.isEmpty || ["completed","cancelled","timed_out","exhausted"].contains(searchPhase)
            self.searchReportText=report.map{String(format:"Target confirmed at %.2f, %.2f m · %d frames",$0["world_x"] as? Double ?? 0,$0["world_y"] as? Double ?? 0,($0["frame_ids"] as? [Any])?.count ?? 0)} ?? ""
        }
        DispatchQueue.main.async {self.autonomyLevel=level;self.autonomyReason=reason;self.proposalRun=proposal?["run_id"] as? String ?? "";self.proposedGoal=(proposal?["proposal_id"] as? NSNumber)?.uint64Value;self.proposalText=proposal.map{String(format:"Search target %.1f, %.1f m",$0["x"] as? Double ?? 0,$0["y"] as? Double ?? 0)} ?? "";if let goal{self.waypointActive=(goal["state"] as? String)=="active";self.waypointDistance=goal["distance"] as? Double ?? 0;self.waypointStatus=goal["state"] as? String ?? "Unknown"}}
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
    private func fail(_ error: Error, context: String = "Controller") {
        TerraLog.control.error("Controller failure: \(context, privacy: .public): \(error.localizedDescription, privacy: .public)")
        let detail = "\(context): \(error.localizedDescription)"
        #if DEBUG
        let destination = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0].appendingPathComponent("ControllerFailure.json")
        if let data = try? JSONSerialization.data(withJSONObject: ["time": ISO8601DateFormatter().string(from: Date()), "context": context, "error": error.localizedDescription, "mode": String(describing: mode), "systemUptime": ProcessInfo.processInfo.systemUptime, "mediaTime": CACurrentMediaTime()]) {
            try? data.write(to: destination, options: .atomic)
        }
        #endif
        bluetooth.disarm(); bluetooth.disconnect(); bleActive = false
        mode = .stopped; timer?.cancel(); timer = nil; try? controller?.reset()
        zenoh?.disconnect(); zenoh = nil
        goalLatched = false
        remotePose = nil; phonePose = nil
        DispatchQueue.main.async {
            self.waypointActive = false; self.waypointDistance = 0; self.waypointStatus = "No goal"
            self.motion.stopDeviceMotionUpdates(); self.session.pause()
            self.status = detail; self.leftEffort = 0; self.rightEffort = 0
            self.mapStatus = "Map paused"
            self.zenohConnecting = false; self.zenohStatus = error.localizedDescription
            self.dashboardConnecting = false; self.dashboardConnected = false; self.dashboardStatus = "Disconnected · controller stopped"
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
    private func revokeHardwareMotion(reason: String = #function) {
        target = (0, 0); resetServoTargets()
        if !hardwareMotionRevoked {
            TerraLog.control.notice("Revoking motion; caller=\(reason, privacy: .public) mode=\(String(describing: self.mode), privacy: .public) feedback=\(self.bleFeedback)")
            bluetooth.disarm(); hardwareMotionRevoked = true
        }
    }
    private func observeBluetooth() {
        bluetooth.onDiscovery = { [weak self] rover, generation in self?.discovered(rover, generation: generation) }
        bluetooth.onLifecycle = { [weak self] event, generation in self?.bluetoothEvent(event, generation: generation) }
        let timer = DispatchSource.makeTimerSource(queue: .main)
        timer.schedule(deadline: .now() + 1, repeating: .seconds(1))
        timer.setEventHandler { [weak self] in self?.serviceAutomaticConnections() }
        timer.resume(); connectionTimer = timer
        bluetooth.objectWillChange.sink { [weak self] _ in
            DispatchQueue.main.async { self?.refreshBluetooth() }
        }.store(in: &subscriptions)
        bluetooth.$replyJSON.sink { [weak self] text in self?.receiveConfiguration(text) }.store(in: &subscriptions)
    }
    private func refreshBluetooth() {
        defer { refreshConnectionPresentation() }
        if configurationGeneration != bluetooth.connectionGeneration {
            configurationGeneration = bluetooth.connectionGeneration
            actuatorEditor.invalidate(generation: configurationGeneration); editorRequest = nil; retryableEditorRequest = nil; hasRetryableActuatorRequest = false; presetUpload = nil; removeAfterBegin = nil; resetFaultRequest = nil; emergencyResetRequest = nil
            acknowledgedCommitRevision = nil
            configurationStatus = "Connection changed · stage draft again"
        }
        hardwareStatus = bluetooth.status; hardwareReady = bluetooth.isReady
        if let identifier = bluetooth.connectedIdentifier {
            UserDefaults.standard.set(identifier.uuidString, forKey: "preferredHardwareRover")
        }
        hardwareConfigurationReady = bluetooth.configurationReady
        let remoteStatus = (try? JSONSerialization.jsonObject(with: Data(bluetooth.statusJSON.utf8))) as? [String: Any]
        hardwareBenchMode = remoteStatus?["gate_mode"] as? String == "bench"
        hardwareBenchEnabled = remoteStatus?["bench_enabled"] as? Bool == true
        activeLayoutRevision = remoteStatus?["active_revision"] as? UInt32 ?? 0
        hardwareFault = remoteStatus?["fault"] as? String
        if let index = try? JSONDecoder().decode(ActuatorLayoutIndex.self, from: Data(bluetooth.layoutIndexJSON.utf8)) {
            let invalidated = actuatorEditor.generation != configurationGeneration || actuatorEditor.revision != index.revision
            actuatorEditor.synchronize(index: index, generation: configurationGeneration)
            if invalidated { editorRequest = nil; retryableEditorRequest = nil; hasRetryableActuatorRequest = false; presetUpload = nil; removeAfterBegin = nil; actuatorPageLoading = false }
        }

        hardwareArmed = bluetooth.armed; hardwareArming = bluetooth.arming
        capabilitiesJSON = bluetooth.capabilitiesJSON; committedLayoutJSON = bluetooth.driveProfileJSON
        discoveredRovers = (try? JSONDecoder().decode([RoverPeripheral].self, from: Data(bluetooth.peripheralsJSON.utf8))) ?? []
        feedbackCompatible = (try? ActuatorDriveProfileSet.decode(json: committedLayoutJSON).supportsFeedback) ?? false
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
    private var connectionNow: TimeInterval { ProcessInfo.processInfo.systemUptime }
    private func automaticConnectionEnabled() -> Bool {
        UserDefaults.standard.object(forKey: "autoConnectHardware") as? Bool != false
    }
    func autoConnectHardware() {
        #if !targetEnvironment(simulator)
        guard automaticConnectionEnabled(), !connectionPolicy.enabled else { return }
        let preferred = UserDefaults.standard.string(forKey: "preferredHardwareRover").flatMap(UUID.init(uuidString:))
        connectionPolicy.begin(preferred: preferred)
        roverConnectionFailed = false
        routerSuppressed = false; routerRetryAt = nil
        if let identifier = bluetooth.connectedIdentifier, connectionPolicy.adoptAuthenticated(identifier) {
            bluetooth.resumeAutomation(generation: connectionPolicy.generation)
        } else { bluetooth.startDiscovery(generation: connectionPolicy.generation) }
        #endif
    }
    func cancelAutoConnection() {
        if connectionPolicy.attempting != nil { stop(); return }
        connectionPolicy.stop(); pairingCandidate = nil; nearbyPairing.removeAll()
        routerSuppressed = true; routerRetryAt = nil; invalidateDashboardAttempt()
        dashboardConnecting = false
        bluetooth.cancelAutomaticConnection()
    }
    func scanBluetooth() { retryAutomaticConnection() }
    func retryAutomaticConnection() {
        cancelAutoConnection(); autoConnectHardware()
    }
    func disconnectHardware() { stop() }
    func forgetHardwareRover() {
        stop(); UserDefaults.standard.removeObject(forKey: "preferredHardwareRover")
        UserDefaults.standard.removeObject(forKey: "preferredHardwareRoverName")
        roverDisplayName = "Your rover"
        autoConnectHardware()
    }
    private func discovered(_ rover: TerraDiscoveredRover, generation: UInt64) {
        guard generation == connectionPolicy.generation else { return }
        roverNames[rover.id] = rover.name
        if rover.id == connectionPolicy.preferred { roverDisplayName = rover.name }
        nearbyPairing.observe(rover, now: connectionNow)
        if !rover.pairing, pairingCandidate?.id == rover.id {
            pairingCandidate = nil; connectionPolicy.decline(rover.id)
        }
        guard let decision = connectionPolicy.discover(rover) else { return }
        switch decision {
        case .offer: pairingCandidate = rover
        case .connect(let id):
            pairingCandidate = nil; connectionPolicy.accept(id, now: connectionNow)
            connectAutomatically(id)
        }
    }
    func respondToPairing(connect: Bool) {
        guard let rover = pairingCandidate else { return }
        pairingCandidate = nil
        if connect {
            connectionPolicy.accept(rover.id, now: connectionNow); connectAutomatically(rover.id)
        } else {
            connectionPolicy.decline(rover.id)
            for candidate in nearbyPairing.available(now: connectionNow) { discovered(candidate, generation: connectionPolicy.generation) }
        }
    }
    private func connectAutomatically(_ id: UUID) {
        let savedPolicy = connectionPolicy
        startBluetooth(identifier: id, feedback: false)
        connectionPolicy = savedPolicy; routerSuppressed = false
    }
    private func bluetoothEvent(_ event: BluetoothLink.Lifecycle, generation: UInt64) {
        guard generation == connectionPolicy.generation, connectionPolicy.enabled else { return }
        switch event {
        case .paired(let id):
            rememberRoverName(id)
            UserDefaults.standard.set(id.uuidString, forKey: "preferredHardwareRover")
            connectionPolicy.paired(id, now: connectionNow)
            bluetooth.startDiscovery(generation: connectionPolicy.generation)
        case .authenticated(let id):
            roverConnectionFailed = false; rememberRoverName(id)
            UserDefaults.standard.set(id.uuidString, forKey: "preferredHardwareRover")
            connectionPolicy.authenticated(id)
        case .closed(let retryable):
            roverConnectionFailed = true
            pairingCandidate = nil
            disconnectDashboardForLifecycle()
            connectionPolicy.failed(retryable: retryable, now: connectionNow)
        }
    }
    private func serviceAutomaticConnections() {
        defer { refreshConnectionPresentation() }
        if connectionPolicy.enabled {
            if let candidate = pairingCandidate,
               !nearbyPairing.available(now: connectionNow).contains(where: { $0.id == candidate.id }) {
                pairingCandidate = nil; connectionPolicy.decline(candidate.id)
            }
            if connectionPolicy.attemptExpired(now: connectionNow) {
                bluetooth.disconnect(); disconnectDashboardForLifecycle()
                connectionPolicy.failed(retryable: true, now: connectionNow)
            }
            if connectionPolicy.retryDue(now: connectionNow) {
                bluetooth.startDiscovery(generation: connectionPolicy.generation)
            }
        }
        guard bluetooth.connectedIdentifier != nil, bluetooth.configurationReady,
              !routerSuppressed, !dashboardConnected, !dashboardConnecting else { return }
        if let due = routerRetryAt, connectionNow < due { return }
        let defaults = UserDefaults.standard
        let endpoint = defaults.string(forKey: "dashboardRouterEndpoint") ?? ""
        guard !endpoint.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty else {
            dashboardStatus = "Set router endpoint in Settings"; return
        }
        routerRetryAt = nil
        connectDashboard(endpoint: endpoint, prefix: defaults.string(forKey: "dashboardTopicPrefix") ?? "terra/phone", roverID: defaults.string(forKey: "dashboardRoverID") ?? "0")
    }
    @discardableResult func applyDashboardSettings(endpoint: String, prefix: String, roverID: String) -> Bool {
        guard let settings = TerraRouterSettings.parse(endpoint: endpoint, prefix: prefix, roverID: roverID) else { return false }
        let defaults = UserDefaults.standard
        defaults.set(settings.endpoint, forKey: "dashboardRouterEndpoint")
        defaults.set(settings.prefix, forKey: "dashboardTopicPrefix")
        defaults.set(String(settings.roverID), forKey: "dashboardRoverID")
        routerSuppressed = false; routerRetryAt = nil
        disconnectDashboardForLifecycle()
        serviceAutomaticConnections(); refreshConnectionPresentation()
        return true
    }
    func retryFleetConnection() {
        routerSuppressed = false; routerRetryAt = nil
        let defaults = UserDefaults.standard
        let endpoint = defaults.string(forKey: "dashboardRouterEndpoint") ?? ""
        let prefix = defaults.string(forKey: "dashboardTopicPrefix") ?? "terra/phone"
        let id = defaults.string(forKey: "dashboardRoverID") ?? "0"
        connectDashboard(endpoint: endpoint, prefix: prefix, roverID: id)
        refreshConnectionPresentation()
    }
    private func rememberRoverName(_ id: UUID) {
        guard let name = roverNames[id] else { return }
        roverDisplayName = name
        UserDefaults.standard.set(name, forKey: "preferredHardwareRoverName")
    }
    private func refreshConnectionPresentation() {
        let defaults = UserDefaults.standard
        var facts = TerraConnectionFacts()
        facts.radio = bluetooth.radioState
        facts.automatic = connectionPolicy.enabled
        facts.pairingOffered = pairingCandidate != nil
        facts.attempting = connectionPolicy.attempting != nil
        facts.authenticated = bluetooth.connectedIdentifier != nil
        facts.configurationAvailable = hardwareConfigurationReady
        facts.hardwareReady = hardwareReady
        facts.hardwareFault = hardwareFault != nil
        facts.roverFailed = roverConnectionFailed
        facts.fleetConfigured = TerraRouterSettings.parse(endpoint: defaults.string(forKey: "dashboardRouterEndpoint") ?? "", prefix: defaults.string(forKey: "dashboardTopicPrefix") ?? "terra/phone", roverID: defaults.string(forKey: "dashboardRoverID") ?? "0") != nil
        facts.fleetAttempting = dashboardConnecting; facts.fleetConnected = dashboardConnected
        facts.fleetPaused = routerSuppressed; facts.fleetFailed = routerRetryAt != nil
        #if targetEnvironment(simulator)
        facts.simulator = true
        #endif
        synchronizeFleetTracking(facts)
        #if DEBUG && targetEnvironment(simulator)
        // Explicit simulator screenshot fixtures never run on a physical device.
        if let preview = ProcessInfo.processInfo.environment["TERRA_UI_PREVIEW_STATE"] {
            facts = TerraConnectionFacts()
            facts.radio = .ready; facts.automatic = true
            roverDisplayName = "Preview rover"
            switch preview {
            case "connecting": facts.attempting = true; facts.fleetConfigured = true
            case "connected", "attention", "retrying":
                facts.authenticated = true; facts.configurationAvailable = true
                facts.hardwareReady = preview != "attention"; facts.hardwareFault = preview == "attention"
                facts.fleetConfigured = true; facts.fleetConnected = preview == "connected"
                facts.fleetFailed = preview == "retrying"
            default: break
            }
        }
        #endif
        connectionFacts = facts
    }
    private func invalidateDashboardAttempt() {
        dashboardGenerationLock.lock(); dashboardGeneration &+= 1; dashboardGenerationLock.unlock()
    }
    private func dashboardAttemptToken() -> UInt64 {
        dashboardGenerationLock.lock(); defer { dashboardGenerationLock.unlock() }; return dashboardGeneration
    }
    private func dashboardAttemptMatches(_ token: UInt64) -> Bool { dashboardAttemptToken() == token }
    private func disconnectDashboardForLifecycle() {
        invalidateDashboardAttempt()
        controlQueue.async {
            try? self.controller?.disconnectDashboard(); self.goalLatched = false
        }
        dashboardConnected = false; dashboardConnecting = false
        dashboardStatus = "Disconnected"
    }
    func retryPhoneTracking() {
        fleetTrackingFailed = false
        refreshConnectionPresentation()
    }
    private func synchronizeFleetTracking(_ facts: TerraConnectionFacts) {
        let retainingWaypoint = fleetTrackingManaged && TerraFleetManualDrive.localWaypointActive(controller?.autonomyStatus() ?? "{}") && !routerSuppressed
        if facts.shouldTrackForFleet || retainingWaypoint {
            fleetTrackingManaged = true
            let feedback = feedbackCompatible && hardwareReady
            if phoneTrackingActive {
                if hardwareFeedback != feedback {
                    disarmHardware(); hardwareFeedback = feedback
                    controlQueue.async {
                        self.bleFeedback = feedback; self.target = (0, 0); self.resetServoTargets()
                    }
                }
                return
            }
            guard !fleetTrackingFailed else { return }
            let permission = AVCaptureDevice.authorizationStatus(for: .video)
            guard permission != .denied, permission != .restricted else {
                phoneTrackingStatus = "Allow camera access in iPhone Settings to start tracking"
                return
            }
            guard ARWorldTrackingConfiguration.isSupported, motion.isDeviceMotionAvailable else {
                phoneTrackingStatus = "Phone tracking is unavailable on this device"
                return
            }
            disarmHardware()
            startPhoneSensors()
            hardwareFeedback = feedback
            phoneTrackingStatus = "Tracking starting · managed by ARGOS in every mode"
            controlQueue.async {
                self.bleActive = true; self.bleFeedback = feedback
                self.target = (0, 0); self.resetServoTargets()
            }
        } else if fleetTrackingManaged {
            fleetTrackingManaged = false; fleetTrackingFailed = false
            setHardwareFeedback(enabled: false)
            phoneTrackingStatus = "Tracking starts after ARGOS connects"
        }
    }
    func setHardwareFeedback(enabled: Bool) {
        if enabled {
            guard bluetooth.connectedIdentifier != nil else { return }
            guard feedbackCompatible, hardwareReady, ARWorldTrackingConfiguration.isSupported,
                  motion.isDeviceMotionAvailable else {
                configurationStatus = "Feedback requires a compatible layout and available phone tracking"; return
            }
            disarmHardware(); startPhoneSensors(); hardwareFeedback = true
            controlQueue.async { self.bleActive = true; self.bleFeedback = true; self.target = (0, 0); self.resetServoTargets() }
        } else {
            invalidateDashboardAttempt(); dashboardConnecting = false
            sensorEpochLock.lock(); sensorEpoch += 1; sensorEpochLock.unlock()
            motion.stopDeviceMotionUpdates(); session.pause(); disarmHardware()
            locationManager.stopUpdatingLocation(); locationManager.stopUpdatingHeading()
            phoneTrackingActive = false
            hardwareFeedback = false
            controlQueue.async {
                self.bleFeedback = false; self.bleActive = true; self.target = (0, 0)
                self.resetServoTargets(); self.mode = .bluetoothManual; self.startTimer()
            }
            source = "Bluetooth · normalized manual effort"
        }
    }
    func startBluetooth(identifier: UUID, feedback: Bool) {
        let connectionGeneration = connectionPolicy.generation
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
            self.bluetooth.connect(identifier: identifier, generation: connectionGeneration)
            DispatchQueue.main.async { self.source = feedback ? "Bluetooth · waiting for fresh feedback-compatible layout" : "Bluetooth · normalized manual effort" }
        }
    }
    func armHardware(expectedRun: String? = nil, expectedRevision: UInt64? = nil, expiresAt: TimeInterval? = nil) {
        guard requestedHardwareFeedback == nil, hardwareReady, !hardwareArmed, !hardwareArming else { return }
        sensorEpochLock.lock(); let epoch = sensorEpoch; sensorEpochLock.unlock()
        controlQueue.async {
            guard self.sensorEpochMatches(epoch), !self.dashboardEmergencyStop else { return }
            if let expiresAt {
                let current = (try? JSONSerialization.jsonObject(with: Data((self.controller?.autonomyStatus() ?? "{}").utf8))) as? [String: Any]
                let authority = current?["status"] as? [String: Any]
                guard ProcessInfo.processInfo.systemUptime < expiresAt, authority?["run_id"] as? String == expectedRun,
                      (authority?["revision"] as? NSNumber)?.uint64Value == expectedRevision,
                      authority?["safety"] as? String == "clear" else {
                    DispatchQueue.main.async { self.dashboardHardwareResult = "rejected: arm authority expired or changed" }
                    return
                }
            }
            // Navigation stops output while retaining the BLE link. An explicit
            // Arm can resume manual output on that already synchronized connection.
            if !self.bleActive {
                guard self.mode == .stopped else { return }
                self.bleActive = true; self.bleFeedback = false; self.mode = .bluetoothManual
                self.tick = 0; self.startTimer()
                DispatchQueue.main.async {
                    guard self.sensorEpochMatches(epoch), !self.dashboardEmergencyStop else { return }
                    self.hardwareActive = true; self.hardwareFeedback = false
                    self.source = "Bluetooth · normalized manual effort"
                }
            }
            let fullyManual = TerraFleetManualDrive.isFullyManual(self.controller?.autonomyStatus() ?? "{}")
            guard DriveJoystickSafety.hardwareArmAllowed(feedback: self.bleFeedback, fullyManual: fullyManual, trackingHealthy: self.feedbackTrackingHealthy, compatible: self.bleCompatible) else {
                DispatchQueue.main.async { self.configurationStatus = "Feedback tracking is not ready to arm"; self.dashboardHardwareResult = "rejected: feedback tracking not ready" }
                return
            }
            self.target = (0, 0); self.resetServoTargets(); self.hardwareMotionRevoked = false
            TerraLog.control.notice("Arm accepted by phone; fullyManual=\(fullyManual) feedback=\(self.bleFeedback)")
            self.bluetooth.arm()
        }
    }
    func disarmHardware() {
        controlQueue.async { self.revokeHardwareMotion() }
    }
    func setServoTarget(id: UInt8, position: Double) {
        guard position.isFinite, hardwareArmed else { return }
        controlQueue.async {
            guard let layout = try? ActuatorDriveProfileSet.decode(json: self.bleLayout),
                  let actuator = layout.actuators.first(where: { $0.id == Int(id) && $0.kind == "positional_servo" }) else { return }
            let value = min(actuator.limits.max, max(actuator.limits.min, position))
            self.servoTargets[String(id)] = value
            DispatchQueue.main.async { self.servoPositions[id] = value }
        }
    }
    private func resetServoTargets() {
        servoTargets = [:]
        if let layout = try? ActuatorDriveProfileSet.decode(json: bleLayout) {
            for a in layout.actuators where a.kind == "positional_servo" {
                servoTargets[String(a.id)] = min(a.limits.max, max(a.limits.min, a.safe.value ?? 0))
            }
        }
        let values = servoTargets.reduce(into: [UInt8: Double]()) { result, entry in if let id = UInt8(entry.key) { result[id] = entry.value } }
        DispatchQueue.main.async { self.servoPositions = values }
    }
    private func routeHardware(left: Double, right: Double, now: TimeInterval, manualTarget: DriveJoystickCommand? = nil) {
        guard bleActive, !dashboardEmergencyStop, !bleFeedback || bleCompatible else { return }
        do {
            let input: [String: Any] = ["left_effort": left, "right_effort": right, "forward": manualTarget?.forward ?? (bleFeedback ? 0 : target.forward), "turn": manualTarget?.yaw ?? (bleFeedback ? 0 : target.yaw), "servo_positions": servoTargets]
            let routed = try ActuatorDriveProfileSet.decode(json: bleLayout).route(input: input)
            let values = String(decoding: try JSONSerialization.data(withJSONObject: routed), as: UTF8.self)
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
    var hasStagedLayout: Bool { actuatorEditor.canCommit }
    var configurationBusy: Bool { editorRequest != nil || retryableEditorRequest != nil }
    func editActuator(_ actuator: ActuatorDraft) {
        guard !configurationBusy else { return }
        if presetUpload != nil { presetUpload?.updateCurrent(actuator) }
        actuatorEditor.change(actuator)
    }
    func discardLocalActuatorChanges() { guard !configurationBusy else { return }; presetUpload = nil; actuatorEditor.discardLocal() }
    func retryActuatorConfiguration() { guard !configurationBusy else { return }; bluetooth.refreshConfigurationIndex() }
    func retryFailedActuatorRequest() {
        guard editorRequest == nil, let request = retryableEditorRequest,
              request.generation == configurationGeneration else { return }
        retryableEditorRequest = nil; hasRetryableActuatorRequest = false
        editorRequest = request; actuatorPageLoading = request.operation == "read_actuator"
        configurationStatus = "Retrying \(request.operation.replacingOccurrences(of: "_", with: " "))"
        bluetooth.retryConfiguration(requestID: request.id, generation: request.generation)
    }
    func selectActuator(_ id: Int) {
        guard !configurationBusy, actuatorEditor.select(id) else { return }
        if actuatorEditor.selected != nil { return }
        actuatorPageLoading = true
        editorSend("read_actuator", payload: ["expected_revision": actuatorEditor.revision, "actuator_id": id], actuatorID: id)
    }
    private func editorSend(_ operation: String, payload: [String: Any], actuator: ActuatorDraft? = nil, actuatorID: Int? = nil) {
        guard !configurationBusy, let id = sendConfiguration(operation, payload: payload) else { return }
        editorRequest = (id, operation, configurationGeneration, actuator, actuatorID)
        configurationStatus = "Waiting for \(operation.replacingOccurrences(of: "_", with: " ")) acknowledgement"
    }
    private func beginEditor(replacement: Bool = false) {
        beginReplacement = replacement
        editorSend("begin_layout_edit", payload: ["expected_revision": actuatorEditor.revision, "mode": replacement ? "replace" : "existing"])
    }
    func saveSelectedActuator() {
        guard configurationAllowed, !configurationBusy, let actuator = actuatorEditor.selected else { return }
        if actuatorEditor.token == nil { beginEditor(); return }
        stageOne(actuator)
    }
    private func stageOne(_ actuator: ActuatorDraft) {
        guard let token = actuatorEditor.token,
              let data = try? JSONEncoder().encode(actuator),
              let object = try? JSONSerialization.jsonObject(with: data) else { return }
        actuatorEditor.change(actuator)
        editorSend("stage_actuator", payload: ["edit_token": token, "edit_version": actuatorEditor.version, "actuator": object], actuator: actuator)
    }
    func addActuator() {
        guard !configurationBusy, !actuatorEditor.dirty, actuatorEditor.entries.count < 16,
              let id = (0...255).first(where: { id in !actuatorEditor.entries.contains(where: { $0.id == id }) }) else { return }
        actuatorEditor.change(ActuatorDraft(id: id, name: "Actuator \(id)"))
    }
    func removeSelectedActuator() {
        guard configurationAllowed, !configurationBusy, let id = actuatorEditor.selectedID else { return }
        presetUpload = nil
        guard let token = actuatorEditor.token else { removeAfterBegin = id; beginEditor(); return }
        editorSend("remove_actuator", payload: ["edit_token": token, "edit_version": actuatorEditor.version, "actuator_id": id], actuatorID: id)
    }
    func uploadPreset(_ actuators: [ActuatorDraft]) {
        guard configurationAllowed, !configurationBusy, !actuatorEditor.dirty else { return }
        presetUpload = ActuatorPresetUpload(actuators: actuators); beginEditor(replacement: true)
    }
    private func presentNextPresetActuator() {
        guard let upload = presetUpload, let actuator = upload.current else { presetUpload = nil; configurationStatus = "Preset saved to draft · validate before applying"; return }
        if upload.needsPortSelection {
            actuatorEditor.change(actuator)
            configurationStatus = "Choose a physical port for \(actuator.name), then save to continue the preset"
        } else { stageOne(actuator) }
    }
    func validateActuatorDraft() {
        guard configurationAllowed, !configurationBusy, !actuatorEditor.dirty, let token = actuatorEditor.token else { return }
        editorSend("validate_layout_edit", payload: ["edit_token": token, "edit_version": actuatorEditor.version])
    }
    func discardActuatorDraft() {
        guard editorRequest == nil, let token = actuatorEditor.token else { return }
        retryableEditorRequest = nil; hasRetryableActuatorRequest = false; presetUpload = nil
        editorSend("discard_layout_edit", payload: ["edit_token": token])
    }
    func commitActuatorLayout() {
        guard configurationAllowed, !configurationBusy, actuatorEditor.canCommit, let token = actuatorEditor.token else { return }
        editorSend("commit_layout_edit", payload: ["edit_token": token, "edit_version": actuatorEditor.version, "base_revision": actuatorEditor.revision])
    }
    private func receiveConfiguration(_ text: String) {
        guard let object = (try? JSONSerialization.jsonObject(with: Data(text.utf8))) as? [String: Any], let id = object["request_id"] as? UInt32 else { return }
        TerraLog.configuration.info("Reply request=\(id) result=\(String(describing: object["result"]), privacy: .public)")
        if id == emergencyResetRequest {
            emergencyResetRequest = nil
            guard object["result"] as? String == "ok" else {
                configurationStatus = "Emergency stop reset rejected"; return
            }
            configurationStatus = "Emergency stop reset acknowledged · remains disarmed"
            sensorEpochLock.lock(); let epoch = sensorEpoch; sensorEpochLock.unlock()
            controlQueue.async {
                guard self.sensorEpochMatches(epoch), self.mode == .bluetoothManual, self.dashboardEmergencyStop else { return }
                // Explicit rover acknowledgement permits clearing this manual runtime's remote stop.
                try? self.controller?.reset(); self.dashboardEmergencyStop = false
                self.target = (0, 0); self.resetServoTargets()
                DispatchQueue.main.async {
                    guard self.sensorEpochMatches(epoch) else { return }
                    self.invalidateDashboardAttempt(); self.dashboardConnected = false; self.dashboardConnecting = false
                    self.routerRetryAt = nil; self.autonomyReason = "Disarmed"
                }
            }
            return
        }
        if id == resetFaultRequest {
            resetFaultRequest = nil
            configurationStatus = object["result"] as? String == "ok" ? "Fault reset acknowledged · arm explicitly when ready" : "Fault reset rejected: \(object["errors"] ?? [])"
            return
        }
        guard let request = editorRequest, request.id == id, request.generation == configurationGeneration else { return }
        editorRequest = nil; actuatorPageLoading = false
        guard object["result"] as? String == "ok" else {
            let errors = object["errors"] as? [[String: Any]] ?? []
            if errors.contains(where: { ["configuration_timeout", "configuration_transfer"].contains($0["code"] as? String ?? "") }) {
                retryableEditorRequest = request; hasRetryableActuatorRequest = true
                configurationStatus = "Acknowledgement unavailable · retry this operation to recover its exact result"
                return
            }
            retryableEditorRequest = nil; hasRetryableActuatorRequest = false
            if request.operation != "stage_actuator" { presetUpload = nil }
            removeAfterBegin = nil; actuatorEditor.validatedVersion = nil
            configurationStatus = "Rejected · local changes retained: \(object["errors"] ?? [])"; return
        }
        retryableEditorRequest = nil; hasRetryableActuatorRequest = false
        let payload = object["payload"] as? [String: Any] ?? [:]
        switch request.operation {
        case "read_actuator":
            if let revision = payload["revision"] as? UInt32, let object = payload["actuator"],
               let data = try? JSONSerialization.data(withJSONObject: object), let actuator = try? JSONDecoder().decode(ActuatorDraft.self, from: data) {
                _ = actuatorEditor.acceptPage(actuator, revision: revision, generation: request.generation)
            }
        case "begin_layout_edit":
            guard let token = payload["edit_token"] as? String, let base = payload["base_revision"] as? UInt32,
                  base == actuatorEditor.revision, let version = payload["edit_version"] as? UInt32 else { return }
            actuatorEditor.begin(token: token, base: base, version: version, replacement: beginReplacement)
            if let id = removeAfterBegin {
                removeAfterBegin = nil
                editorSend("remove_actuator", payload: ["edit_token": token, "edit_version": version, "actuator_id": id], actuatorID: id)
            } else if presetUpload != nil { presentNextPresetActuator(); return }
            else if let actuator = actuatorEditor.selected, actuatorEditor.dirty { stageOne(actuator) }
        case "stage_actuator", "remove_actuator", "validate_layout_edit":
            guard payload["edit_token"] as? String == actuatorEditor.token,
                  let version = payload["edit_version"] as? UInt32 else { return }
            if request.operation == "validate_layout_edit" {
                guard version == actuatorEditor.version else { return }; actuatorEditor.validatedVersion = version
            } else {
                guard version == actuatorEditor.version + 1 else { return }
                if let actuator = request.actuator { actuatorEditor.acknowledge(actuator, version: version) }
                if request.operation == "remove_actuator", let id = request.actuatorID { actuatorEditor.removed(id, version: version) }
                if request.operation == "stage_actuator", let actuator = request.actuator, presetUpload != nil {
                    guard presetUpload?.acknowledge(id: actuator.id) == true else { return }
                    presentNextPresetActuator(); return
                }
            }
        case "commit_layout_edit":
            guard let revision = payload["revision"] as? UInt32 else { return }
            acknowledgedCommitRevision = revision; actuatorEditor = ActuatorPaginationState()
            disarmHardware(); bluetooth.refreshConfigurationIndex()
        case "discard_layout_edit":
            presetUpload = nil
            actuatorEditor = ActuatorPaginationState(); bluetooth.refreshConfigurationIndex()
        default: break
        }
        configurationStatus = "\(request.operation.replacingOccurrences(of: "_", with: " ")) acknowledged"

    }
}

extension PhoneController {
    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        guard let fix = locations.last else { return }
        controlQueue.async { if self.mode == .phone { self.locationFix = fix } }
    }
    func locationManager(_ manager: CLLocationManager, didUpdateHeading heading: CLHeading) {
        controlQueue.async { if self.mode == .phone { self.locationHeading = heading } }
    }
    func locationManager(_ manager: CLLocationManager, didFailWithError error: Error) {
        controlQueue.async { self.locationFix = nil; self.locationHeading = nil }
    }
    private func publishLocalization(_ controller: MobileController) {
        let now = Date().timeIntervalSince1970
        guard now - lastLocalizationPublish >= 1 else { return }
        lastLocalizationPublish = now
        let report = localizationPolicy.update(PhoneLocalizationInput(now: now,
            latitude: locationFix?.coordinate.latitude, longitude: locationFix?.coordinate.longitude,
            fixTime: locationFix?.timestamp.timeIntervalSince1970, accuracy: locationFix?.horizontalAccuracy,
            heading: locationHeading?.trueHeading, headingAccuracy: locationHeading?.headingAccuracy,
            headingTime: locationHeading?.timestamp.timeIntervalSince1970,
            localX: phonePose?.x ?? 0, localY: phonePose?.y ?? 0, topYaw: phoneTopYaw, tracked: localizationTracked && previousPoseTime.map { CACurrentMediaTime() - $0 < 1 } == true))
        if let data = try? JSONEncoder().encode(report), let json = String(data: data, encoding: .utf8) {
            try? controller.publishLocalization(payload: json)
        }
    }
}
