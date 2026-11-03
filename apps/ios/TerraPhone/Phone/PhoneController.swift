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
    @Published var source = "Stopped"
    @Published private(set) var status = "Motor output disabled"
    @Published private(set) var measuredForward = 0.0
    @Published private(set) var measuredYaw = 0.0
    @Published private(set) var leftEffort = 0.0
    @Published private(set) var rightEffort = 0.0
    @Published private(set) var benchmark = "Tests feedback control under load and saturation."
    @Published var occupancy: OccupancyGrid?
    @Published var mapStatus = "Start Simulated rover to build a map"
    @Published var mapPose = SIMD3<Double>.zero
    @Published var calibrationStatus = "Mount measurements not saved"
    @Published var offsetFitStatus = "Offset fit not started"
    @Published var fittedCameraOffset: SIMD2<Double>?
    @Published var offsetFitInProgress = false
    var mountCalibration = MountCalibration.load()
    var isCollectingOffsetFit = false
    var offsetFitSamples: [RotationOffsetSample] = []
    var firstTurnFit: RotationOffsetFit?
    var occupancyMap: MobileOccupancyMap?
    /// Control-queue copy of whether the classification view wants labelled frames;
    /// re-applied to every new map.
    var classificationDebugEnabled = false
    private let plant = SimulatedPlant()
    var mapGroundOffset: Double?
    var groundPolicy = PhoneGroundPolicy()
    var lastMapTime = -Double.infinity
    @Published var dashboardStatus = "Disconnected"
    @Published var dashboardConnecting = false
    @Published var dashboardConnected = false
    @Published var fpvStatus = "Idle"
    @Published var fpvLive = false
    @Published var fpvActive = false
    @Published var waypointActive = false
    @Published var waypointStatus = "No goal"
    @Published var waypointDistance = 0.0
    @Published var objectDetectorStatus = "Requires LiDAR, camera tracking and scene depth"
    @Published var searchAvailable=false
    @Published var searchPhase=""
    @Published var searchReportText=""
    @Published var searchReportHistory:[String]=[]
    var searchReportReceipts:[String:Double]=[:]
    @Published var searchTerminal=true
    @Published var autonomyLevel="teleop"
    @Published var autonomyReason="Idle"
    @Published var explorationSummary=""
    @Published var runLog:URL?
    @Published var proposedGoal:UInt64?
    var proposalRun=""
    @Published var proposalText=""
    var phonePose: (x: Double, y: Double, yaw: Double)?
    enum Mode { case stopped, simulation, phone, bluetoothManual }
    let bluetooth = BluetoothLink()
    @Published var hardwareActive = false
    @Published var hardwareFeedback = false
    @Published var phoneTrackingActive = false
    /// Selected rover profile; Rust owns its dimensions and drive parameters.
    @Published var roverProfile = RoverProfiles.selected
    @Published var localizationReport: PhoneLocalizationReport?
    @Published var phoneTrackingStatus = "Tracking starts after ARGOS connects"
    var fleetTrackingManaged = false
    var fleetTrackingFailed = false
    @Published var hardwareReady = false
    @Published var hardwareConfigurationReady = false
    @Published var activeLayoutRevision: UInt32 = 0
    @Published var hardwareFault: String?
    @Published var connectionFacts = TerraConnectionFacts()
    @Published var savedRovers = TerraSavedRovers()
    @Published var roverDisplayName = UserDefaults.standard.string(forKey: "preferredHardwareRoverName") ?? "Your rover"
    var roverNames: [UUID: String] = [:]
    var roverConnectionFailed = false
    @Published var pairingCandidate: TerraDiscoveredRover?
    var connectionPolicy = TerraConnectionPolicy()
    var connectionTimer: DispatchSourceTimer?
    var nearbyPairing = TerraPairingCandidates()
    var routerRetryAt: TimeInterval?
    var routerSuppressed = false
    let dashboardGenerationLock = NSLock()
    var dashboardGeneration: UInt64 = 0
    var configurationGeneration: UInt64 = 0
    var resetFaultRequest: UInt32?
    @Published var hardwareOutputActive = false
    /// Operator confirmed the phone is on the rover mount. Resets on launch.
    @Published var phoneMounted = false
    // Control-queue copy of `phoneMounted`, read on every motor command.
    var phoneMountConfirmed = false
    var mountHoldReported = false
    static let mountHoldReason = "Phone mount not confirmed · only the debug joystick can drive"
    @Published var hardwareStatus = "Disconnected"
    @Published private(set) var lastStopDiagnostic = TerraStopDiagnostic()
    @Published var capabilitiesJSON = "{}"
    @Published var committedLayoutJSON = "{}"
    @Published var configurationStatus = "No staged layout"
    @Published var servoPositions: [UInt8: Double] = [:]
    @Published var dumpGate = DumpGateSetting.load()
    @Published var dumpGateStatus = ""
    @Published var feedbackCompatible = false
    @Published var vioControlCompatible = false
    var requestedHardwareFeedback: UUID?
    var feedbackConnectionSawUnready = false
    var subscriptions = Set<AnyCancellable>()
    let routerQueue = DispatchQueue(label: "terra.router.connection")
    var bleActive = false
    var bleFeedback = false
    var bleLayout = "{}"
    var servoTargets: [String: Double] = [:]
    /// Controller and layout last handed to Rust's actuator routing (shadow mode).
    var rustLayoutSync: (controller: ObjectIdentifier, layout: String)?
    var lastShadowMismatchLog: TimeInterval = 0
    @Published var actuatorEditor = ActuatorPaginationState()
    @Published var actuatorPageLoading = false
    typealias EditorRequest = (id: UInt32, operation: String, generation: UInt64, actuator: ActuatorDraft?, actuatorID: Int?)
    var editorRequest: EditorRequest?
    var retryableEditorRequest: EditorRequest?
    @Published var hasRetryableActuatorRequest = false
    var presetUpload: ActuatorPresetUpload?
    var beginReplacement = false
    var removeAfterBegin: Int?
    var nextRequest: UInt32 = max(1000, UInt32(clamping: UserDefaults.standard.integer(forKey: "hardwareConfigurationRequestID")))
    var sensorEpoch = 0
    let sensorEpochLock = NSLock()
    /// Operator or router e-stop latch. Survives stop() and restarts; only an explicit reset clears it.
    var dashboardEmergencyStop = false {
        didSet { let latched = dashboardEmergencyStop; DispatchQueue.main.async { self.emergencyStopLatched = latched } }
    }
    @Published private(set) var emergencyStopLatched = false
    let objectDetector = ObjectSearchDetector()
    /// On-phone language model for L5 tasks; nil when this phone has none.
    private let taskModel = TaskModelRunner.onDevice()
    /// Qwen 3.5 2B as the on-phone MCP agent for L5 tasks; loads on the first phone start.
    let taskAgent = TaskAgentRunner()
    var detectorHealth = "not started"
    var feedbackTrackingHealthy = false
    var phoneFrameOrder = PhoneFrameOrder()
    var bleReady = false
    var bleCompatible = false
    var phoneStartedAt = 0.0
    let controlQueue = DispatchQueue(label: "terra.control", qos: .userInteractive)
    /// ARKit delivers frames here, not on the control queue, so a slow control step cannot
    /// back up frames in GCD. Only the newest frame waits for the control queue; older ones are dropped.
    private let arFrameQueue = DispatchQueue(label: "terra.ar-frames", qos: .userInteractive, autoreleaseFrequency: .workItem)
    let arFrameLock = NSLock()
    var pendingARFrame: ARFrame?
    var pendingARFrameDelivered = Double.nan
    var arFrameScheduled = false
    var droppedARFrames = 0
    /// Newest IMU and VIO stamps handed to Rust; control queue only. For SensorTimingProbe ticks.
    var timingLastImu = Double.nan, timingLastVio = Double.nan, timingVioPushedAt = Double.nan
    let fpvVideo = FpvVideoCall()
    /// Read from the AR frame queue as well as the control queue.
    var fpvRunning: Bool {
        get { fpvCameraLock.lock(); defer { fpvCameraLock.unlock() }; return fpvRunningFlag }
        set { fpvCameraLock.lock(); fpvRunningFlag = newValue; fpvCameraLock.unlock() }
    }
    private var fpvRunningFlag = false
    var fpvFailure: String?
    let fpvCameraLock = NSLock()
    var sharedCameraForFpv = false
    private let motionQueue = OperationQueue()
    let motion = CMMotionManager()
    let session = ARSession()
    let locationManager = CLLocationManager()
    var locationPermissionReason: String?
    var lastLocalizationReason = ""
    var locationFix: CLLocation?
    var locationHeading: CLHeading?
    var outdoorFixTime: Date?
    var outdoorHeadingTime: Date?
    private static let connectivityPolicy: PhoneConnectivityPolicy = {
        let decoder = JSONDecoder(); decoder.keyDecodingStrategy = .convertFromSnakeCase
        // Native defaults are the single source; a broken binding must fail during development.
        return try! decoder.decode(PhoneConnectivityPolicy.self, from: Data(defaultConnectivityPolicySettings().utf8))
    }()
    var localizationPolicy = PhoneLocalizationPolicy(displayFreshnessSeconds: PhoneController.connectivityPolicy.gpsDisplayFreshnessSeconds)
    var localizationTracked = false
    var localizationTrackingReason: String?
    var phoneHeadingReferenceYaw: Double?
    var lastLocalizationPublish = -Double.infinity
    let observationCapture = PhoneObservationCapture()
    let occupancySettings = defaultOccupancySettings()
    let observation = TerraObservation()
    private let observationQueue = DispatchQueue(label: "terra.observation.startup", qos: .utility)
    var controller: MobileController? {
        didSet { if let current = controller { configureObservability(current, primary: false) } }
    }
    private func configureObservability(_ current: MobileController, primary: Bool) {
        let rover = UserDefaults.standard.string(forKey: "dashboardRoverID") ?? "_producer"
        let directory = FileManager.default.urls(for: .applicationSupportDirectory, in: .userDomainMask)[0].appendingPathComponent("Observability", isDirectory: true)
        let producer = "phone-" + (UserDefaults.standard.string(forKey: "observationProducerID") ?? {
            let id = UUID().uuidString; UserDefaults.standard.set(id, forKey: "observationProducerID"); return id
        }())
        let observer = observation
        observationQueue.async {
            do {
                var config: [String: Any] = ["path": directory.appendingPathComponent("events.db").path, "producer_id": producer, "rover_id": rover]
                if let settingsData = try? Data(contentsOf: directory.appendingPathComponent("settings.json")), settingsData.count <= 65536,
                   let settings = try? JSONSerialization.jsonObject(with: settingsData) as? [String: Any] { config["settings"] = settings }
                let raw = String(decoding: try JSONSerialization.data(withJSONObject: config), as: UTF8.self)
                try current.startObservability(configJson: raw)
                if let settings = try? JSONSerialization.jsonObject(with: Data(current.observabilitySettings().utf8)) as? [String: Any],
                   let evidenceSeconds = settings["evidence_sample_seconds"] as? Double, let mapSeconds = settings["map_sample_seconds"] as? Double {
                    self.observationCapture.configure(evidenceSeconds: evidenceSeconds, mapSeconds: mapSeconds)
                }
                if primary { observer.attach(roverID: rover, sessionID: current.observationRunId(), sink: { current.recordObservation(eventJson: $0) }) }
                observer.record(component: "phone", kind: "lifecycle.controller_attached", attributes: [:])
                if primary, let upload = ObservationUploadSettings.load(directory: directory, producer: producer) {
                    do { try current.startObservabilityUpload(endpoint: upload.endpoint, token: upload.token) }
                    catch { observer.record(component: "observability", kind: "upload.configuration_failed", attributes: [:]) }
                } else if primary { observer.record(component: "observability", kind: "upload.unconfigured", attributes: [:]) }
            } catch { TerraLog.control.error("Observability storage unavailable; control continues") }
        }
    }
    private var timer: DispatchSourceTimer?
    var mode: Mode = .stopped
    var target = (forward: 0.0, yaw: 0.0)
    private var lastLocalTarget = (forward: 0.0, yaw: 0.0)
    var lastCloudTime = -Double.infinity
    var lastFloorTime = -Double.infinity
    // Control-queue state; the mesh walk itself runs on floorQueue. The epoch
    // changes whenever the map or ground offset resets, so a walk that was in
    // flight then is dropped instead of installing a floor at the old offset.
    var floorUpdateInFlight = false
    var floorEpoch = 0
    let floorQueue = DispatchQueue(label: "terra.floor-mesh", qos: .utility)
    var tick = 0
    // Publication identity must survive prepare/preserveRuntime and UI tick resets.
    var occupancyPublicationRevision: UInt64 = 0
    var simulatedTime = 0.0
    var previousPosition: SIMD3<Float>?
    var previousPoseTime: Double?
    var filteredVelocity = SIMD3<Float>.zero
    // ARKit world +Y up -> robotics world +Z up, preserving right handedness.
    let worldFromAR = simd_float3x3(columns: (SIMD3(0, -1, 0), SIMD3(0, 0, 1), SIMD3(-1, 0, 0)))
    override init() {
        super.init()
        // This object is used only as a journal API; it never steps or controls hardware.
        if let journal = try? MobileController(settings: defaultControlSettings()) {
            configureObservability(journal, primary: true)
        }
        bluetooth.observe(using: observation)
        locationManager.delegate = self
        locationManager.desiredAccuracy = kCLLocationAccuracyBest
        locationManager.distanceFilter = kCLDistanceFilterNone
        locationManager.headingFilter = kCLHeadingFilterNone
        locationManager.headingOrientation = PhoneMountGeometry.headingReference(deviceToBody: mountCalibration.deviceToBody).orientation
        motionQueue.maxConcurrentOperationCount = 1
        motionQueue.qualityOfService = .userInteractive
        session.delegate = self
        session.delegateQueue = arFrameQueue
        fpvVideo.onEvent = { [weak self] event in
            self?.controlQueue.async { self?.handleFpvEvent(event) }
        }
        observeBluetooth()
    }
    func setTarget(forward: Double, yaw: Double) {
        controlQueue.async { self.target = self.dashboardEmergencyStop ? (0, 0) : (forward, yaw) }
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
        startPhoneSensors()
    }
    func startPhoneSensors() {
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
        TerraLog.tracking.notice("Phone mount: upright portrait, top edge up, rear camera forward; AR optical -Z and phone -Z map to rover forward")
        configuration.worldAlignment = .gravity
        configuration.planeDetection = [.horizontal]
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) { configuration.frameSemantics.insert(.sceneDepth) }
        // Classified mesh (LiDAR only): its floor faces veto false obstacles in Rust.
        if ARWorldTrackingConfiguration.supportsSceneReconstruction(.meshWithClassification) { configuration.sceneReconstruction = .meshWithClassification }
        locationManager.requestWhenInUseAuthorization()
        locationManager.startUpdatingLocation()
        if CLLocationManager.headingAvailable() { locationManager.startUpdatingHeading() }
        session.run(configuration, options: [.resetTracking, .removeExistingAnchors])
        motion.deviceMotionUpdateInterval = 0.01
        motion.startDeviceMotionUpdates(using: .xArbitraryZVertical, to: motionQueue) { [weak self] data, error in
            let delivered = CACurrentMediaTime()
            guard let self else { return }
            if let error { self.controlQueue.async { if self.sensorEpochMatches(epoch) { self.handleTrackingFailure(error) } }; return }
            guard let data else { return }
            let ua = data.userAcceleration, rate = data.rotationRate
            guard [ua.x, ua.y, ua.z, rate.x, rate.y, rate.z].allSatisfy(\.isFinite) else { return }
            // Core Motion user acceleration is in g. Android linear acceleration is already m/s².
            let deviceAcceleration = SIMD3(Float(ua.x), Float(ua.y), Float(ua.z)) * 9.80665
            let deviceRate = SIMD3(Float(rate.x), Float(rate.y), Float(rate.z))
            self.controlQueue.async {
                let dequeued = CACurrentMediaTime()
                guard self.mode == .phone, self.sensorEpochMatches(epoch) else { return }
                do {
                    // Upright, rear-camera-forward mount, the same mapping ARKit poses use.
                    let toBody = self.mountCalibration.deviceToBody
                    let a = toBody.act(deviceAcceleration)
                    let g = toBody.act(deviceRate)
                    try self.controller?.pushImu(sample: ImuReading(timestamp: data.timestamp, accelerationForward: Double(a.x), accelerationLeft: Double(a.y), accelerationUp: Double(a.z), gyroRoll: Double(g.x), gyroPitch: Double(g.y), gyroYaw: Double(g.z)))
                    self.timingLastImu = data.timestamp
                    SensorTimingProbe.shared.record(.imu, sensor: data.timestamp, os: delivered, queue: dequeued, done: CACurrentMediaTime())
                } catch { self.fail(error, context: "IMU sample at \(data.timestamp)") }
            }
        }
        phoneTrackingActive = true
        updateSharedCamera(true)
        source = "Core Motion + ARKit"
    }
    /// keepBluetooth leaves the rover link up for the actuator layout screen,
    /// which is pushed from this root and would otherwise drop on navigation.
    func stop(keepBluetooth: Bool = false) {
        observation.record(component: "phone", kind: "lifecycle.stop", attributes: ["keep_bluetooth": keepBluetooth])
        connectionPolicy.stop(); pairingCandidate = nil; nearbyPairing.removeAll()
        routerSuppressed = true; routerRetryAt = nil; invalidateDashboardAttempt()
        dashboardConnecting = false
        bluetooth.cancelAutomaticConnection()
        sensorEpochLock.lock(); sensorEpoch += 1; sensorEpochLock.unlock()
        bluetooth.stopOutput()
        if !keepBluetooth { bluetooth.disconnect() }
        hardwareActive = false; requestedHardwareFeedback = nil
        phoneTrackingActive = false
        setSharedCameraFlag(false)
        hardwareFeedback = false
        localizationReport = nil
        fleetTrackingManaged = false; fleetTrackingFailed = false
        phoneTrackingStatus = "Tracking starts after ARGOS connects"
        motion.stopDeviceMotionUpdates()
        locationManager.stopUpdatingLocation(); locationManager.stopUpdatingHeading()
        session.pause()
        controlQueue.async {
            self.closeFpv(publish: true)
            self.timer?.cancel(); self.timer = nil
            self.mode = .stopped
            self.isCollectingOffsetFit = false; self.offsetFitSamples.removeAll(); self.firstTurnFit = nil
            DispatchQueue.main.async { self.offsetFitInProgress = false }
            self.bleActive = false; self.bleReady = false; self.bleFeedback = false; self.feedbackTrackingHealthy = false; self.target = (0, 0); self.servoTargets = [:]
            self.phonePose = nil
            try? self.controller?.reset()
            self.controller = nil
            DispatchQueue.main.async {
                self.source = "Stopped"; self.status = "Motor output disabled"
                self.mapStatus = "Map paused"
                self.waypointActive = false; self.waypointDistance = 0; self.waypointStatus = "No goal"
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
        observation.record(component: "phone", kind: "lifecycle.prepare", attributes: ["mode": String(describing: mode)])
        let preserveRuntime = mode == .phone && self.mode == .bluetoothManual && self.controller != nil
        if !preserveRuntime {
            try? self.controller?.reset()
            self.controller = try makeController()
            DispatchQueue.main.async {
                self.dashboardConnecting = false; self.dashboardConnected = false; self.dashboardStatus = "Disconnected"
            }
        }
        if !preserveRuntime {
                let log=FileManager.default.urls(for:.documentDirectory,in:.userDomainMask)[0].appendingPathComponent("Terra-run-\(UUID().uuidString).jsonl")
                try self.controller?.beginRecording(path:log.path,runId:UUID().uuidString)
                DispatchQueue.main.async {self.runLog=log}
        }
        if mode == .phone { try self.controller?.resetOutdoorFrame() }
        // L5 is offered only when the phone can run a model.
        try self.controller?.setTaskModel(name: mode == .phone ? taskModel?.model.name : nil)
        try self.controller?.setTaskAgent(name: mode == .phone ? taskAgent.model?.name : nil, inProcess: true)
        if mode == .phone && taskAgent.model == nil { loadTaskAgent() }
        self.occupancyMap = try MobileOccupancyMap(settings: self.occupancySettings)
        self.floorEpoch &+= 1
        if let map = self.occupancyMap { try self.controller?.attachOccupancyMap(map: map); map.setClassificationDebug(enabled: classificationDebugEnabled) }
        mapGroundOffset = nil; groundPolicy.reset(); lastMapTime = -Double.infinity
        DispatchQueue.main.async { self.occupancy = nil; self.mapPose = .zero; self.mapStatus = mode == .simulation ? "Simulated depth · 10 Hz" : (ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) ? "Waiting for scene depth" : "Scene depth unavailable on this device") }
        self.mode = mode
        phoneStartedAt = ProcessInfo.processInfo.systemUptime
        target = (0, 0)
        tick = 0; simulatedTime = 0
        if mode == .simulation { try plant.reset() }
        previousPosition = nil; previousPoseTime = nil; filteredVelocity = .zero
        DispatchQueue.main.async { self.localizationReport = nil }
        localizationTrackingReason = nil
        outdoorFixTime = nil; outdoorHeadingTime = nil
        localizationPolicy.reset(); locationFix = nil; locationHeading = nil; localizationTracked = false; phoneHeadingReferenceYaw = nil; lastLocalizationPublish = -Double.infinity
    }
    func startTimer() {
        self.timer?.cancel(); self.timer = nil
        let timer = DispatchSource.makeTimerSource(queue: controlQueue)
        timer.schedule(deadline: .now(), repeating: .milliseconds(10), leeway: .milliseconds(1))
        timer.setEventHandler { [weak self] in self?.update() }
        self.timer = timer
        timer.resume()
    }
    private func update() {
        if mode == .bluetoothManual {
            let now = CACurrentMediaTime()
            do {
                if controller == nil { controller = try makeController() }
                guard let controller else { return }
                try controller.setActuationMode(feedback: false)
                // Without a fleet session this is the phone's own debug joystick.
                let localDebug = controller.dashboardStatus() != "connected"
                if localDebug {
                    try controller.setManualInput(forward: target.forward, turn: target.yaw, timestamp: now)
                }
                syncRustLayout(controller)
                let output = try controller.step(timestamp: now)
                applyGateServo(output.gateServo)
                if tick % 10 == 0 { serviceManualDashboard(); updateAuthority() }
                tick += 1
                if tick % 5 == 0 {
                    let command = DriveJoystickCommand(forward: (output.leftEffort + output.rightEffort) / 2, yaw: (output.rightEffort - output.leftEffort) / 2)
                    routeHardware(left: output.leftEffort, right: output.rightEffort, now: ProcessInfo.processInfo.systemUptime, manualTarget: command, localDebug: localDebug, rustFrame: output.actuatorFrame)
                }
            } catch { fail(error, context: "Shared runtime manual drive") }
            return
        }
        guard let controller, mode != .stopped else { return }
        let now = mode == .simulation ? simulatedTime : CACurrentMediaTime()
        do {
            if mode == .simulation {
                let pose = try plant.state()
                try controller.pushImu(sample: ImuReading(timestamp: now, accelerationForward: pose.accelerationForward, accelerationLeft: pose.forward * pose.yawRate, accelerationUp: 0, gyroRoll: 0, gyroPitch: 0, gyroYaw: pose.yawRate))
                if tick % 5 == 0 {
                    try controller.pushVio(sample: VioReading(timestamp: now, positionX: pose.x, positionY: pose.y, positionZ: 0, quaternionX: pose.quaternionX, quaternionY: pose.quaternionY, quaternionZ: pose.quaternionZ, quaternionW: pose.quaternionW, velocityX: pose.velocityX, velocityY: pose.velocityY, velocityZ: 0, tracked: true))
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
            let direct = bleActive && !bleFeedback
            try controller.setActuationMode(feedback: !direct)
            syncRustLayout(controller)
            let output = try controller.step(timestamp: now)
            applyGateServo(output.gateServo)
            if mode == .phone {
                SensorTimingProbe.shared.record(.tick, sensor: timingLastImu, queue: now, done: CACurrentMediaTime(), a: timingLastVio, b: output.safety == .active ? 1 : 0)
            }
            if tick % 5 == 0 { drainFpvSignaling() }
            if tick % 10 == 0 && controller.dashboardStatus() == "failed" {
                self.closeFpv(publish: false)
                let continuingMission = controller.acceptedOnboardMission()
                observation.record(component: "router", kind: "connection.lost", attributes: ["onboard_mission_retained": continuingMission])
                try controller.handleDashboardLinkLoss()
                // Remote operator intent is cleared natively; no cloud-only output latch.
                self.target = (0, 0)
                DispatchQueue.main.async {
                    self.dashboardConnecting = false; self.dashboardConnected = false
                    self.dashboardStatus = continuingMission ? "Router interrupted · continuing onboard mission" : "Router connection lost · retrying"
                    self.routerRetryAt = self.connectionNow + self.connectionPolicy.nextRouterDelay()
                }
            }
            if mode == .simulation {
                let pose = try plant.state()
                if tick % 10 == 0 { try updateSimulatedMap(x: pose.x, y: pose.y, yaw: pose.yaw, timestamp: now) }
                _ = try plant.step(leftEffort: output.leftEffort, rightEffort: output.rightEffort)
                simulatedTime += 0.01
            }
            tick += 1
            let autonomy = controller.autonomyBrief()
            let fullyManual = TerraFleetManualDrive.isFullyManual(autonomy)
            let vioWaypoint = TerraFleetManualDrive.requiresAutonomousControl(autonomy)
            let vioCompatible = (try? ActuatorDriveProfileSet.decode(json: bleLayout).supportsVioControl) == true
            if bleActive && (bleFeedback || vioWaypoint) {
                feedbackTrackingHealthy = mode == .phone && TerraFleetManualDrive.vioReady(autonomy)
                if vioWaypoint && !vioCompatible { revokeHardwareMotion(reason: "L3 requires differential motor routes") }
            }
            if bleActive && fullyManual && tick % 5 == 0 {
                // L0 has no sensor-driven motor correction. The arbiter still
                // gates the operator command by authority, freshness and safety.
                let command = DriveJoystickCommand(forward: (output.leftEffort + output.rightEffort) / 2, yaw: (output.rightEffort - output.leftEffort) / 2)
                routeHardware(left: output.leftEffort, right: output.rightEffort, now: ProcessInfo.processInfo.systemUptime, manualTarget: command, rustFrame: output.actuatorFrame)
            }
            if bleActive && (bleFeedback || vioWaypoint) && !fullyManual {
                if !feedbackTrackingHealthy || output.safety != .active {
                    pauseHardwareMotion(reason: "Autonomy: \(TerraStopDiagnostic.autonomyReason(autonomy)); feedback controller safety=\(String(describing: output.safety)); time=\(now) target=(\(output.targetForward),\(output.targetYawRate)) estimated=(\(output.estimatedForward),\(output.estimatedYawRate)) effort=(\(output.leftEffort),\(output.rightEffort))")
                }
                if tick % 5 == 0 { routeHardware(left: output.leftEffort, right: output.rightEffort, now: ProcessInfo.processInfo.systemUptime, closedLoop: true, rustFrame: output.actuatorFrame) }
            }
            if bleActive && !bleFeedback && !vioWaypoint && !fullyManual && tick % 5 == 0 && controller.dashboardStatus() == "connected" {
                let command = DriveJoystickCommand(forward: (output.leftEffort + output.rightEffort) / 2, yaw: (output.rightEffort - output.leftEffort) / 2)
                routeHardware(left: output.leftEffort, right: output.rightEffort, now: ProcessInfo.processInfo.systemUptime, manualTarget: command, rustFrame: output.actuatorFrame)
            }
            if tick % 10 == 0 { publish(output);updateAuthority() }
            if tick % 10 == 0, mode == .phone { taskModel?.service(controller); taskAgent.service(controller) }
            if mode == .phone { publishLocalization(controller) }
        } catch { fail(error, context: "Control update in \(mode) at \(now)") }
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
    func fail(_ error: Error, context: String = "Controller") {
        observation.record(component: "phone", kind: "lifecycle.failure", attributes: ["context": context, "error": error.localizedDescription])
        retainStopReason("\(context): \(String(describing: error))")
        TerraLog.control.error("Controller failure: \(context, privacy: .public): \(error.localizedDescription, privacy: .public)")
        let detail = "\(context): \(error.localizedDescription)"
        #if DEBUG
        let destination = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0].appendingPathComponent("ControllerFailure.json")
        if let data = try? JSONSerialization.data(withJSONObject: ["time": ISO8601DateFormatter().string(from: Date()), "context": context, "error": error.localizedDescription, "mode": String(describing: mode), "systemUptime": ProcessInfo.processInfo.systemUptime, "mediaTime": CACurrentMediaTime()]) {
            try? data.write(to: destination, options: .atomic)
        }
        #endif
        bluetooth.stopOutput(); bluetooth.disconnect(); bleActive = false
        mode = .stopped; timer?.cancel(); timer = nil; try? controller?.reset()
        phonePose = nil
        sensorEpochLock.lock(); sensorEpoch += 1; sensorEpochLock.unlock()
        DispatchQueue.main.async {
            self.hardwareActive = false; self.hardwareFeedback = false; self.phoneTrackingActive = false
            self.waypointActive = false; self.waypointDistance = 0; self.waypointStatus = "No goal"
            self.motion.stopDeviceMotionUpdates(); self.session.pause()
            self.status = detail; self.leftEffort = 0; self.rightEffort = 0
            self.mapStatus = "Map paused"
            self.dashboardConnecting = false; self.dashboardConnected = false; self.dashboardStatus = "Disconnected · controller stopped"
        }
    }
}

extension PhoneController {
    func retainStopReason(_ reason: String) {
        let time = Date().timeIntervalSince1970
        DispatchQueue.main.async { self.lastStopDiagnostic.record(reason, at: time) }
    }
    func clearStopReason() { lastStopDiagnostic.clear() }
    func sensorEpochMatches(_ epoch: Int) -> Bool {
        sensorEpochLock.lock(); defer { sensorEpochLock.unlock() }; return sensorEpoch == epoch
    }
    func pauseHardwareMotion(reason: String) {
        target = (0, 0); resetServoTargets()
        observation.recordChanged(component: "control", kind: "motion.stop", attributes: ["reason": reason])
        retainStopReason(reason)
        bluetooth.stopOutput()
    }
    func revokeHardwareMotion(reason: String = #function) {
        pauseHardwareMotion(reason: reason)
    }
    /// Runs on `controlQueue`. Every driving controller starts with the selected rover profile.
    func makeController() throws -> MobileController {
        let controller = try MobileController(settings: defaultControlSettings())
        if let json = RoverProfiles.json(named: RoverProfiles.selected) { try controller.setRoverProfile(profileJson: json) }
        applyDock(to: controller)
        return controller
    }
}
