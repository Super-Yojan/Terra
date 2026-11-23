import Combine
import Foundation

/// Two independent loops that share only the latest drive command:
///
///     Planner loop, 20 Hz:   PhoneSensors ─► SensorFrame ─► TerraAv.tick ─► DriveCommand
///                                                ▲ e-stop        ▲ Goal (joystick)
///     Velocity loop, 50 Hz:  latest DriveCommand ─► VelocityController ─► BluetoothLink
///
/// A slow terra-av tick only delays the target; the velocity loop keeps its
/// fixed rate and stops on its own if the target goes stale.
///
/// Everything that moves the rover goes through both loops: the phone's
/// joystick, the base station's stick and the base station's tasks all
/// become a `DriveCommand` in `TerraAv.tick`, where the e-stops (the
/// phone's and the base station's) have the last word.
final class RoverLoop: ObservableObject {
    let sensors = PhoneSensors()
    let motors = BluetoothLink()
    /// Replaced only on `plannerQueue` (see `setRouterPassword`).
    var av: TerraAv
    /// Used only on `velocityQueue`.
    private var controller = VelocityController()

    let plannerQueue = DispatchQueue(label: "terra.planner")
    private let velocityQueue = DispatchQueue(label: "terra.velocity", qos: .userInteractive)
    private var plannerTimer: DispatchSourceTimer?
    private var velocityTimer: DispatchSourceTimer?
    private var setupSubscription: AnyCancellable?

    // Planner-queue state.
    private var activeSetup: VehicleSetup?
    private var activeBaseStation: BaseStationConfig
    var activeFpvIce: FpvIceSettings
    var fpvCall: FpvVideoCall?
    let fpvLock = NSLock()
    var fpvActiveCall: FpvVideoCall?
    private var goal = Goal.none
    private var estop = false
    private var ticksSinceStatus = 0
    /// Auto-request the base station's saved world map once per session when
    /// the base link first connects, so joining rovers align automatically.
    private var didAutoRequestWorldMap = false
    // Shared between the loops.
    private let lock = NSLock()
    private var latest: (command: DriveCommand, at: TimeInterval)?
    private var gateConfig: (id: Int, open: Double, closed: Double)?

    @Published private(set) var baseStation: BaseStationConfig
    @Published var fpvIce: FpvIceSettings
    @Published var fpvStatus = "Idle"
    @Published var fpvLive = false
    @Published var fpvActive = false
    @Published private(set) var running = false
    @Published private(set) var estopped = false
    @Published private(set) var hasDumpGate = false
    @Published private(set) var gateOpen = false
    @Published private(set) var worldMapStatus: String?
    @Published private(set) var relocalizing = false
    @Published private(set) var command = WheelCommand.stop
    /// What terra-av's last tick saw, refreshed twice a second while running.
    @Published private(set) var status: RoverStatus?

    /// Joystick full scale.
    let maxSpeed = 0.5 // m/s
    let maxTurn = 1.5 // rad/s

    init() {
        let loadedBaseStation = BaseStationConfig.load()
        let loadedFpvIce = FpvIceStore.load()
        baseStation = loadedBaseStation
        activeBaseStation = loadedBaseStation
        fpvIce = loadedFpvIce
        activeFpvIce = loadedFpvIce
        av = Self.makeAv(setup: nil, baseStation: loadedBaseStation)
        setupSubscription = motors.$vehicleSetup
            .dropFirst()
            .removeDuplicates()
            .sink { [weak self] setup in
                self?.applyVehicle(setup)
            }
        sensors.onRelocalized = { [weak self] in
            print("[WorldMap] ARKit relocalized to shared world map")
            DispatchQueue.main.async {
                self?.relocalizing = false
                self?.worldMapStatus = "Aligned to shared map"
            }
        }
        sensors.onMeshChunk = { [weak self] id, vertices, indices, classes, colors in
            guard let self else { return }
            let t = ProcessInfo.processInfo.systemUptime
            self.plannerQueue.async {
                self.av.publishMeshChunk(t: t, id: id, vertices: vertices, indices: indices, classes: classes, colors: colors)
            }
        }
        sensors.onCameraFrame = { [weak self] pixelBuffer, timestamp in
            self?.sendFpvFrame(pixelBuffer: pixelBuffer, timestampSeconds: timestamp)
        }
    }

    /// terra-av from the bundled terra-av.toml, overridden with the connected
    /// rover's setup from the Raspberry Pi (if loaded), the UI base-station
    /// settings, and the Keychain router password. No rover-specific state is
    /// persisted on the phone so the same iPhone can move freely between
    /// different rovers.
    private static func makeAv(setup: VehicleSetup?, baseStation: BaseStationConfig) -> TerraAv {
        let url = Bundle.main.url(forResource: "terra-av", withExtension: "toml")
        var toml = url.flatMap { try? String(contentsOf: $0, encoding: .utf8) } ?? ""
        if let setup {
            toml = setup.applying(to: toml)
            print("[RoverLoop] makeAv with Pi setup: \(setup.bodyLength)x\(setup.bodyWidth)m track=\(setup.trackWidth)m maxSpeed=\(setup.maxWheelSpeed)m/s")
        } else {
            print("[RoverLoop] makeAv with bundled terra-av.toml defaults (no Pi setup loaded yet)")
        }
        toml = baseStation.applying(to: toml, roverIdOverride: setup?.roverId)
        return TerraAv(configToml: toml, routerPassword: RouterPassword.load(), stateDir: nil)
    }

    private func applyVehicle(_ setup: VehicleSetup?) {
        let newGate: (id: Int, open: Double, closed: Double)? = setup.flatMap { s in
            s.dumpGateActuatorId.map { ($0, s.dumpGateOpen, s.dumpGateClosed) }
        }
        lock.lock(); gateConfig = newGate; lock.unlock()
        DispatchQueue.main.async { self.hasDumpGate = newGate != nil }

        plannerQueue.async {
            self.stopFpvLocked(publish: true)
            self.activeSetup = setup
            self.av = Self.makeAv(setup: setup, baseStation: self.activeBaseStation)
        }
        let trackWidth = setup?.trackWidth ?? VehicleSetup().trackWidth
        velocityQueue.async { self.controller.trackWidth = trackWidth }
    }

    /// Rebuild terra-av and the velocity loop with `setup` (persisted on the
    /// Raspberry Pi via `BluetoothLink.applyLayout`, never on the phone).
    func setVehicle(_ setup: VehicleSetup) {
        applyVehicle(setup)
    }

    /// Save base station connection settings and reconnect terra-av immediately.
    func setBaseStation(_ config: BaseStationConfig) {
        config.save()
        DispatchQueue.main.async { self.baseStation = config }
        plannerQueue.async {
            self.stopFpvLocked(publish: true)
            self.activeBaseStation = config
            self.av = Self.makeAv(setup: self.activeSetup, baseStation: config)
        }
    }

    /// Store the router password and reconnect with it (empty clears it).
    func setRouterPassword(_ password: String) {
        RouterPassword.save(password)
        plannerQueue.async {
            self.stopFpvLocked(publish: true)
            self.av = Self.makeAv(setup: self.activeSetup, baseStation: self.activeBaseStation)
        }
    }

    /// Ask the base station to send back its saved shared `ARWorldMap` so this
    /// rover relocalizes into the same world coordinates as the rest of the fleet.
    func requestWorldMap() {
        let t = ProcessInfo.processInfo.systemUptime
        DispatchQueue.main.async { self.worldMapStatus = "Requesting saved map..." }
        plannerQueue.async {
            self.didAutoRequestWorldMap = true
            self.av.requestWorldMap(t: t, isAuto: false)
        }
    }

    /// Cancel a stuck relocalization and restart ARKit with a clean local origin.
    func cancelRelocalization() {
        sensors.resetTracking()
        DispatchQueue.main.async {
            self.relocalizing = false
            self.worldMapStatus = "Tracking reset (local origin)"
        }
    }

    private func applySharedWorldMap(_ data: Data) {
        let kb = Double(data.count) / 1024.0
        let size = kb >= 1024.0 ? String(format: "%.2f MB", kb / 1024.0) : String(format: "%.0f KB", kb)
        switch sensors.loadWorldMap(data) {
        case let .success(pointsCount):
            print("[WorldMap] loaded shared map (\(pointsCount) pts, \(size)); relocalizing...")
            DispatchQueue.main.async {
                self.relocalizing = true
                self.worldMapStatus = "Relocalizing (\(pointsCount) pts · \(size)) — look around"
            }
        case let .failure(error):
            print("[WorldMap] load failed: \(error.localizedDescription)")
            DispatchQueue.main.async {
                self.relocalizing = false
                self.worldMapStatus = "Load map failed: \(error.localizedDescription)"
            }
        }
    }

    /// Capture the current ARKit `ARWorldMap` and publish it to the base station.
    func sendWorldMap() {
        plannerQueue.async { self.didAutoRequestWorldMap = true }
        DispatchQueue.main.async { self.worldMapStatus = "Capturing world map..." }
        sensors.captureWorldMap { [weak self] result in
            guard let self else { return }
            switch result {
            case let .success(snapshot):
                let t = ProcessInfo.processInfo.systemUptime
                self.plannerQueue.async {
                    self.av.publishWorldMap(t: t, points: snapshot.points, data: snapshot.data)
                }
                let kb = Double(snapshot.data.count) / 1024.0
                let size = kb >= 1024.0 ? String(format: "%.2f MB", kb / 1024.0) : String(format: "%.0f KB", kb)
                print("[WorldMap] sent \(snapshot.points.count) pts, \(size) (\(snapshot.data.count) bytes)")
                DispatchQueue.main.async { self.worldMapStatus = "Map sent (\(snapshot.points.count) pts · \(size))" }
            case let .failure(error):
                print("[WorldMap] capture failed: \(error.localizedDescription)")
                DispatchQueue.main.async { self.worldMapStatus = "Map failed: \(error.localizedDescription)" }
            }
        }
    }

    func start() {
        guard plannerTimer == nil else { return }
        sensors.start()
        // Show the status line from the first tick on.
        plannerQueue.async { self.ticksSinceStatus = 10 }
        plannerTimer = repeating(every: .milliseconds(50), on: plannerQueue) { [weak self] in self?.plannerTick() }
        velocityTimer = repeating(every: .milliseconds(20), on: velocityQueue) { [weak self] in self?.velocityTick() }
        running = true
    }

    func stop() {
        plannerTimer?.cancel(); plannerTimer = nil
        velocityTimer?.cancel(); velocityTimer = nil
        lock.lock(); latest = nil; lock.unlock()
        stopFpv()
        sensors.stop()
        motors.send(.stop)
        running = false
        command = .stop
        status = nil
    }

    /// Stick in [-1, 1] → Goal.velocity.
    func setStick(forward: Double, turn: Double) {
        plannerQueue.async {
            self.goal = (forward == 0 && turn == 0) ? .none : .velocity(v: forward * self.maxSpeed, w: turn * self.maxTurn)
        }
    }

    func setEstop(_ on: Bool) {
        plannerQueue.async { self.estop = on }
        if on {
            lock.lock(); latest = nil; lock.unlock(); motors.send(.stop)
        }
        estopped = on
        if on { gateOpen = false }
    }

    /// Command the dump gate open (`true`) or closed (`false`) from the phone UI.
    func setGateOpen(_ open: Bool) {
        plannerQueue.async {
            self.av.setGateOpen(open: open)
        }
        DispatchQueue.main.async { self.gateOpen = open }
    }

    /// Planner loop: one terra-av tick, and its result becomes the latest
    /// drive command.
    private func plannerTick() {
        let out = av.tick(frame: sensors.frame(estop: estop), goal: goal)
        lock.lock(); latest = (out, ProcessInfo.processInfo.systemUptime); lock.unlock()
        pollFpvLocked()
        if !didAutoRequestWorldMap && av.baseConnected() {
            didAutoRequestWorldMap = true
            av.requestWorldMap(t: ProcessInfo.processInfo.systemUptime, isAuto: true)
            print("[WorldMap] connected to base — auto-requesting saved world map")
        }
        if av.takeCaptureWorldMap() {
            print("[WorldMap] base station requested world map capture")
            sendWorldMap()
        }
        if let mapData = av.takeLoadWorldMap() {
            applySharedWorldMap(mapData)
        }
        DispatchQueue.main.async {
            if self.gateOpen != out.gateOpen {
                self.gateOpen = out.gateOpen
            }
        }

        // The status line, every 10th tick (2 Hz).
        ticksSinceStatus += 1
        if ticksSinceStatus >= 10 {
            ticksSinceStatus = 0
            let status = av.status()
            DispatchQueue.main.async {
                if self.status != status {
                    self.status = status
                }
            }
        }
    }

    /// Velocity loop: the latest drive command → wheel efforts → Bluetooth.
    /// With no command (not started, or the e-stop just cleared it) or a
    /// stale one, the wheels stop.
    private func velocityTick() {
        lock.lock()
        let latest = self.latest
        let gate = self.gateConfig
        lock.unlock()

        let out: WheelCommand
        var overrides: [Int: Double] = [:]
        if let latest {
            let age = ProcessInfo.processInfo.systemUptime - latest.at
            out = controller.command(target: latest.command.target, measured: latest.command.measured, targetAge: age)
            if let gate {
                overrides[gate.id] = latest.command.gateOpen ? gate.open : gate.closed
            }
        } else {
            out = .stop
        }

        motors.send(out, overrides: overrides)
        DispatchQueue.main.async {
            if self.command != out {
                self.command = out
            }
        }
    }

    private func repeating(every interval: DispatchTimeInterval, on queue: DispatchQueue, _ body: @escaping () -> Void) -> DispatchSourceTimer {
        let t = DispatchSource.makeTimerSource(flags: .strict, queue: queue)
        t.schedule(deadline: .now(), repeating: interval, leeway: .milliseconds(1))
        t.setEventHandler(handler: body)
        t.resume()
        return t
    }
}
