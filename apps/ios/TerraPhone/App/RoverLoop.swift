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
    private var av: TerraAv
    /// Used only on `velocityQueue`.
    private var controller = VelocityController()

    private let plannerQueue = DispatchQueue(label: "terra.planner")
    private let velocityQueue = DispatchQueue(label: "terra.velocity", qos: .userInteractive)
    private var plannerTimer: DispatchSourceTimer?
    private var velocityTimer: DispatchSourceTimer?
    private var setupSubscription: AnyCancellable?

    // Planner-queue state.
    private var activeSetup: VehicleSetup?
    private var activeBaseStation: BaseStationConfig
    private var goal = Goal.none
    private var estop = false
    private var ticksSinceStatus = 0
    // Shared between the loops.
    private let lock = NSLock()
    private var latest: (command: DriveCommand, at: TimeInterval)?

    @Published private(set) var baseStation: BaseStationConfig
    @Published private(set) var running = false
    @Published private(set) var estopped = false
    @Published private(set) var worldMapStatus: String?
    @Published private(set) var command = WheelCommand.stop
    /// What terra-av's last tick saw, refreshed twice a second while running.
    @Published private(set) var status: RoverStatus?

    /// Joystick full scale.
    let maxSpeed = 0.5 // m/s
    let maxTurn = 1.5 // rad/s

    init() {
        let loadedBaseStation = BaseStationConfig.load()
        baseStation = loadedBaseStation
        activeBaseStation = loadedBaseStation
        av = Self.makeAv(setup: nil, baseStation: loadedBaseStation)
        setupSubscription = motors.$vehicleSetup
            .dropFirst()
            .removeDuplicates()
            .sink { [weak self] setup in
                self?.applyVehicle(setup)
            }
        sensors.onMeshChunk = { [weak self] id, vertices, indices, classes in
            guard let self else { return }
            let t = ProcessInfo.processInfo.systemUptime
            self.plannerQueue.async {
                self.av.publishMeshChunk(t: t, id: id, vertices: vertices, indices: indices, classes: classes)
            }
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
        }
        toml = baseStation.applying(to: toml, roverIdOverride: setup?.roverId)
        return TerraAv(configToml: toml, routerPassword: RouterPassword.load(), stateDir: nil)
    }

    private func applyVehicle(_ setup: VehicleSetup?) {
        plannerQueue.async {
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
            self.activeBaseStation = config
            self.av = Self.makeAv(setup: self.activeSetup, baseStation: config)
        }
    }

    /// Store the router password and reconnect with it (empty clears it).
    func setRouterPassword(_ password: String) {
        RouterPassword.save(password)
        plannerQueue.async { self.av = Self.makeAv(setup: self.activeSetup, baseStation: self.activeBaseStation) }
    }

    /// Capture the current ARKit `ARWorldMap` and publish it to the base station.
    func sendWorldMap() {
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
    }

    /// Planner loop: one terra-av tick, and its result becomes the latest
    /// drive command.
    private func plannerTick() {
        let out = av.tick(frame: sensors.frame(estop: estop), goal: goal)
        lock.lock(); latest = (out, ProcessInfo.processInfo.systemUptime); lock.unlock()

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
        lock.lock(); let latest = self.latest; lock.unlock()

        let out: WheelCommand
        if let latest {
            let age = ProcessInfo.processInfo.systemUptime - latest.at
            out = controller.command(target: latest.command.target, measured: latest.command.measured, targetAge: age)
        } else {
            out = .stop
        }

        motors.send(out)
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
