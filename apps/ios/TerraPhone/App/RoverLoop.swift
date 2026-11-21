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
final class RoverLoop: ObservableObject {
    let sensors = PhoneSensors()
    let motors = BluetoothLink()
    /// Replaced only on `plannerQueue` (see `setRouterPassword`).
    private var av: TerraAv
    private var controller = VelocityController()

    private let plannerQueue = DispatchQueue(label: "terra.planner")
    private let velocityQueue = DispatchQueue(label: "terra.velocity", qos: .userInteractive)
    private var plannerTimer: DispatchSourceTimer?
    private var velocityTimer: DispatchSourceTimer?

    // Planner-queue state.
    private var goal = Goal.none
    private var estop = false
    // Shared between the loops.
    private let lock = NSLock()
    private var latest: (command: DriveCommand, at: TimeInterval)?

    @Published private(set) var running = false
    @Published private(set) var estopped = false
    @Published private(set) var worldMapStatus: String?
    @Published private(set) var command = WheelCommand.stop

    /// Joystick full scale.
    let maxSpeed = 0.5 // m/s
    let maxTurn = 1.5 // rad/s

    init() {
        av = Self.makeAv()
        sensors.onMeshChunk = { [weak self] id, vertices, indices, classes in
            guard let self else { return }
            let t = ProcessInfo.processInfo.systemUptime
            self.plannerQueue.async {
                self.av.publishMeshChunk(t: t, id: id, vertices: vertices, indices: indices, classes: classes)
            }
        }
    }

    /// terra-av from the bundled terra-av.toml, with the vehicle from the
    /// app's setup (if any), plus the Keychain router password.
    private static func makeAv() -> TerraAv {
        let url = Bundle.main.url(forResource: "terra-av", withExtension: "toml")
        var toml = url.flatMap { try? String(contentsOf: $0, encoding: .utf8) } ?? ""
        if let setup = VehicleSetup.load() { toml = setup.applying(to: toml) }
        return TerraAv(configToml: toml, routerPassword: RouterPassword.load())
    }

    /// Save the vehicle setup and rebuild terra-av with it.
    func setVehicle(_ setup: VehicleSetup) {
        setup.save()
        plannerQueue.async { self.av = Self.makeAv() }
    }

    /// Store the router password and reconnect with it (empty clears it).
    func setRouterPassword(_ password: String) {
        RouterPassword.save(password)
        plannerQueue.async { self.av = Self.makeAv() }
    }

    /// Capture the current ARKit `ARWorldMap` and publish it to the base station.
    func sendWorldMap() {
        DispatchQueue.main.async { self.worldMapStatus = "Capturing world map..." }
        sensors.captureWorldMap { [weak self] result in
            guard let self else { return }
            switch result {
            case .success(let snapshot):
                let t = ProcessInfo.processInfo.systemUptime
                self.plannerQueue.async {
                    self.av.publishWorldMap(t: t, points: snapshot.points, data: snapshot.data)
                }
                let kb = Double(snapshot.data.count) / 1024.0
                let size = kb >= 1024.0 ? String(format: "%.2f MB", kb / 1024.0) : String(format: "%.0f KB", kb)
                print("[WorldMap] sent \(snapshot.points.count) pts, \(size) (\(snapshot.data.count) bytes)")
                DispatchQueue.main.async { self.worldMapStatus = "Map sent (\(snapshot.points.count) pts · \(size))" }
            case .failure(let error):
                print("[WorldMap] capture failed: \(error.localizedDescription)")
                DispatchQueue.main.async { self.worldMapStatus = "Map failed: \(error.localizedDescription)" }
            }
        }
    }

    func start() {
        guard plannerTimer == nil else { return }
        sensors.start()
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

    private func plannerTick() {
        let out = av.tick(frame: sensors.frame(estop: estop), goal: goal)

        // print(sensors.frame(estop: false).depth)

        lock.lock(); latest = (out, ProcessInfo.processInfo.systemUptime); lock.unlock()
    }

    /// Joystick → wheel efforts (arcade drive).
    /// forward: +1 = full ahead, turn: +1 = full left. Both in [-1, 1].
    func joystickToMotors(forward: Double, turn: Double, maxEffort: Double = 0.5) -> WheelCommand {
        var left = forward - turn
        var right = forward + turn
        // Keep the ratio between the wheels if either side would exceed 1.
        let biggest = max(1, abs(left), abs(right))
        left /= biggest
        right /= biggest
        return WheelCommand(left: left * maxEffort, right: right * maxEffort)
    }

    private func velocityTick() {
        lock.lock(); let latest = self.latest; lock.unlock()

        // Test drive from the phone's joystick goes straight to the motors
        // (terra-av's tick doesn't drive yet). Otherwise send only the
        // controller's output: sending .stop first made every tick alternate
        // stop / drive frames.
        if case let .velocity(v, w) = goal {
            let out = joystickToMotors(forward: v, turn: w)
            motors.send(out)
            DispatchQueue.main.async { if self.command != out { self.command = out } }
            return
        }

        let out: WheelCommand
        if let latest {
            out = controller.command(target: latest.command.target, measured: latest.command.measured, targetAge: ProcessInfo.processInfo.systemUptime - latest.at)
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
