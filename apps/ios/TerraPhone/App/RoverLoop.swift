import Combine
import Foundation

/// The whole app in one loop, 100 times a second:
///
///     PhoneSensors ─► SensorFrame ─► TerraAv.tick ─► WheelCommand ─► BluetoothLink
///                         ▲               ▲
///                      e-stop        Goal (joystick)
final class RoverLoop: ObservableObject {
    let sensors = PhoneSensors()
    let motors = BluetoothLink()
    private let av: TerraAv
    private let queue = DispatchQueue(label: "terra.loop")
    private var timer: DispatchSourceTimer?
    private var goal = Goal.none
    private var estop = false

    @Published private(set) var running = false
    @Published private(set) var estopped = false
    @Published private(set) var command = WheelCommand.stop

    /// Joystick full scale.
    let maxSpeed = 0.5      // m/s
    let maxTurn = 1.5       // rad/s

    init() {
        let url = Bundle.main.url(forResource: "terra-av", withExtension: "toml")
        av = TerraAv(configToml: url.flatMap { try? String(contentsOf: $0, encoding: .utf8) } ?? "")
    }

    func start() {
        guard timer == nil else { return }
        sensors.start()
        let t = DispatchSource.makeTimerSource(queue: queue)
        t.schedule(deadline: .now(), repeating: .milliseconds(10))
        t.setEventHandler { [weak self] in self?.tick() }
        t.resume(); timer = t
        running = true
    }

    func stop() {
        timer?.cancel(); timer = nil
        sensors.stop()
        motors.send(.stop)
        running = false
        command = .stop
    }

    /// Stick in [-1, 1] → Goal.velocity.
    func setStick(forward: Double, turn: Double) {
        queue.async {
            self.goal = (forward == 0 && turn == 0) ? .none : .velocity(v: forward * self.maxSpeed, w: turn * self.maxTurn)
        }
    }

    func setEstop(_ on: Bool) {
        queue.async { self.estop = on }
        estopped = on
        if on { motors.send(.stop) }
    }

    private func tick() {
        let frame = sensors.frame(estop: estop)
        let out = av.tick(frame: frame, goal: goal)
        motors.send(out)
        DispatchQueue.main.async { if self.command != out { self.command = out } }
    }
}
