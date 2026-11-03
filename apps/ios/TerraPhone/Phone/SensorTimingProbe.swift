import ARKit
import CoreMotion
import Foundation
import QuartzCore
import UIKit

/// Per-sample hop timing for the iPhone sensor feasibility experiment.
///
/// Off by default. While recording, each hop of each sensor stream appends one row; a
/// utility queue writes them once a second to `Documents/SensorTiming/<condition>-<utc>.csv`.
/// All times are `CACurrentMediaTime()` seconds, the clock Core Motion and ARKit stamp
/// samples with, so ages need no clock sync. GPS and compass stamps are wall-clock `Date`s
/// and are converted on arrival.
///
/// Columns: `stream,seq,t_sensor,t_os,t_queue,t_done,a,b` (NaN when a hop does not apply).
///
/// | stream     | t_sensor               | t_os                  | t_queue                | t_done              | a                        | b              |
/// |------------|------------------------|-----------------------|------------------------|---------------------|--------------------------|----------------|
/// | imu        | CMDeviceMotion stamp   | motion handler entry  | control block start    | after pushImu       |                          |                |
/// | ar         | ARFrame stamp          | delegate entry        | processARFrame start   | after pushVio       | processARFrame end       | dropped frames |
/// | detector   | camera frame stamp     | YOLO result callback  | control block start    | after Rust pushes   | detections               |                |
/// | gps        | CLLocation stamp       | delegate entry        | control block start    |                     | horizontal accuracy (m)  | speed (m/s)    |
/// | heading    | CLHeading stamp        | delegate entry        | control block start    |                     | heading accuracy (deg)   |                |
/// | tick       | newest IMU pushed      |                       | tick start (`now`)     | after step()        | newest VIO pushed        | safety active  |
/// | block      | ARFrame that started it|                       | block start            | block end           | 1 = floor+barrier apply  | floats handed  |
/// | bare_imu   | CMDeviceMotion stamp   | bare handler entry    |                        |                     |                          |                |
/// | bare_ar    | ARFrame stamp          | bare delegate entry   |                        |                     |                          |                |
/// | meta       | flush time             |                       |                        |                     | thermal state 0–3        | low power 0/1  |
/// | battery    | start or change time   |                       |                        |                     | level 0–1 (−1 unknown)   | state 0–3      |
final class SensorTimingProbe: @unchecked Sendable {
    static let shared = SensorTimingProbe()
    enum Stream: String { case imu, ar, detector, gps, heading, tick, block, bare_imu, bare_ar, meta, battery }

    private struct Row {
        var stream: Stream, seq: UInt64
        var sensor, os, queue, done, a, b: Double
    }
    private let lock = NSLock()
    private var rows: [Row] = []
    private var sequence: [Stream: UInt64] = [:]
    private var counts: [Stream: Int] = [:]
    private var file: FileHandle?
    private var fileURL: URL?
    private var flushTimer: DispatchSourceTimer?
    private var lostRows = 0
    private let ioQueue = DispatchQueue(label: "terra.sensor-timing", qos: .utility)
    /// Bounds memory if the disk stalls: about 20 s of every stream at full rate.
    private let maxBufferedRows = 20_000

    var isRecording: Bool { lock.lock(); defer { lock.unlock() }; return file != nil }
    var currentFile: URL? { lock.lock(); defer { lock.unlock() }; return fileURL }
    func rowCounts() -> [String: Int] {
        lock.lock(); defer { lock.unlock() }
        var out = Dictionary(uniqueKeysWithValues: counts.map { ($0.key.rawValue, $0.value) })
        if lostRows > 0 { out["lost"] = lostRows }
        return out
    }

    /// Hot path: one lock and one append; formatting happens on the io queue.
    @inline(__always)
    func record(_ stream: Stream, sensor: Double, os: Double = .nan, queue: Double = .nan,
                done: Double = .nan, a: Double = .nan, b: Double = .nan) {
        lock.lock(); defer { lock.unlock() }
        guard file != nil else { return }
        // Sequence advances even for dropped rows, so the analysis can count the holes.
        let seq = sequence[stream, default: 0]
        sequence[stream] = seq + 1
        guard rows.count < maxBufferedRows else { lostRows += 1; return }
        counts[stream, default: 0] += 1
        rows.append(Row(stream: stream, seq: seq, sensor: sensor, os: os, queue: queue, done: done, a: a, b: b))
    }

    /// Converts a wall-clock sensor stamp (Core Location) to the media clock.
    static func mediaTime(of date: Date) -> Double {
        CACurrentMediaTime() - Date().timeIntervalSince(date)
    }

    func start(condition: String) throws {
        stop()
        let directory = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0].appendingPathComponent("SensorTiming")
        try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
        let stamp = ISO8601DateFormatter().string(from: Date()).replacingOccurrences(of: ":", with: "")
        let safe = condition.filter { $0.isLetter || $0.isNumber || $0 == "-" || $0 == "_" }
        let url = directory.appendingPathComponent("\(safe.isEmpty ? "run" : safe)-\(stamp).csv")
        let device = ProcessInfo.processInfo
        var header = "# terra sensor timing v1\n# condition=\(condition)\n# os=\(device.operatingSystemVersionString)\n"
        #if DEBUG
        header += "# build=debug\n"
        #else
        header += "# build=release\n"
        #endif
        header += "# model=\(Self.modelIdentifier())\n# media_time_at_start=\(CACurrentMediaTime())\n# wall_time_at_start=\(Date().timeIntervalSince1970)\n"
        header += "stream,seq,t_sensor,t_os,t_queue,t_done,a,b\n"
        guard FileManager.default.createFile(atPath: url.path, contents: Data(header.utf8)) else {
            throw CocoaError(.fileWriteUnknown)
        }
        let handle = try FileHandle(forWritingTo: url)
        try handle.seekToEnd()
        lock.lock()
        rows.removeAll(keepingCapacity: true); sequence.removeAll(); counts.removeAll(); lostRows = 0
        file = handle; fileURL = url
        lock.unlock()
        let timer = DispatchSource.makeTimerSource(queue: ioQueue)
        timer.schedule(deadline: .now() + 1, repeating: 1, leeway: .milliseconds(100))
        timer.setEventHandler { [weak self] in self?.flush() }
        timer.resume()
        flushTimer = timer
        // Battery is a UIDevice (main-thread) property: one row now, then one per change.
        DispatchQueue.main.async { [weak self] in
            guard let self else { return }
            UIDevice.current.isBatteryMonitoringEnabled = true
            self.recordBattery()
            guard self.batteryObservers.isEmpty else { return }
            for name in [UIDevice.batteryLevelDidChangeNotification, UIDevice.batteryStateDidChangeNotification] {
                self.batteryObservers.append(NotificationCenter.default.addObserver(forName: name, object: nil, queue: .main) { [weak self] _ in self?.recordBattery() })
            }
        }
    }

    /// Main thread only.
    private var batteryObservers: [NSObjectProtocol] = []
    private func recordBattery() {
        let device = UIDevice.current
        record(.battery, sensor: CACurrentMediaTime(), a: Double(device.batteryLevel), b: Double(device.batteryState.rawValue))
    }

    func stop() {
        flushTimer?.cancel(); flushTimer = nil
        ioQueue.sync { flush() }
        lock.lock()
        let handle = file
        file = nil
        lock.unlock()
        try? handle?.close()
    }

    private func flush() {
        let thermal = Double(ProcessInfo.processInfo.thermalState.rawValue)
        let lowPower: Double = ProcessInfo.processInfo.isLowPowerModeEnabled ? 1 : 0
        record(.meta, sensor: CACurrentMediaTime(), a: thermal, b: lowPower)
        lock.lock()
        let batch = rows
        rows.removeAll(keepingCapacity: true)
        let handle = file
        lock.unlock()
        guard let handle, !batch.isEmpty else { return }
        var text = ""
        text.reserveCapacity(batch.count * 96)
        for r in batch {
            text += "\(r.stream.rawValue),\(r.seq),\(Self.f(r.sensor)),\(Self.f(r.os)),\(Self.f(r.queue)),\(Self.f(r.done)),\(Self.f(r.a)),\(Self.f(r.b))\n"
        }
        try? handle.write(contentsOf: Data(text.utf8))
    }

    private static func f(_ v: Double) -> String { v.isFinite ? String(format: "%.6f", v) : "" }

    private static func modelIdentifier() -> String {
        var info = utsname(); uname(&info)
        return withUnsafeBytes(of: &info.machine) { String(decoding: $0.prefix { $0 != 0 }, as: UTF8.self) }
    }
}

/// The "bare" side of the experiment: Core Motion (and optionally ARKit with LiDAR depth
/// and mesh) with handlers that only stamp arrival time. No Terra code runs in them.
/// It can run beside the full pipeline (IMU only; ARKit allows one session per camera)
/// or alone with the AR load switched on while Terra is stopped.
final class BareSensorLogger: NSObject, ARSessionDelegate, @unchecked Sendable {
    private let motion = CMMotionManager()
    private let queue: OperationQueue = {
        let q = OperationQueue()
        q.name = "terra.bare-motion"
        q.maxConcurrentOperationCount = 1
        q.qualityOfService = .userInteractive
        return q
    }()
    private var session: ARSession?
    private let arQueue = DispatchQueue(label: "terra.bare-ar", qos: .userInteractive)
    private(set) var running = false

    func start(arLoad: Bool) {
        stop()
        motion.deviceMotionUpdateInterval = 0.01
        motion.startDeviceMotionUpdates(using: .xArbitraryZVertical, to: queue) { data, _ in
            let arrived = CACurrentMediaTime()
            guard let data else { return }
            SensorTimingProbe.shared.record(.bare_imu, sensor: data.timestamp, os: arrived)
        }
        if arLoad {
            let configuration = ARWorldTrackingConfiguration()
            configuration.worldAlignment = .gravity
            configuration.planeDetection = [.horizontal]
            if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) { configuration.frameSemantics.insert(.sceneDepth) }
            if ARWorldTrackingConfiguration.supportsSceneReconstruction(.meshWithClassification) { configuration.sceneReconstruction = .meshWithClassification }
            let session = ARSession()
            session.delegate = self
            session.delegateQueue = arQueue
            session.run(configuration, options: [.resetTracking, .removeExistingAnchors])
            self.session = session
        }
        running = true
    }

    func stop() {
        motion.stopDeviceMotionUpdates()
        session?.pause(); session = nil
        running = false
    }

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        SensorTimingProbe.shared.record(.bare_ar, sensor: frame.timestamp, os: CACurrentMediaTime())
    }
}
