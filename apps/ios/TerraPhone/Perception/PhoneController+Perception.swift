import OSLog
import ARKit
import AVFoundation
import Combine
import CoreMotion
import CoreLocation
import Foundation
import QuartzCore
import simd

// Object search detector and the detection/classification debug views.
extension PhoneController {
    func updateObjectSearch(frame: ARFrame, tracked: Bool, epoch: Int) {
        guard let controller else { return }
        let version = ObjectSearchDetector.version
        let ready = tracked && frame.sceneDepth?.confidenceMap != nil
        guard ready else {
            try? controller.registerSearchDetector(classes: [], version: version, timestamp: frame.timestamp)
            setDetectorHealth(tracked ? "LiDAR scene depth unavailable" : "tracking unavailable")
            return
        }
        if let reason = objectDetector.unavailableReason {
            try? controller.registerSearchDetector(classes: [], version: version, timestamp: frame.timestamp)
            setDetectorHealth(reason)
            return
        }
        objectDetector.submit(frame: frame, worldFromAR: worldFromAR) { [weak self] result in
            let delivered = CACurrentMediaTime()
            guard let self else { return }
            self.controlQueue.async {
                guard self.sensorEpochMatches(epoch), self.mode == .phone, self.localizationTracked,
                      let controller = self.controller else { return }
                let now = CACurrentMediaTime()
                defer {
                    if case .success(let frame) = result {
                        SensorTimingProbe.shared.record(.detector, sensor: frame.timestamp, os: delivered, queue: now, done: CACurrentMediaTime(), a: Double(frame.detections.count))
                    }
                }
                do {
                    switch result {
                    case .failure(let error):
                        try controller.registerSearchDetector(classes: [], version: version, timestamp: now)
                        self.setDetectorHealth("failed: \(error)")
                    case .success(let frame):
                        guard now >= frame.timestamp, now-frame.timestamp < 0.5 else {
                            try controller.registerSearchDetector(classes: [], version: version, timestamp: now)
                            self.setDetectorHealth("inference frame stale")
                            return
                        }
                        try controller.registerSearchDetector(classes: ObjectSearchDetector.classes, version: version, timestamp: frame.timestamp)
                        self.setDetectorHealth("bucket detector ready")
                        if frame.rejected > 0 { TerraLog.tracking.debug("Object boxes rejected: \(frame.rejected); unknown class or measured depth unavailable") }
                        // Every box goes to the object map, which the dashboard shows as markers.
                        for d in frame.detections {
                            do {
                                try controller.pushObjectDetection(d: MobileObjectDetection(className: d.className, color: d.color,
                                    confidence: d.confidence, frameId: frame.frameID, worldX: Double(d.world.x), worldY: Double(d.world.y),
                                    receivedAt: frame.timestamp), timestamp: now)
                            } catch { TerraLog.tracking.debug("Object detection rejected: \(String(describing: error), privacy: .public)") }
                        }
                        guard let search = controller.targetSearchSnapshot(timestamp: now), ["searching", "confirming"].contains(search.phase) else { return }
                        // One observation per camera frame; never let separate buckets become duplicate frame evidence.
                        if let best = frame.detections.filter({ $0.className == search.targetClass }).max(by: { $0.confidence < $1.confidence }) {
                            try controller.pushTargetObservation(o: MobileTargetObservation(runId: search.runId, searchId: search.searchId,
                                frameId: frame.frameID, targetClass: search.targetClass, confidence: best.confidence,
                                worldX: Double(best.world.x), worldY: Double(best.world.y), receivedAt: frame.timestamp,
                                evidenceId: "\(search.targetClass)-\(frame.frameID)"), timestamp: now)
                        }
                    }
                } catch { TerraLog.tracking.notice("Object observation or capability rejected: \(String(describing: error), privacy: .public)") }
            }
        }
    }
    private func setDetectorHealth(_ value: String) {
        guard detectorHealth != value else { return }
        detectorHealth = value
        TerraLog.tracking.notice("L4 detector: \(value, privacy: .public)")
        DispatchQueue.main.async { self.objectDetectorStatus = value }
    }
    /// Ask the mapper for labelled frames (or stop). Works in phone mode with no
    /// rover connected, so classification can be checked off the robot.
    /// Object detection debug page: keep the model's input image and every raw box.
    func setObjectDetectionDebug(_ enabled: Bool) { objectDetector.setDebug(enabled) }
    /// Newest detector snapshot since the last call, on the main queue (nil when none is new).
    func takeObjectDebugFrame(_ deliver: @escaping (ObjectDebugFrame?) -> Void) {
        let frame = objectDetector.takeDebugFrame()
        DispatchQueue.main.async { deliver(frame) }
    }
    /// The run's object map as JSON (the `objects` topic body), on the main queue.
    func objectMapJSON(_ deliver: @escaping (String) -> Void) {
        controlQueue.async {
            let json = self.controller?.objectMapJson() ?? ""
            DispatchQueue.main.async { deliver(json) }
        }
    }
    func clearObjectMap() { controlQueue.async { try? self.controller?.clearObjectMap() } }
    func setClassificationDebug(_ enabled: Bool) {
        controlQueue.async {
            self.classificationDebugEnabled = enabled
            self.occupancyMap?.setClassificationDebug(enabled: enabled)
        }
    }
    /// Newest labelled depth frame since the last call, labelled in Rust with the
    /// grid's own rules, delivered on the main queue (nil when none is new).
    func takeClassifiedFrame(stride: UInt32, _ deliver: @escaping (ClassifiedFrame?) -> Void) {
        controlQueue.async {
            let map = self.occupancyMap
            DispatchQueue.global(qos: .userInitiated).async {
                let frame = map?.takeClassifiedFrame(stride: stride)
                DispatchQueue.main.async { deliver(frame) }
            }
        }
    }
}
