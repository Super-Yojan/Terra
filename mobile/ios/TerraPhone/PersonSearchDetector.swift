import ARKit
import Vision
import UIKit
import ImageIO

struct PersonSearchDetection { let confidence: Double; let world: SIMD3<Float> }
struct PersonSearchFrame {
    let timestamp: Double
    let frameID: UInt64
    let detections: [PersonSearchDetection]
    let rejected: Int
}

/// One retained AR frame and one inference at a time; never queue camera history.
final class PersonSearchDetector {
    private let queue = DispatchQueue(label: "terra.person-detector", qos: .userInitiated)
    private let lock = NSLock()
    private var gate = PersonDetectorFrameGate()
    private var orientation = PersonImageOrientation.right
    private var orientationObserver: NSObjectProtocol?
    init() {
        UIDevice.current.beginGeneratingDeviceOrientationNotifications()
        orientationObserver = NotificationCenter.default.addObserver(forName: UIDevice.orientationDidChangeNotification, object: nil, queue: .main) { [weak self] _ in self?.updateOrientation() }
        updateOrientation()
    }
    deinit { if let orientationObserver { NotificationCenter.default.removeObserver(orientationObserver) } }
    private func updateOrientation() {
        lock.lock(); defer { lock.unlock() }
        switch UIDevice.current.orientation {
        case .portrait: orientation = .right
        case .portraitUpsideDown: orientation = .left
        case .landscapeLeft: orientation = .up
        case .landscapeRight: orientation = .down
        default: break
        }
    }
    func submit(frame: ARFrame, worldFromAR: simd_float3x3,
                completion: @escaping (Result<PersonSearchFrame, Error>) -> Void) {
        lock.lock()
        guard let id = gate.begin(timestamp: frame.timestamp) else { lock.unlock(); return }
        let orientation = self.orientation
        lock.unlock()
        queue.async { [self] in
            defer { lock.lock(); gate.finish(); lock.unlock() }
            do {
                guard let depth = frame.sceneDepth, let confidenceMap = depth.confidenceMap else {
                    throw DetectorError.depthUnavailable
                }
                let imageOrientation: CGImagePropertyOrientation
                switch orientation { case .up: imageOrientation = .up; case .right: imageOrientation = .right; case .down: imageOrientation = .down; case .left: imageOrientation = .left }
                let request = VNDetectHumanRectanglesRequest()
                request.upperBodyOnly = true
                try VNImageRequestHandler(cvPixelBuffer: frame.capturedImage, orientation: imageOrientation).perform([request])
                var detections: [PersonSearchDetection] = [], rejected = 0
                CVPixelBufferLockBaseAddress(depth.depthMap, .readOnly)
                CVPixelBufferLockBaseAddress(confidenceMap, .readOnly)
                defer {
                    CVPixelBufferUnlockBaseAddress(depth.depthMap, .readOnly)
                    CVPixelBufferUnlockBaseAddress(confidenceMap, .readOnly)
                }
                let width = CVPixelBufferGetWidth(depth.depthMap), height = CVPixelBufferGetHeight(depth.depthMap)
                guard width == CVPixelBufferGetWidth(confidenceMap), height == CVPixelBufferGetHeight(confidenceMap),
                      let depths = CVPixelBufferGetBaseAddress(depth.depthMap), let confidences = CVPixelBufferGetBaseAddress(confidenceMap) else { throw DetectorError.depthUnavailable }
                let depthStride = CVPixelBufferGetBytesPerRow(depth.depthMap), confidenceStride = CVPixelBufferGetBytesPerRow(confidenceMap)
                for person in request.results ?? [] {
                    guard person.confidence >= 0.8 else { rejected += 1; continue }
                    let box = person.boundingBox
                    var samples: [Float] = []
                    // Use the central torso region to avoid the background at silhouette edges.
                    for y in 0..<5 { for x in 0..<5 {
                        let vx = Float(box.minX + box.width*(0.3+Double(x)*0.1))
                        let vy = Float(box.minY + box.height*(0.3+Double(y)*0.1))
                        let raw = PersonDepthGeometry.rawPoint(x: vx, y: vy, orientation: orientation)
                        let px = min(width-1, max(0, Int(raw.x*Float(width))))
                        let py = min(height-1, max(0, Int(raw.y*Float(height))))
                        if confidences.advanced(by: py*confidenceStride).assumingMemoryBound(to: UInt8.self)[px] == UInt8(ARConfidenceLevel.high.rawValue) {
                            samples.append(depths.advanced(by: py*depthStride).assumingMemoryBound(to: Float.self)[px])
                        }
                    } }
                    let raw = PersonDepthGeometry.rawPoint(x: Float(box.midX), y: Float(box.midY), orientation: orientation)
                    let pixel = raw * SIMD2(Float(frame.camera.imageResolution.width), Float(frame.camera.imageResolution.height))
                    guard let measured = PersonDepthGeometry.measuredDepth(samples),
                          let world = PersonDepthGeometry.worldPoint(pixel: pixel, depth: measured, intrinsics: frame.camera.intrinsics,
                                                                    cameraTransform: frame.camera.transform, worldFromAR: worldFromAR) else { rejected += 1; continue }
                    detections.append(PersonSearchDetection(confidence: Double(person.confidence), world: world))
                }
                completion(.success(PersonSearchFrame(timestamp: frame.timestamp, frameID: id, detections: detections, rejected: rejected)))
            } catch { completion(.failure(error)) }
        }
    }
    enum DetectorError: Error { case depthUnavailable }
}
