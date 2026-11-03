import ARKit
import CoreML
import Vision
import ImageIO
import CoreImage

/// One YOLO box placed in the world frame. `className` is `bucket` or `ball`; `color` is
/// the rest of the model label (`blue`, `orange`) or nil.
struct ObjectSearchDetection { let className: String; let color: String?; let confidence: Double; let world: SIMD3<Float> }
/// One raw model box for the debug page, before the object-map threshold.
struct ObjectDebugBox: Identifiable {
    let id: Int
    /// Vision-normalized rectangle of the upright image, bottom-left origin.
    let box: CGRect
    let className: String
    let color: String?
    let confidence: Float
    /// LiDAR depth of the box centre region, metres; nil when too few confident samples agree.
    let depth: Float?
}
/// What the model saw on one frame, kept only while the debug page is open.
struct ObjectDebugFrame {
    let timestamp: Double
    /// Upright camera image as the model received it, before the letterbox.
    let image: CGImage?
    let boxes: [ObjectDebugBox]
    let inferenceMilliseconds: Double
    let orientation: String
    let imageSize: CGSize
}
struct ObjectSearchFrame {
    let timestamp: Double
    let frameID: UInt64
    let detections: [ObjectSearchDetection]
    let rejected: Int
}

/// Runs the rover YOLO model (Models/RoverDetector.mlpackage, trained on ball, blue bucket and
/// orange bucket) on the AR camera image and places each box with LiDAR depth.
/// One inference at a time; never queue camera history. Only the camera image, depth and camera
/// pose are kept during inference, not the ARFrame, so ARKit's frame budget is untouched.
final class ObjectSearchDetector {
    static let version = "yolo26n-rover-depth-v1"
    /// Classes the model can report, as search and object-map class tokens.
    static let classes = ["bucket", "ball"]
    /// Boxes below this confidence are dropped before depth lookup.
    static let minimumConfidence: Float = 0.5
    private let queue = DispatchQueue(label: "terra.object-detector", qos: .userInitiated)
    private let lock = NSLock()
    private var gate = PersonDetectorFrameGate()
    private var model: VNCoreMLModel?
    private var labels: [Int: (String, String?)] = [:]
    private var inputSize: Float = 640
    private var loadError: String?
    private var debugEnabled = false
    private var latestDebug: ObjectDebugFrame?
    private let imageContext = CIContext()
    /// Boxes shown on the debug page start this low so near-misses are visible.
    static let debugMinimumConfidence: Float = 0.1

    init() { queue.async { [self] in load() } }

    /// Keep a debug snapshot of every inference (costs one image render per frame).
    func setDebug(_ enabled: Bool) {
        lock.lock(); debugEnabled = enabled; if !enabled { latestDebug = nil }; lock.unlock()
    }
    /// Newest debug snapshot since the last call, or nil when none is new.
    func takeDebugFrame() -> ObjectDebugFrame? {
        lock.lock(); defer { lock.unlock() }
        let frame = latestDebug; latestDebug = nil
        return frame
    }
    /// Nil when the model is loaded; otherwise why it is not.
    var unavailableReason: String? {
        lock.lock(); defer { lock.unlock() }
        return model == nil ? (loadError ?? "loading YOLO model") : nil
    }

    private func load() {
        do {
            guard let source = Bundle.main.url(forResource: "RoverDetector", withExtension: "mlpackage", subdirectory: "Models") else {
                throw DetectorError.modelMissing
            }
            // The Models folder is copied as-is, so the package is compiled once on the phone and cached.
            let cache = try FileManager.default.url(for: .applicationSupportDirectory, in: .userDomainMask, appropriateFor: nil, create: true)
                .appendingPathComponent("RoverDetector.mlmodelc")
            let stamp = cache.appendingPathComponent("terra-source-size")
            let size = (try? FileManager.default.attributesOfItem(atPath: source.appendingPathComponent("Data/com.apple.CoreML/weights/weight.bin").path)[.size] as? Int) ?? 0
            if (try? String(contentsOf: stamp, encoding: .utf8)) != "\(size)" {
                let compiled = try MLModel.compileModel(at: source)
                try? FileManager.default.removeItem(at: cache)
                try FileManager.default.moveItem(at: compiled, to: cache)
                try "\(size)".write(to: stamp, atomically: true, encoding: .utf8)
            }
            let configuration = MLModelConfiguration()
            configuration.computeUnits = .all
            let ml = try MLModel(contentsOf: cache, configuration: configuration)
            let labels = Self.labels(ml.modelDescription.metadata[.creatorDefinedKey] as? [String: String])
            let input = ml.modelDescription.inputDescriptionsByName.values.first?.imageConstraint
            let vision = try VNCoreMLModel(for: ml)
            lock.lock()
            self.labels = labels
            self.inputSize = Float(input?.pixelsWide ?? 640)
            self.model = vision
            lock.unlock()
            TerraLog.tracking.notice("YOLO object detector loaded: \(labels.count) classes")
        } catch {
            lock.lock(); loadError = "YOLO model failed: \(error)"; lock.unlock()
            TerraLog.tracking.error("YOLO object detector failed to load: \(String(describing: error), privacy: .public)")
        }
    }

    /// Ultralytics stores names as "{0: 'ball', 1: 'blue bucket', 2: 'orange bucket'}".
    /// The last word is the class token and any words before it are the colour.
    static func labels(_ metadata: [String: String]?) -> [Int: (String, String?)] {
        let names = metadata?["names"] ?? "{0: 'ball', 1: 'blue bucket', 2: 'orange bucket'}"
        var out: [Int: (String, String?)] = [:]
        let pattern = try! NSRegularExpression(pattern: "(\\d+):\\s*'([^']+)'")
        for m in pattern.matches(in: names, range: NSRange(names.startIndex..., in: names)) {
            guard let i = Range(m.range(at: 1), in: names).flatMap({ Int(names[$0]) }),
                  let r = Range(m.range(at: 2), in: names) else { continue }
            let words = names[r].lowercased().split(separator: " ").map(String.init)
            guard let cls = words.last else { continue }
            out[i] = (cls, words.count > 1 ? words.dropLast().joined(separator: "-") : nil)
        }
        return out
    }

    func submit(frame: ARFrame, worldFromAR: simd_float3x3,
                completion: @escaping (Result<ObjectSearchFrame, Error>) -> Void) {
        lock.lock()
        guard let model, let id = gate.begin(timestamp: frame.timestamp) else { lock.unlock(); return }
        let labels = self.labels, inputSize = self.inputSize, debug = debugEnabled
        lock.unlock()
        let capturedImage = frame.capturedImage, sceneDepth = frame.sceneDepth, camera = frame.camera, timestamp = frame.timestamp
        // Orientation comes from gravity, not UIDevice: Terra mounts the phone landscape and the
        // UI may be locked to portrait, so the device orientation does not describe the image.
        let up = camera.transform
        queue.async { [self] in
            defer { lock.lock(); gate.finish(); lock.unlock() }
            do {
                guard let depth = sceneDepth, let confidenceMap = depth.confidenceMap else {
                    throw DetectorError.depthUnavailable
                }
                guard let orientation = PersonDepthGeometry.orientation(upX: up.columns.0.y, upY: up.columns.1.y) else {
                    throw DetectorError.cameraVertical
                }
                let imageOrientation: CGImagePropertyOrientation
                switch orientation { case .up: imageOrientation = .up; case .right: imageOrientation = .right; case .down: imageOrientation = .down; case .left: imageOrientation = .left }
                let request = VNCoreMLRequest(model: model)
                request.imageCropAndScaleOption = .scaleFit
                let started = CACurrentMediaTime()
                try VNImageRequestHandler(cvPixelBuffer: capturedImage, orientation: imageOrientation).perform([request])
                let inferenceMilliseconds = (CACurrentMediaTime() - started) * 1000
                guard let output = (request.results?.first as? VNCoreMLFeatureValueObservation)?.featureValue.multiArrayValue,
                      output.shape.count == 3, output.shape[2].intValue >= 6 else { throw DetectorError.unexpectedOutput }
                // The upright image is portrait for .left/.right and landscape for .up/.down.
                let rawWidth = Float(CVPixelBufferGetWidth(capturedImage)), rawHeight = Float(CVPixelBufferGetHeight(capturedImage))
                let (uprightWidth, uprightHeight) = orientation == .up || orientation == .down ? (rawWidth, rawHeight) : (rawHeight, rawWidth)
                var detections: [ObjectSearchDetection] = [], rejected = 0, debugBoxes: [ObjectDebugBox] = []
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
                func vision(_ x: Float, _ y: Float) -> SIMD2<Float> {
                    PersonDepthGeometry.visionPoint(modelX: x, modelY: y, input: inputSize, imageWidth: uprightWidth, imageHeight: uprightHeight)
                }
                // End-to-end YOLO26 output: up to 300 rows of [x1, y1, x2, y2, score, class] in
                // model-input pixels, already free of duplicates, so no NMS here.
                for row in 0..<output.shape[1].intValue {
                    func value(_ k: Int) -> Float { output[[0, row, k] as [NSNumber]].floatValue }
                    let score = value(4)
                    guard score >= (debug ? Self.debugMinimumConfidence : Self.minimumConfidence), value(5).isFinite else { continue }
                    guard let label = labels[Int(value(5).rounded())] else { rejected += 1; continue }
                    let a = vision(value(0), value(1)), b = vision(value(2), value(3))
                    let box = CGRect(x: CGFloat(min(a.x, b.x)), y: CGFloat(min(a.y, b.y)), width: CGFloat(abs(b.x-a.x)), height: CGFloat(abs(b.y-a.y)))
                    var samples: [Float] = []
                    // Use the central region of the box to avoid the background at its edges.
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
                    let pixel = raw * SIMD2(Float(camera.imageResolution.width), Float(camera.imageResolution.height))
                    if debug {
                        debugBoxes.append(ObjectDebugBox(id: row, box: box, className: label.0, color: label.1,
                                                         confidence: score, depth: PersonDepthGeometry.measuredDepth(samples)))
                    }
                    guard score >= Self.minimumConfidence else { continue }
                    guard let measured = PersonDepthGeometry.measuredDepth(samples),
                          let world = PersonDepthGeometry.worldPoint(pixel: pixel, depth: measured, intrinsics: camera.intrinsics,
                                                                    cameraTransform: camera.transform, worldFromAR: worldFromAR) else { rejected += 1; continue }
                    detections.append(ObjectSearchDetection(className: label.0, color: label.1, confidence: Double(score), world: world))
                }
                if debug {
                    // Downscaled upright image, the same view the model letterboxed.
                    let upright = CIImage(cvPixelBuffer: capturedImage).oriented(imageOrientation)
                    let scale = 640 / max(upright.extent.width, upright.extent.height)
                    let small = upright.transformed(by: CGAffineTransform(scaleX: scale, y: scale))
                    let image = imageContext.createCGImage(small, from: small.extent)
                    let snapshot = ObjectDebugFrame(timestamp: timestamp, image: image, boxes: debugBoxes,
                                                    inferenceMilliseconds: inferenceMilliseconds, orientation: "\(orientation)",
                                                    imageSize: CGSize(width: CGFloat(uprightWidth), height: CGFloat(uprightHeight)))
                    lock.lock(); if debugEnabled { latestDebug = snapshot }; lock.unlock()
                }
                completion(.success(ObjectSearchFrame(timestamp: timestamp, frameID: id, detections: detections, rejected: rejected)))
            } catch { completion(.failure(error)) }
        }
    }
    enum DetectorError: Error { case depthUnavailable, modelMissing, unexpectedOutput, cameraVertical }
}
