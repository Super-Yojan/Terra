import SwiftUI

/// Live view of the YOLO bucket/ball detector: the upright camera image the model saw,
/// every box with its score and LiDAR depth, and the run's object map. Works in the hand
/// with no rover connected.
struct ObjectDetectionDebugView: View {
    @ObservedObject var brain: PhoneController
    @State private var frame: ObjectDebugFrame?
    @State private var lastFrameAt: Date?
    @State private var now = Date()
    @State private var framesReceived = 0
    @State private var showFrom: Double = 0.25
    @State private var objects: [ObjectMapEntry] = []

    var body: some View {
        Form {
            Section {
                DetectionOverlay(frame: frame, showFrom: Float(showFrom))
                    .listRowInsets(EdgeInsets())
                Text(summary).font(.footnote).foregroundStyle(.secondary)
            } footer: {
                Text("Solid boxes reach the object map (score ≥ \(percent(ObjectSearchDetector.minimumConfidence))); dashed ones are below it. L4 search confirms a target only at 0.80 or more. Distance is LiDAR depth at the box centre; \"no depth\" boxes never reach the map.")
            }
            Section("Boxes on this frame") {
                Slider(value: $showFrom, in: Double(ObjectSearchDetector.debugMinimumConfidence)...0.9) {
                    Text("Show from")
                } minimumValueLabel: { Text(percent(ObjectSearchDetector.debugMinimumConfidence)).font(.caption) }
                  maximumValueLabel: { Text("0.90").font(.caption) }
                Text("Showing scores from \(String(format: "%.2f", showFrom))").font(.caption).foregroundStyle(.secondary)
                let shown = (frame?.boxes ?? []).filter { $0.confidence >= Float(showFrom) }.sorted { $0.confidence > $1.confidence }
                if shown.isEmpty {
                    Text(frame == nil ? "–" : "No boxes").foregroundStyle(.secondary)
                }
                ForEach(shown) { box in
                    LabeledContent {
                        Text(boxDetail(box)).monospacedDigit()
                    } label: {
                        Label { Text(name(box.className, box.color)) } icon: {
                            Image(systemName: "square.fill").foregroundStyle(Color(DetectionStyle.color(box.className, box.color)))
                        }
                    }
                }
            }
            Section {
                LabeledContent("Model", value: brain.objectDetectorStatus)
                LabeledContent("Inference", value: frame.map { String(format: "%.0f ms", $0.inferenceMilliseconds) } ?? "–")
                LabeledContent("Image to model", value: frame.map { "\(Int($0.imageSize.width))×\(Int($0.imageSize.height)) · \(orientationText($0.orientation))" } ?? "–")
            } header: {
                Text("Detector")
            } footer: {
                Text("Orientation comes from gravity, so a landscape-mounted phone feeds an upright landscape image. The model runs about five times a second, one frame at a time.")
            }
            Section {
                if objects.isEmpty { Text("No objects yet").foregroundStyle(.secondary) }
                ForEach(objects) { object in
                    VStack(alignment: .leading, spacing: 2) {
                        HStack {
                            Image(systemName: object.confirmed ? "checkmark.circle.fill" : "circle.dashed")
                                .foregroundStyle(Color(DetectionStyle.color(object.className, object.color)))
                            Text("\(name(object.className, object.color)) #\(object.id)")
                            Spacer()
                            Text("seen \(object.sightings)×").font(.caption).foregroundStyle(.secondary)
                        }
                        Text(String(format: "local %.2f, %.2f m · best %.2f", object.x, object.y, object.confidence))
                            .font(.caption).monospacedDigit()
                        if let lat = object.latitude, let lon = object.longitude {
                            Text(String(format: "%.6f, %.6f%@", lat, lon, object.geoAccuracy.map { String(format: " ±%.0f m", $0) } ?? ""))
                                .font(.caption).monospacedDigit()
                        } else {
                            Text("No lat/lon (GPS fix or heading not good enough)").font(.caption).foregroundStyle(.secondary)
                        }
                    }
                }
                Button("Clear object map", role: .destructive) { brain.clearObjectMap(); objects = [] }
            } header: {
                Text("Object map")
            } footer: {
                Text("Sightings of the same class and colour within 1 m merge. Three camera frames confirm an object; only confirmed objects appear in ARGOS.")
            }
            Section("Phone sensors") {
                LabeledContent("Tracking", value: brain.phoneTrackingStatus)
                Button(frame == nil ? "Start phone sensors" : "Restart phone sensors") { frame = nil; brain.startPhone() }
                Text("No rover is needed. Phone mode runs ARKit and scene depth, which the detector needs; motors stay off without a Bluetooth link.")
                    .font(.footnote).foregroundStyle(.secondary)
            }
        }
        .navigationTitle("Object detection")
        .onDisappear { brain.setObjectDetectionDebug(false) }
        // Poll from a task tied to the page's lifetime (see ClassificationDebugView).
        .task {
            brain.setObjectDetectionDebug(true)
            var ticks = 0
            while !Task.isCancelled {
                try? await Task.sleep(nanoseconds: 200_000_000)
                now = Date()
                brain.takeObjectDebugFrame { next in
                    guard let next else { return }
                    frame = next; lastFrameAt = Date(); framesReceived += 1
                }
                ticks += 1
                if ticks % 5 == 0 {
                    brain.objectMapJSON { json in objects = ObjectMapEntry.decode(json) }
                }
            }
        }
    }

    private var summary: String {
        guard let frame, let lastFrameAt else {
            return "Waiting for detector frames. Start phone sensors; the detector needs camera tracking and LiDAR scene depth."
        }
        let age = now.timeIntervalSince(lastFrameAt)
        let passing = frame.boxes.filter { $0.confidence >= ObjectSearchDetector.minimumConfidence }.count
        let status = "\(frame.boxes.count) boxes ≥ \(percent(ObjectSearchDetector.debugMinimumConfidence)), \(passing) ≥ \(percent(ObjectSearchDetector.minimumConfidence)) · \(framesReceived) frames"
        return age > 2 ? String(format: "%@ · no new frame for %.0f s", status, age) : status
    }
    private func boxDetail(_ box: ObjectDebugBox) -> String {
        let depth = box.depth.map { String(format: "%.2f m", $0) } ?? "no depth"
        return String(format: "%.2f · %@", box.confidence, depth)
    }
    private func percent(_ value: Float) -> String { String(format: "%.2f", value) }
    private func orientationText(_ orientation: String) -> String {
        orientation == "up" || orientation == "down" ? "landscape (\(orientation))" : "portrait (\(orientation))"
    }
}

private func name(_ className: String, _ color: String?) -> String {
    color.map { "\($0) \(className)" } ?? className
}

enum DetectionStyle {
    static func color(_ className: String, _ color: String?) -> UIColor {
        switch (className, color) {
        case ("bucket", "blue"): return UIColor(red: 0.18, green: 0.48, blue: 1, alpha: 1)
        case ("bucket", "orange"): return UIColor(red: 1, green: 0.54, blue: 0.12, alpha: 1)
        case ("bucket", _): return .systemPink
        default: return UIColor(red: 0.96, green: 0.88, blue: 0.3, alpha: 1)
        }
    }
}

/// The detector's input image with its boxes drawn on top.
private struct DetectionOverlay: View {
    let frame: ObjectDebugFrame?
    let showFrom: Float
    var body: some View {
        if let frame, let image = frame.image {
            let aspect = CGFloat(image.width) / CGFloat(image.height)
            Image(decorative: image, scale: 1)
                .resizable()
                .aspectRatio(aspect, contentMode: .fit)
                .overlay {
                    GeometryReader { geometry in
                        ForEach(frame.boxes.filter { $0.confidence >= showFrom }) { box in
                            let rect = CGRect(x: box.box.minX * geometry.size.width,
                                              y: (1 - box.box.maxY) * geometry.size.height,
                                              width: box.box.width * geometry.size.width,
                                              height: box.box.height * geometry.size.height)
                            let color = Color(DetectionStyle.color(box.className, box.color))
                            let passes = box.confidence >= ObjectSearchDetector.minimumConfidence
                            Rectangle()
                                .stroke(color, style: StrokeStyle(lineWidth: passes ? 3 : 1.5, dash: passes ? [] : [5, 4]))
                                .frame(width: rect.width, height: rect.height)
                                .overlay(alignment: .topLeading) {
                                    Text(String(format: "%@ %.2f%@", name(box.className, box.color), box.confidence,
                                                box.depth.map { String(format: " · %.1f m", $0) } ?? ""))
                                        .font(.caption2.bold())
                                        .padding(.horizontal, 3)
                                        .background(color.opacity(0.85))
                                        .foregroundStyle(.black)
                                        .fixedSize()
                                        .offset(y: -16)
                                }
                                .position(x: rect.midX, y: rect.midY)
                        }
                    }
                }
        } else {
            Rectangle().fill(.black).aspectRatio(4 / 3, contentMode: .fit)
                .overlay { Text("No detector frame yet").foregroundStyle(.secondary) }
        }
    }
}

/// One entry of the Rust object map (`objects` topic body).
struct ObjectMapEntry: Identifiable, Decodable {
    let id: Int
    let className: String
    let color: String?
    let x: Double
    let y: Double
    let latitude: Double?
    let longitude: Double?
    let geoAccuracy: Double?
    let confidence: Double
    let sightings: Int
    let confirmed: Bool
    enum CodingKeys: String, CodingKey {
        case id, color, x, y, latitude, longitude, confidence, sightings, confirmed
        case className = "class", geoAccuracy = "geo_accuracy_m"
    }
    private struct Body: Decodable { let objects: [ObjectMapEntry] }
    static func decode(_ json: String) -> [ObjectMapEntry] {
        guard let data = json.data(using: .utf8), let body = try? JSONDecoder().decode(Body.self, from: data) else { return [] }
        return body.objects
    }
}
