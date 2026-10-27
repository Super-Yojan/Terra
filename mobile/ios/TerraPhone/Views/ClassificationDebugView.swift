import SwiftUI
import SceneKit

/// Live 3D view of how the mapper labels each depth return, for checking the
/// classifier and the ARKit floor veto on the phone, held in the hand, with no
/// rover connected. Labels come from Rust with the grid's own rules.
struct ClassificationDebugView: View {
    @ObservedObject var brain: PhoneController
    @State private var frame: ClassifiedFrame?
    @State private var lastFrameAt: Date?
    @State private var now = Date()
    @State private var showOutOfRange = false
    @State private var showFloorCells = true
    @State private var fineDetail = false
    @State private var framesReceived = 0

    var body: some View {
        Form {
            Section {
                ClassifiedCloudView(frame: frame, showOutOfRange: showOutOfRange, showFloorCells: showFloorCells)
                    .frame(height: 380)
                    .listRowInsets(EdgeInsets())
                Text(summary).font(.footnote).foregroundStyle(.secondary)
            } footer: {
                Text("Drag to orbit, pinch to zoom. The white dot is the phone; up is world up. Mesh floor cells are what ARKit classified as floor.")
            }
            Section {
                LabeledContent("ARKit floor mesh", value: below(frame?.meshFloorZ))
                LabeledContent("Lowest depth returns", value: below(frame?.lowReturnZ))
                LabeledContent("Classifier ground plane", value: frame.map { $0.groundZ == nil ? "not locked" : below($0.groundZ) } ?? "–")
                if let mismatch { Text(mismatch).font(.footnote).foregroundStyle(.orange) }
            } header: {
                Text("Height below the phone")
            } footer: {
                Text("Point the camera at open floor. All three should agree within a few centimetres; obstacles start 0.15 m above the classifier ground.")
            }
            Section("Labels") {
                ForEach(PointLabelStyle.all, id: \.byte) { style in
                    LabeledContent {
                        Text(frame.map { count($0, style.byte) } ?? "–").monospacedDigit()
                    } label: {
                        Label { Text(style.name) } icon: {
                            Image(systemName: "circle.fill").foregroundStyle(Color(style.color))
                        }
                    }
                }
                LabeledContent {
                    Text(frame.map { "\($0.floorCells.count / 3)" } ?? "–").monospacedDigit()
                } label: {
                    Label { Text("Mesh floor cells") } icon: {
                        Image(systemName: "square.fill").foregroundStyle(Color(PointLabelStyle.floorCell))
                    }
                }
            }
            Section("View") {
                Toggle("Show out-of-range returns", isOn: $showOutOfRange)
                Toggle("Show mesh floor cells", isOn: $showFloorCells)
                Toggle("Finer points (slower)", isOn: $fineDetail)
            }
            Section("Phone sensors") {
                LabeledContent("Tracking", value: brain.phoneTrackingStatus)
                Text(brain.mapStatus).font(.footnote)
                Button(frame == nil ? "Start phone sensors" : "Restart phone sensors") { frame = nil; brain.startPhone() }
                Text("No rover is needed. Phone mode runs ARKit, scene depth and the mapper; motors stay off without a Bluetooth link. Mount calibration still sets the floor height if it is saved.")
                    .font(.footnote).foregroundStyle(.secondary)
            }
        }
        .navigationTitle("3D classification")
        .onDisappear { brain.setClassificationDebug(false) }
        // Poll from a task tied to the page's lifetime. A Timer.publish stored on
        // the view is rebuilt every time the controller publishes (10 Hz), so its
        // 0.5 s tick never arrived and the page stayed on "Waiting for depth frames".
        .task {
            brain.setClassificationDebug(true)
            while !Task.isCancelled {
                try? await Task.sleep(nanoseconds: 500_000_000)
                now = Date()
                brain.takeClassifiedFrame(stride: fineDetail ? 2 : 4) { next in
                    guard let next else { return }
                    frame = next; lastFrameAt = Date(); framesReceived += 1
                }
            }
        }
    }

    private var summary: String {
        guard let frame, let lastFrameAt else {
            return "Waiting for depth frames. Start phone sensors and point the camera at the floor and nearby objects."
        }
        let mode = frame.classifier == "ground_plane"
            ? "Ground plane: floor locked, obstacles are 0.15–2 m above it"
            : "Level band: floor not locked yet, every return in the band is an obstacle"
        let age = now.timeIntervalSince(lastFrameAt)
        let status = age > 2 ? String(format: "%@ · no new frame for %.0f s", mode, age) : mode
        return "\(status) · \(framesReceived) frames"
    }
    /// How far below the phone a world height is, as text.
    private func below(_ z: Double?) -> String {
        guard let z, let frame, frame.camera.count == 3 else { return "–" }
        return String(format: "%.2f m", frame.camera[2] - z)
    }
    private var mismatch: String? {
        guard let frame, let mesh = frame.meshFloorZ else { return nil }
        if let low = frame.lowReturnZ, abs(mesh - low) > 0.15 {
            return String(format: "Mesh floor and depth floor differ by %.2f m.", mesh - low)
        }
        if let ground = frame.groundZ, abs(mesh - ground) > 0.08 {
            return String(format: "Classifier ground is %.2f m %@ the mesh floor; check the mount height or restart sensors.", abs(ground - mesh), ground > mesh ? "above" : "below")
        }
        return nil
    }
    private func count(_ frame: ClassifiedFrame, _ byte: UInt8) -> String {
        Int(byte) < frame.counts.count ? "\(frame.counts[Int(byte)])" : "0"
    }
}

/// Colours shared with ARGOS captures, keyed by the point-label wire byte.
enum PointLabelStyle {
    struct Entry { let byte: UInt8; let name: String; let color: UIColor }
    static let outOfRange: UInt8 = 3
    static let all: [Entry] = [
        Entry(byte: 1, name: "Obstacle", color: rgb(218, 54, 51)),
        Entry(byte: 6, name: "Floor veto (ARKit floor)", color: rgb(32, 178, 170)),
        Entry(byte: 0, name: "Free", color: rgb(46, 160, 67)),
        Entry(byte: 2, name: "Overhead", color: rgb(56, 132, 244)),
        Entry(byte: 4, name: "Outside band", color: rgb(230, 180, 34)),
        Entry(byte: 3, name: "Out of range", color: rgb(140, 140, 140)),
    ]
    static let floorCell = rgb(148, 103, 189)
    static func rgba(_ byte: UInt8) -> SIMD4<Float> {
        let color = all.first { $0.byte == byte }?.color ?? .white
        var r: CGFloat = 0, g: CGFloat = 0, b: CGFloat = 0, a: CGFloat = 0
        color.getRed(&r, green: &g, blue: &b, alpha: &a)
        return SIMD4(Float(r), Float(g), Float(b), byte == outOfRange ? 0.4 : 1)
    }
    private static func rgb(_ r: Int, _ g: Int, _ b: Int) -> UIColor {
        UIColor(red: CGFloat(r) / 255, green: CGFloat(g) / 255, blue: CGFloat(b) / 255, alpha: 1)
    }
}

/// Orbitable point cloud of one labelled frame, centred on the phone.
struct ClassifiedCloudView: UIViewRepresentable {
    let frame: ClassifiedFrame?
    let showOutOfRange: Bool
    let showFloorCells: Bool

    final class Coordinator {
        let cloud = SCNNode()
        let floor = SCNNode()
        var shown: (Double, Bool, Bool)?
    }
    func makeCoordinator() -> Coordinator { Coordinator() }
    func makeUIView(context: Context) -> SCNView {
        let scene = SCNScene()
        scene.rootNode.addChildNode(context.coordinator.floor)
        scene.rootNode.addChildNode(context.coordinator.cloud)
        let phone = SCNNode(geometry: SCNSphere(radius: 0.04))
        phone.geometry?.firstMaterial?.diffuse.contents = UIColor.white
        phone.geometry?.firstMaterial?.lightingModel = .constant
        scene.rootNode.addChildNode(phone)
        let camera = SCNNode()
        camera.camera = SCNCamera()
        camera.camera?.zNear = 0.01
        camera.position = SCNVector3(0, 2.5, 3.5)
        camera.look(at: SCNVector3Zero)
        scene.rootNode.addChildNode(camera)
        let view = SCNView()
        view.scene = scene
        view.pointOfView = camera
        view.backgroundColor = .black
        view.allowsCameraControl = true
        view.defaultCameraController.interactionMode = .orbitTurntable
        view.defaultCameraController.target = SCNVector3Zero
        view.defaultCameraController.inertiaEnabled = false
        return view
    }
    func updateUIView(_ view: SCNView, context: Context) {
        let coordinator = context.coordinator
        guard let frame, frame.camera.count == 3 else {
            coordinator.cloud.geometry = nil; coordinator.floor.geometry = nil; coordinator.shown = nil
            return
        }
        // The parent redraws whenever the controller publishes; rebuild only on change.
        let key = (frame.timestamp, showOutOfRange, showFloorCells)
        if let shown = coordinator.shown, shown == key { return }
        coordinator.shown = key
        let origin = SIMD3(Float(frame.camera[0]), Float(frame.camera[1]), Float(frame.camera[2]))
        var points: [SIMD3<Float>] = [], colors: [SIMD4<Float>] = []
        for (i, label) in frame.labels.enumerated() where 3 * i + 2 < frame.points.count {
            if label == PointLabelStyle.outOfRange && !showOutOfRange { continue }
            points.append(SIMD3(frame.points[3 * i], frame.points[3 * i + 1], frame.points[3 * i + 2]) - origin)
            colors.append(PointLabelStyle.rgba(label))
        }
        coordinator.cloud.geometry = Self.points(points, colors: colors, size: 4)
        if showFloorCells {
            let cells = stride(from: 0, to: frame.floorCells.count - 2, by: 3).map {
                SIMD3(frame.floorCells[$0], frame.floorCells[$0 + 1], frame.floorCells[$0 + 2]) - origin
            }
            let c = PointLabelStyle.floorCell.cgColor.components ?? [1, 1, 1, 1]
            let colour = SIMD4(Float(c[0]), Float(c[1]), Float(c[2]), 0.35)
            coordinator.floor.geometry = Self.points(cells, colors: Array(repeating: colour, count: cells.count), size: 7)
        } else {
            coordinator.floor.geometry = nil
        }
    }
    /// Terra world is Z-up; SceneKit is Y-up.
    private static func sceneVector(_ p: SIMD3<Float>) -> SCNVector3 { SCNVector3(p.x, p.z, -p.y) }
    private static func points(_ points: [SIMD3<Float>], colors: [SIMD4<Float>], size: CGFloat) -> SCNGeometry? {
        guard !points.isEmpty, points.count == colors.count else { return nil }
        let vertices = SCNGeometrySource(vertices: points.map(sceneVector))
        let colorData = colors.withUnsafeBufferPointer { Data(buffer: $0) }
        let colorSource = SCNGeometrySource(data: colorData, semantic: .color, vectorCount: colors.count,
                                            usesFloatComponents: true, componentsPerVector: 4,
                                            bytesPerComponent: MemoryLayout<Float>.size, dataOffset: 0,
                                            dataStride: MemoryLayout<SIMD4<Float>>.stride)
        let element = SCNGeometryElement(indices: (0..<Int32(points.count)).map { $0 }, primitiveType: .point)
        element.pointSize = size
        element.minimumPointScreenSpaceRadius = size / 2
        element.maximumPointScreenSpaceRadius = size
        let geometry = SCNGeometry(sources: [vertices, colorSource], elements: [element])
        let material = SCNMaterial()
        material.lightingModel = .constant
        material.isDoubleSided = true
        material.blendMode = .alpha
        geometry.materials = [material]
        return geometry
    }
}
