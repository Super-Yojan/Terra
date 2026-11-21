import SceneKit
import SwiftUI
import simd

/// What the 3D rover shows, and where its camera looks from.
enum RoverViewMode: String, CaseIterable, Identifiable {
    case ports, forward, height, angle, size
    var id: String { rawValue }
    var title: String {
        switch self {
        case .ports: "Ports"
        case .forward: "Ahead"
        case .height: "Height"
        case .angle: "Angle"
        case .size: "Size"
        }
    }
}

/// Line colours, shared with the text fields under the view.
enum DimensionColor {
    static let forward = Color(red: 1.0, green: 0.6, blue: 0.2)
    static let height = Color(red: 0.35, green: 0.85, blue: 0.45)
    static let size = Color(red: 0.4, green: 0.65, blue: 1.0)
    static let view = Color(red: 1.0, green: 0.85, blue: 0.25)   // camera field of view
}

/// The rover in 3D (SceneKit), built from the setup: body length and width,
/// wheels, steering, and the phone where it is mounted. Drag to orbit.
/// - ports: top view; tap a wheel to choose its port.
/// - forward: line from the rover's centre to the phone.
/// - height: side view, line from the ground up to the phone.
/// - angle: close-up of the phone; the camera's field of view shows where it looks.
/// - size: lines along the body's length and width.
struct RoverModelView: UIViewRepresentable {
    let setup: VehicleSetup
    let mode: RoverViewMode
    var onTapOutput: ((Output) -> Void)?

    func makeCoordinator() -> RoverScene { RoverScene() }

    func makeUIView(context: Context) -> SCNView {
        let view = SCNView()
        view.scene = context.coordinator.scene
        view.pointOfView = context.coordinator.camera
        view.backgroundColor = .clear
        view.antialiasingMode = .multisampling4X
        view.allowsCameraControl = true
        view.defaultCameraController.interactionMode = .orbitTurntable
        view.addGestureRecognizer(UITapGestureRecognizer(target: context.coordinator,
                                                         action: #selector(RoverScene.tapped(_:))))
        return view
    }

    func updateUIView(_ view: SCNView, context: Context) {
        context.coordinator.onTapOutput = mode == .ports ? onTapOutput : nil
        context.coordinator.update(setup: setup, mode: mode, view: view)
    }
}

/// Builds and updates the scene. Rover body frame (+X forward, +Y left, +Z
/// up, origin at the centre on the ground) maps to SceneKit (+Y up, -Z ahead).
final class RoverScene: NSObject {
    let scene = SCNScene()
    let camera = SCNNode()
    var onTapOutput: ((Output) -> Void)?
    private let rover = SCNNode()
    private var built: (VehicleSetup, RoverViewMode)?
    private var outputsByWheel: [String: Output] = [:]

    override init() {
        super.init()
        camera.camera = SCNCamera()
        camera.camera?.fieldOfView = 38
        camera.camera?.zNear = 0.01
        scene.rootNode.addChildNode(camera)
        scene.rootNode.addChildNode(rover)

        let ambient = SCNNode(); ambient.light = SCNLight(); ambient.light?.type = .ambient
        ambient.light?.intensity = 350
        let key = SCNNode(); key.light = SCNLight(); key.light?.type = .directional
        key.light?.intensity = 900; key.eulerAngles = SCNVector3(-1.0, 0.6, 0)
        let fill = SCNNode(); fill.light = SCNLight(); fill.light?.type = .directional
        fill.light?.intensity = 300; fill.eulerAngles = SCNVector3(-0.4, -2.4, 0)
        [ambient, key, fill].forEach(scene.rootNode.addChildNode)

        let floor = SCNFloor()
        floor.reflectivity = 0
        floor.firstMaterial?.diffuse.contents = UIColor(white: 0.07, alpha: 1)
        scene.rootNode.addChildNode(SCNNode(geometry: floor))
    }

    /// Body frame (x fwd, y left, z up) → SceneKit.
    private func p(_ x: Double, _ y: Double, _ z: Double) -> SCNVector3 {
        SCNVector3(Float(-y), Float(z), Float(-x))
    }

    func update(setup: VehicleSetup, mode: RoverViewMode, view: SCNView) {
        if let built, built.0 == setup, built.1 == mode { return }
        let modeChanged = built?.1 != mode
        built = (setup, mode)
        rover.childNodes.forEach { $0.removeFromParentNode() }
        build(setup, mode)
        if modeChanged { aim(setup, mode, view: view) }
    }

    // MARK: Geometry

    private func build(_ s: VehicleSetup, _ mode: RoverViewMode) {
        let length = max(0.05, s.bodyLength), width = max(0.05, s.bodyWidth)
        let axles = s.axleCount
        let wheelR = min(0.12, max(0.035, length / (Double(axles) * 4.5)))
        let wheelW = wheelR * 0.65
        let bodyH = min(0.14, max(0.05, width * 0.25))
        let bodyBottom = wheelR * 0.75
        let bodyTop = bodyBottom + bodyH

        // Body.
        let box = SCNBox(width: CGFloat(width), height: CGFloat(bodyH), length: CGFloat(length),
                         chamferRadius: CGFloat(min(0.025, bodyH * 0.3)))
        box.firstMaterial = material(UIColor(white: 0.24, alpha: 1))
        let body = SCNNode(geometry: box)
        body.position = p(0, 0, bodyBottom + bodyH / 2)
        rover.addChildNode(body)
        // Centre mark on top of the body (where the forward line starts).
        let mark = SCNNode(geometry: SCNCylinder(radius: CGFloat(width * 0.05), height: 0.002))
        mark.geometry?.firstMaterial = material(UIColor(white: 0.6, alpha: 1), emissive: true)
        mark.position = p(0, 0, bodyTop + 0.001)
        rover.addChildNode(mark)

        // Wheels, front axle first.
        outputsByWheel = [:]
        for wheel in s.wheels {
            let left = wheel.hasPrefix("L"), axle = Int(wheel.dropFirst()) ?? 0
            let x = axles == 1 ? 0 : length / 2 - wheelR - Double(axle) * (length - 2 * wheelR) / Double(axles - 1)
            let y = (width / 2 + wheelW / 2 + 0.004) * (left ? 1 : -1)
            let steered = s.steering == .ackermann && axle == 0
            let output = s.outputs.first { $0.wheels.contains(wheel) && ($0.role == .steering) == steered }
            outputsByWheel[wheel] = output
            let port = output.flatMap { s.assignments[$0.id]?.port }

            let tire = SCNCylinder(radius: CGFloat(wheelR), height: CGFloat(wheelW))
            let wired = mode == .ports && port != nil
            tire.firstMaterial = material(wired ? UIColor(Theme.accent) : UIColor(white: 0.08, alpha: 1))
            let node = SCNNode(geometry: tire)
            node.name = "wheel:\(wheel)"
            node.position = p(x, y, wheelR)
            node.eulerAngles = SCNVector3(0, steered ? 0.26 : 0, Float.pi / 2)
            rover.addChildNode(node)

            if mode == .ports {
                let text = steered ? "steer \(port ?? "–")" : (port ?? "Set port")
                let label = label(text, color: port == nil ? UIColor(Theme.accent) : .white, height: wheelR * 0.9)
                label.position = p(x, y + (left ? 1 : -1) * wheelR * 1.4, wheelR * 2.4)
                rover.addChildNode(label)
            }
        }

        // Phone, with its camera, rotated exactly as the mount sent to terra-av.
        rover.addChildNode(phone(s, showView: mode == .angle || mode == .forward))
        if s.phoneHeight - 0.04 > bodyTop {
            addLine(from: p(s.phoneForward, 0, bodyTop), to: p(s.phoneForward, 0, s.phoneHeight - 0.04),
                    color: UIColor(white: 0.35, alpha: 1), radius: 0.006)
        }

        // Dimension lines for the mode.
        let gap = width / 2 + wheelW + 0.06
        switch mode {
        case .ports, .angle:
            break
        case .forward:
            let z = bodyTop + 0.012
            addLine(from: p(0, 0, z), to: p(s.phoneForward, 0, z), color: UIColor(DimensionColor.forward))
            addLine(from: p(s.phoneForward, 0, z), to: p(s.phoneForward, 0, s.phoneHeight),
                    color: UIColor(DimensionColor.forward).withAlphaComponent(0.4), radius: 0.003)
            addLabel(String(format: "%.2f m", s.phoneForward), color: DimensionColor.forward,
                     at: p(s.phoneForward / 2, 0, z + 0.05), scale: width)
        case .height:
            addLine(from: p(s.phoneForward, -gap, 0.002), to: p(s.phoneForward, -gap, s.phoneHeight),
                    color: UIColor(DimensionColor.height))
            addLine(from: p(s.phoneForward, -gap, s.phoneHeight), to: p(s.phoneForward, 0, s.phoneHeight),
                    color: UIColor(DimensionColor.height).withAlphaComponent(0.4), radius: 0.003)
            addLabel(String(format: "%.2f m", s.phoneHeight), color: DimensionColor.height,
                     at: p(s.phoneForward, -gap - 0.06, s.phoneHeight / 2), scale: width)
        case .size:
            addLine(from: p(-length / 2, -gap, 0.003), to: p(length / 2, -gap, 0.003), color: UIColor(DimensionColor.size))
            addLine(from: p(length / 2 + 0.06, -width / 2, 0.003), to: p(length / 2 + 0.06, width / 2, 0.003),
                    color: UIColor(DimensionColor.size))
            addLabel(String(format: "L %.2f m", length), color: DimensionColor.size,
                     at: p(0, -gap - 0.05, 0.05), scale: width)
            addLabel(String(format: "W %.2f m", width), color: DimensionColor.size,
                     at: p(length / 2 + 0.12, 0, 0.05), scale: width)
        }
    }

    private func material(_ color: UIColor, emissive: Bool = false) -> SCNMaterial {
        let m = SCNMaterial()
        m.diffuse.contents = color
        m.lightingModel = .blinn
        m.specular.contents = UIColor(white: 0.25, alpha: 1)
        if emissive { m.emission.contents = color.withAlphaComponent(0.35) }
        return m
    }

    /// The phone (iPhone 12 Pro size) built in ARKit camera axes: +X right,
    /// +Y up, screen on +Z, lens on the back looking along -Z. The rear
    /// camera module sits in the top-right corner seen from behind (ARKit's
    /// native landscape). `showView` adds the field of view: a pyramid out of
    /// the lens with a tick marking the top of the image.
    private func phone(_ s: VehicleSetup, showView: Bool) -> SCNNode {
        let node = SCNNode()
        let w = 0.1467, h = 0.0715, t = 0.0074
        let body = SCNNode(geometry: SCNBox(width: CGFloat(w), height: CGFloat(h), length: CGFloat(t), chamferRadius: 0.006))
        body.geometry?.firstMaterial = material(UIColor(white: 0.55, alpha: 1))
        node.addChildNode(body)
        let screen = SCNNode(geometry: SCNPlane(width: CGFloat(w - 0.008), height: CGFloat(h - 0.008)))
        screen.geometry?.firstMaterial = material(UIColor(Theme.accent), emissive: true)
        screen.position = SCNVector3(0, 0, Float(t / 2 + 0.0004))
        node.addChildNode(screen)

        // Camera module on the back, and three lenses.
        let lensCenter = SIMD3<Double>(w / 2 - 0.022, h / 2 - 0.02, -t / 2)
        let module = SCNNode(geometry: SCNBox(width: 0.034, height: 0.034, length: 0.003, chamferRadius: 0.006))
        module.geometry?.firstMaterial = material(UIColor(white: 0.2, alpha: 1))
        module.position = SCNVector3(Float(lensCenter.x), Float(lensCenter.y), Float(lensCenter.z - 0.0015))
        node.addChildNode(module)
        for (dx, dy) in [(-0.0085, 0.0085), (-0.0085, -0.0085), (0.0085, 0.0)] {
            let lens = SCNNode(geometry: SCNCylinder(radius: 0.0055, height: 0.003))
            lens.geometry?.firstMaterial = material(UIColor(white: 0.03, alpha: 1))
            lens.geometry?.firstMaterial?.specular.contents = UIColor.white
            lens.eulerAngles = SCNVector3(Float.pi / 2, 0, 0)
            lens.position = SCNVector3(Float(lensCenter.x + dx), Float(lensCenter.y + dy), Float(lensCenter.z - 0.0045))
            node.addChildNode(lens)
        }

        if showView {
            // ARKit's 4:3 image, ~63° wide.
            let d = 0.32, hw = d * tan(31.5 * .pi / 180), hh = hw * 0.75
            let apex = SCNVector3(Float(lensCenter.x), Float(lensCenter.y), Float(lensCenter.z - 0.006))
            let corners = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)].map {
                SCNVector3(Float(lensCenter.x + $0.0), Float(lensCenter.y + $0.1), Float(lensCenter.z - d))
            }
            let color = UIColor(DimensionColor.view)
            for i in 0..<4 {
                addLine(from: apex, to: corners[i], color: color.withAlphaComponent(0.6), radius: 0.002, parent: node, dots: false)
                addLine(from: corners[i], to: corners[(i + 1) % 4], color: color, radius: 0.0025, parent: node, dots: false)
            }
            // Tick above the top edge: the top of the camera image.
            let top = SCNVector3(Float(lensCenter.x), Float(lensCenter.y + hh * 1.25), Float(lensCenter.z - d))
            addLine(from: SCNVector3(Float(lensCenter.x), Float(lensCenter.y + hh), Float(lensCenter.z - d)), to: top,
                    color: color, radius: 0.0035, parent: node, dots: false)
            let face = SCNNode(geometry: SCNPlane(width: CGFloat(hw * 2), height: CGFloat(hh * 2)))
            let m = material(color.withAlphaComponent(0.08), emissive: true)
            m.isDoubleSided = true
            m.writesToDepthBuffer = false
            face.geometry?.firstMaterial = m
            face.position = SCNVector3(Float(lensCenter.x), Float(lensCenter.y), Float(lensCenter.z - d))
            node.addChildNode(face)
        }

        // Camera axes → body (the mount) → SceneKit, plus the position.
        let toScene = simd_double3x3(rows: [SIMD3(0, -1, 0), SIMD3(0, 0, 1), SIMD3(-1, 0, 0)])
        let r = toScene * s.cameraToBody
        let pos = p(s.phoneForward, 0, s.phoneHeight)
        func col(_ c: Int) -> SIMD4<Float> { SIMD4(Float(r[c].x), Float(r[c].y), Float(r[c].z), 0) }
        node.simdTransform = simd_float4x4(col(0), col(1), col(2), SIMD4(pos.x, pos.y, pos.z, 1))
        return node
    }

    private func addLine(from a: SCNVector3, to b: SCNVector3, color: UIColor, radius: Double = 0.005,
                         parent: SCNNode? = nil, dots: Bool = true) {
        let dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z
        let length = sqrt(dx * dx + dy * dy + dz * dz)
        guard length > 0.001 else { return }
        let rod = SCNCylinder(radius: CGFloat(radius), height: CGFloat(length))
        rod.firstMaterial = material(color, emissive: true)
        let node = SCNNode(geometry: rod)
        node.position = SCNVector3((a.x + b.x) / 2, (a.y + b.y) / 2, (a.z + b.z) / 2)
        let holder = parent ?? rover
        holder.addChildNode(node)
        // Orient in the holder's space (look(at:) works in world space).
        let dir = simd_normalize(SIMD3<Float>(dx, dy, dz))
        let yAxis = SIMD3<Float>(0, 1, 0)
        let axis = simd_cross(yAxis, dir)
        let angle = acos(max(-1, min(1, simd_dot(yAxis, dir))))
        node.simdOrientation = simd_length(axis) < 1e-5
            ? (simd_dot(yAxis, dir) > 0 ? simd_quatf(angle: 0, axis: SIMD3(1, 0, 0)) : simd_quatf(angle: .pi, axis: SIMD3(1, 0, 0)))
            : simd_quatf(angle: angle, axis: simd_normalize(axis))
        guard dots else { return }
        for end in [a, b] {
            let dot = SCNNode(geometry: SCNSphere(radius: CGFloat(radius * 2.2)))
            dot.geometry?.firstMaterial = material(color, emissive: true)
            dot.position = end
            holder.addChildNode(dot)
        }
    }

    private func addLabel(_ text: String, color: Color, at position: SCNVector3, scale: Double) {
        let node = label(text, color: UIColor(color), height: max(0.035, scale * 0.11))
        node.position = position
        rover.addChildNode(node)
    }

    /// A flat text tag that always faces the camera and draws on top.
    private func label(_ text: String, color: UIColor, height: Double) -> SCNNode {
        let font = UIFont.monospacedSystemFont(ofSize: 44, weight: .semibold)
        let textSize = (text as NSString).size(withAttributes: [.font: font])
        let size = CGSize(width: textSize.width + 36, height: textSize.height + 18)
        let image = UIGraphicsImageRenderer(size: size).image { _ in
            UIColor(white: 0.08, alpha: 0.9).setFill()
            UIBezierPath(roundedRect: CGRect(origin: .zero, size: size), cornerRadius: size.height / 2).fill()
            (text as NSString).draw(at: CGPoint(x: 18, y: 9), withAttributes: [.font: font, .foregroundColor: color])
        }
        let plane = SCNPlane(width: CGFloat(height) * size.width / size.height, height: CGFloat(height))
        let m = SCNMaterial()
        m.diffuse.contents = image
        m.lightingModel = .constant
        m.readsFromDepthBuffer = false
        m.isDoubleSided = true
        plane.firstMaterial = m
        let node = SCNNode(geometry: plane)
        node.renderingOrder = 10
        node.constraints = [SCNBillboardConstraint()]
        return node
    }

    // MARK: Camera

    private func aim(_ s: VehicleSetup, _ mode: RoverViewMode, view: SCNView) {
        let size = max(s.bodyLength, s.bodyWidth, s.phoneHeight, abs(s.phoneForward) * 2)
        let d = size * 2.3
        let eye: SCNVector3, target: SCNVector3, up: SCNVector3
        switch mode {
        case .ports:
            eye = p(-d * 0.15, 0, d * 1.15); target = p(0, 0, 0); up = p(1, 0, 0)
        case .forward:
            eye = p(-d * 0.75, -d * 0.7, d * 0.75); target = p(s.phoneForward / 2, 0, s.phoneHeight / 2); up = p(0, 0, 1)
        case .angle:
            eye = p(s.phoneForward - 0.45, -0.5, s.phoneHeight + 0.3)
            target = p(s.phoneForward + 0.12, 0, s.phoneHeight); up = p(0, 0, 1)
        case .height:
            eye = p(s.phoneForward / 2, -d * 1.15, s.phoneHeight * 0.55 + 0.05)
            target = p(s.phoneForward / 2, 0, s.phoneHeight / 2); up = p(0, 0, 1)
        case .size:
            eye = p(d * 0.8, -d * 0.8, d * 0.8); target = p(0, 0, 0.05); up = p(0, 0, 1)
        }
        view.pointOfView = camera
        SCNTransaction.begin()
        SCNTransaction.animationDuration = 0.5
        camera.position = eye
        camera.look(at: target, up: up, localFront: SCNVector3(0, 0, -1))
        view.defaultCameraController.target = target
        SCNTransaction.commit()
    }

    // MARK: Tap

    @objc func tapped(_ gesture: UITapGestureRecognizer) {
        guard let onTapOutput, let view = gesture.view as? SCNView else { return }
        let hits = view.hitTest(gesture.location(in: view), options: [.searchMode: SCNHitTestSearchMode.all.rawValue])
        for hit in hits {
            if let name = hit.node.name, name.hasPrefix("wheel:"), let output = outputsByWheel[String(name.dropFirst(6))] {
                onTapOutput(output)
                return
            }
        }
    }
}
