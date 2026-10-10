import SwiftUI
import SceneKit

struct RoverModelView: View {
    @State private var scene: SCNScene?
    @State private var failed = false
    @State private var resetID = UUID()
    var body: some View {
        ZStack(alignment: .bottomTrailing) {
            Color(uiColor: .tertiarySystemGroupedBackground)
            if let scene {
                RoverSceneView(scene: scene).id(resetID)
                    .accessibilityLabel("Interactive Terra rover model")
                    .accessibilityHint("Drag to rotate. Pinch to zoom. Use Reset View to restore the camera.")
                Button { resetID = UUID() } label: {
                    Image(systemName: "arrow.counterclockwise")
                        .font(.subheadline.weight(.semibold)).frame(width: 44, height: 44)
                        .background(.regularMaterial, in: Circle())
                }.buttonStyle(.plain).padding(8).accessibilityLabel("Reset rover view")
            } else if failed {
                ContentUnavailableView("Model Unavailable", systemImage: "cube", description: Text("The rover model could not be loaded."))
            } else {
                ProgressView("Loading rover…").frame(maxWidth: .infinity, maxHeight: .infinity)
            }
        }
        .task {
            guard scene == nil, !failed else { return }
            // Decode off the UI thread, then publish the complete scene once.
            let loaded = await Task.detached(priority: .userInitiated) { Self.loadScene() }.value
            if let loaded { scene = loaded } else { failed = true }
        }
    }
    nonisolated private static func loadScene() -> SCNScene? {
        guard let url = Bundle.main.url(forResource: "rover", withExtension: "usdz", subdirectory: "Models"),
              let model = try? SCNScene(url: url, options: [.checkConsistency: true, .createNormalsIfAbsent: true]) else { return nil }
        let scene = SCNScene()
        let content = SCNNode()
        let orientation = SCNNode()
        orientation.eulerAngles.x = -.pi / 2 // CAD export uses Z up; SceneKit uses Y up.
        for child in model.rootNode.childNodes { orientation.addChildNode(child) }
        content.addChildNode(orientation)
        let paletteURL = Bundle.main.url(forResource: "materials", withExtension: "json", subdirectory: "Models")
        let palette = paletteURL.flatMap { try? Data(contentsOf: $0) }
            .flatMap { try? JSONDecoder().decode([String: [Double]].self, from: $0) } ?? [:]
        content.enumerateChildNodes { node, _ in
            for material in node.geometry?.materials ?? [] {
                if let color = palette[material.name ?? ""], color.count == 3 {
                    material.diffuse.contents = UIColor(red: color[0], green: color[1], blue: color[2], alpha: 1)
                }
                material.lightingModel = .blinn
                material.specular.contents = UIColor(white: 0.22, alpha: 1)
                material.shininess = 0.25
                material.isDoubleSided = true
            }
        }
        let (min, max) = content.boundingBox
        let width = max.x - min.x, height = max.y - min.y, depth = max.z - min.z
        let extent = Swift.max(width, Swift.max(height, depth))
        guard extent.isFinite, extent > 0 else { return nil }
        let scale: Float = 2.5 / extent
        content.scale = SCNVector3(scale, scale, scale)
        content.position = SCNVector3(-(min.x + max.x) * 0.5 * scale, -(min.y + max.y) * 0.5 * scale, -(min.z + max.z) * 0.5 * scale)
        scene.rootNode.addChildNode(content)
        let camera = SCNNode()
        camera.name = "overviewCamera"
        camera.camera = SCNCamera()
        camera.camera?.fieldOfView = 37
        camera.camera?.zNear = 0.01
        camera.camera?.zFar = 100
        camera.position = SCNVector3(2.8, 2.5, 3.6)
        camera.look(at: SCNVector3Zero)
        scene.rootNode.addChildNode(camera)
        let ambient = SCNNode()
        ambient.light = SCNLight()
        ambient.light?.type = .ambient
        ambient.light?.intensity = 300
        scene.rootNode.addChildNode(ambient)
        let key = SCNNode()
        key.light = SCNLight()
        key.light?.type = .omni
        key.light?.intensity = 700
        key.position = SCNVector3(-3, 5, 4)
        scene.rootNode.addChildNode(key)
        return scene
    }
}
