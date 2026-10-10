import SwiftUI
import SceneKit

struct RoverSceneView: UIViewRepresentable {
    let scene: SCNScene
    func makeUIView(context: Context) -> SCNView {
        let view = SCNView()
        view.backgroundColor = .clear
        view.scene = scene
        view.pointOfView = scene.rootNode.childNode(withName: "overviewCamera", recursively: true)?.clone()
        view.allowsCameraControl = true
        view.defaultCameraController.interactionMode = .orbitTurntable
        view.defaultCameraController.target = SCNVector3Zero
        view.defaultCameraController.inertiaEnabled = false
        view.defaultCameraController.minimumVerticalAngle = 5
        view.defaultCameraController.maximumVerticalAngle = 80
        view.antialiasingMode = .multisampling4X
        view.preferredFramesPerSecond = 30
        // Render on demand instead of keeping the dashboard's GPU continuously active.
        view.rendersContinuously = false
        return view
    }
    func updateUIView(_ view: SCNView, context: Context) { }
}
