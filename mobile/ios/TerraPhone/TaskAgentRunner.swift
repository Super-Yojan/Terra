import CoreImage
import Foundation
#if canImport(MLXVLM) && canImport(MLXLMCommon) && canImport(MLXHuggingFace)
import HuggingFace
import MLXHuggingFace
import MLXLMCommon
import MLXVLM
import Tokenizers
#endif

/// A model the phone runs as an MCP agent for L5 tasks. Rust writes every prompt, runs
/// every tool call and decides when to ask again; the model only turns a prompt into text.
protocol TaskAgentModel: AnyObject {
    var name: String { get }
    func respond(system: String, prompt: String, image: CIImage?) async throws -> String
}

#if canImport(MLXVLM) && canImport(MLXLMCommon) && canImport(MLXHuggingFace)
/// Qwen 3.5 2B (4-bit MLX), a vision-language model small enough to run beside ARKit.
/// The weights (about 1.5 GB) download from Hugging Face on first use and stay cached.
final class QwenTaskAgent: TaskAgentModel {
    static let modelId = "mlx-community/Qwen3.5-2B-MLX-4bit"
    let name = "qwen3.5-2b"
    private let container: ModelContainer

    private init(container: ModelContainer) { self.container = container }

    static func load() async throws -> QwenTaskAgent {
        let container = try await #huggingFaceLoadModelContainer(
            configuration: ModelConfiguration(id: modelId)
        )
        return QwenTaskAgent(container: container)
    }

    func respond(system: String, prompt: String, image: CIImage?) async throws -> String {
        // A fresh session per prompt: Rust already puts the recent tool calls in the prompt.
        let session = ChatSession(
            container,
            instructions: system,
            generateParameters: GenerateParameters(maxTokens: 384, temperature: 0.2)
        )
        if let image {
            return try await session.respond(to: prompt, image: .ciImage(image))
        }
        return try await session.respond(to: prompt)
    }
}
#endif

/// Runs the on-phone MCP agent: takes prompts from Rust, answers them with the model one at
/// a time off the control queue, and keeps a small copy of the camera image for `look`.
final class TaskAgentRunner {
    private let lock = NSLock()
    private var loaded: TaskAgentModel?
    private var loading = false
    private var busy = false
    private var frame: CIImage?
    private var frameTakenAt: TimeInterval = 0
    private let context = CIContext(options: [.useSoftwareRenderer: false])

    /// The model once it has loaded; nil before then or when the app was built without MLX.
    var model: TaskAgentModel? {
        lock.lock(); defer { lock.unlock() }
        return loaded
    }

    /// Load the model in the background and call `ready` with its name once it can answer.
    /// A second call while loading does nothing; a failed load can be retried.
    func load(ready: @escaping (String) -> Void) {
        #if canImport(MLXVLM) && canImport(MLXLMCommon) && canImport(MLXHuggingFace)
        lock.lock()
        guard loaded == nil, !loading else { lock.unlock(); return }
        loading = true
        lock.unlock()
        Task.detached(priority: .utility) { [weak self] in
            do {
                let model = try await QwenTaskAgent.load()
                self?.lock.lock(); self?.loaded = model; self?.loading = false; self?.lock.unlock()
                ready(model.name)
            } catch {
                self?.lock.lock(); self?.loading = false; self?.lock.unlock()
                TerraLog.tracking.error("Qwen task agent failed to load: \(String(describing: error), privacy: .public)")
            }
        }
        #endif
    }

    /// Keep a 640 px copy of the camera image about once a second. Copying releases the
    /// ARKit buffer at once, so the camera never runs out of frames.
    func offer(pixelBuffer: CVPixelBuffer, timestamp: TimeInterval) {
        lock.lock(); let due = loaded != nil && timestamp - frameTakenAt >= 1; lock.unlock()
        guard due else { return }
        let image = CIImage(cvPixelBuffer: pixelBuffer).oriented(.right)
        let scale = 640 / max(image.extent.width, image.extent.height)
        let small = image.transformed(by: CGAffineTransform(scaleX: scale, y: scale))
        guard let copy = context.createCGImage(small, from: small.extent) else { return }
        lock.lock(); frame = CIImage(cgImage: copy); frameTakenAt = timestamp; lock.unlock()
    }

    /// Call from the control queue. Never blocks; at most one prompt is in flight.
    func service(_ controller: MobileController) {
        lock.lock()
        guard !busy, let model = loaded, let prompt = controller.takeAgentPrompt() else { lock.unlock(); return }
        busy = true
        let image = prompt.attachImage ? frame : nil
        lock.unlock()
        Task.detached(priority: .utility) { [weak self] in
            let started = ProcessInfo.processInfo.systemUptime
            do {
                let text = try await model.respond(system: prompt.system, prompt: prompt.prompt, image: image)
                _ = controller.submitAgentReply(promptId: prompt.promptId, reply: text, latencySeconds: ProcessInfo.processInfo.systemUptime - started)
            } catch {
                _ = controller.failAgentPrompt(promptId: prompt.promptId, reason: String(describing: error))
            }
            self?.finish()
        }
    }

    private func finish() {
        lock.lock(); busy = false; lock.unlock()
    }
}
