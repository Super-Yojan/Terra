import CoreImage
import Foundation
#if canImport(MLXVLM) && canImport(MLXLMCommon) && canImport(MLXHuggingFace)
import HuggingFace
import MLXHuggingFace
import MLXLMCommon
import MLXVLM
import Tokenizers
#endif

/// One model answer with the generation numbers the debug page shows.
struct TaskAgentReply {
    var text: String
    var promptTokens: Int?
    var generatedTokens: Int?
    var promptSeconds: Double?
    var generateSeconds: Double?
    var tokensPerSecond: Double? {
        guard let generatedTokens, let generateSeconds, generateSeconds > 0 else { return nil }
        return Double(generatedTokens) / generateSeconds
    }
}

/// A model the phone runs as an MCP agent for L5 tasks. Rust writes every prompt, runs
/// every tool call and decides when to ask again; the model only turns a prompt into text.
protocol TaskAgentModel: AnyObject {
    var name: String { get }
    func respond(system: String, prompt: String, image: CIImage?) async throws -> TaskAgentReply
    /// One turn of the debug page's general chat. The chat keeps its own history and has
    /// no tools, so nothing said here can move the rover. `onText` gets the reply so far.
    func chat(_ message: String, image: CIImage?, thinking: Bool,
              onText: @escaping @Sendable (String) -> Void) async throws -> TaskAgentReply
    /// Forget the general chat's history.
    func resetChat() async
}

/// Instructions for the debug page's general chat.
let taskAgentChatInstructions = """
You are Terra's on-phone assistant, a small language model running locally on the iPhone \
that is the brain of a Terra rover. Answer any question the user asks, clearly and briefly. \
Use Markdown for lists and code. You cannot move the rover or call tools from this chat; \
if the user wants the rover to do something, tell them to start an L5 task from ARGOS. \
When a message starts with a ROVER CONTEXT block, it is the rover's live state; use it \
only when the question is about the rover.
"""

#if canImport(MLXVLM) && canImport(MLXLMCommon) && canImport(MLXHuggingFace)
/// Qwen 3.5 2B (4-bit MLX), a vision-language model small enough to run beside ARKit.
/// The weights are not in the app: they download from Hugging Face on first use and stay cached.
final class QwenTaskAgent: TaskAgentModel {
    static let modelId = "mlx-community/Qwen3.5-2B-4bit"
    let name = "qwen3.5-2b"
    private let container: ModelContainer
    /// Rust's tools in the chat template's own format, so Qwen sees them natively.
    private let tools: [ToolSpec]
    /// The general chat, kept between turns so it remembers the conversation.
    private var chatSession: ChatSession?
    private var chatThinking = false

    private init(container: ModelContainer) {
        self.container = container
        let json = agentToolSpecsJson().data(using: .utf8) ?? Data()
        tools = ((try? JSONSerialization.jsonObject(with: json)) as? [[String: Any]] ?? [])
            .map { $0.mapValues(Self.sendable) }
    }

    /// JSONSerialization output as the plain Swift values a `ToolSpec` holds.
    private static func sendable(_ value: Any) -> any Sendable {
        switch value {
        case let object as [String: Any]: return object.mapValues(sendable)
        case let array as [Any]: return array.map(sendable)
        case let number as NSNumber:
            if CFGetTypeID(number) == CFBooleanGetTypeID() { return number.boolValue }
            return number.doubleValue.rounded() == number.doubleValue ? number.intValue : number.doubleValue
        case let string as String: return string
        default: return String(describing: value)
        }
    }

    static func load(progress: @escaping @Sendable (Double) -> Void) async throws -> QwenTaskAgent {
        let container = try await #huggingFaceLoadModelContainer(
            configuration: ModelConfiguration(id: modelId),
            progressHandler: { progress($0.fractionCompleted) }
        )
        return QwenTaskAgent(container: container)
    }

    func respond(system: String, prompt: String, image: CIImage?) async throws -> TaskAgentReply {
        // A fresh session per prompt: Rust already puts the recent tool calls in the prompt.
        let session = ChatSession(
            container,
            instructions: system,
            generateParameters: GenerateParameters(maxTokens: 384, temperature: 0.2),
            // One short tool call per prompt; Qwen 3.5's thinking would spend the token budget.
            additionalContext: ["enable_thinking": false],
            tools: tools
        )
        // MLX parses Qwen's tool calls out of the text; write each back in the
        // `<tool_call>` JSON form Rust reads, so Rust stays the one that checks it.
        var reply = TaskAgentReply(text: "")
        for try await item in session.streamDetails(to: prompt, images: image.map { [.ciImage($0)] } ?? []) {
            switch item {
            case .chunk(let text):
                reply.text += text
            case .toolCall(let call):
                let body = (try? JSONEncoder().encode(call.function)).flatMap { String(data: $0, encoding: .utf8) } ?? "{}"
                reply.text += "<tool_call>\(body)</tool_call>"
            case .rejectedToolCall(let rejected):
                reply.text += rejected.rawTextPreview
            case .info(let info):
                reply.promptTokens = info.promptTokenCount
                reply.generatedTokens = info.generationTokenCount
                reply.promptSeconds = info.promptTime
                reply.generateSeconds = info.generateTime
            @unknown default:
                break
            }
        }
        return reply
    }

    func chat(_ message: String, image: CIImage?, thinking: Bool,
              onText: @escaping @Sendable (String) -> Void) async throws -> TaskAgentReply {
        let session: ChatSession
        if let existing = chatSession {
            session = existing
        } else {
            session = ChatSession(
                container,
                instructions: taskAgentChatInstructions,
                generateParameters: GenerateParameters(maxTokens: 1024, temperature: 0.6, topP: 0.95),
                additionalContext: ["enable_thinking": thinking]
            )
            chatSession = session
        }
        if thinking != chatThinking {
            session.additionalContext = ["enable_thinking": thinking]
            chatThinking = thinking
        }
        var reply = TaskAgentReply(text: "")
        for try await item in session.streamDetails(to: message, images: image.map { [.ciImage($0)] } ?? []) {
            try Task.checkCancellation()
            switch item {
            case .chunk(let text):
                reply.text += text
                onText(reply.text)
            case .info(let info):
                reply.promptTokens = info.promptTokenCount
                reply.generatedTokens = info.generationTokenCount
                reply.promptSeconds = info.promptTime
                reply.generateSeconds = info.generateTime
            default:
                break
            }
        }
        return reply
    }

    func resetChat() async {
        await chatSession?.clear()
        chatSession = nil
    }
}
#endif

/// Model state for the agent debug page.
struct TaskAgentStatus {
    enum Phase: Equatable {
        /// The app was built without the MLX packages.
        case unavailable
        case notLoaded
        case downloading(Double)
        case ready
        case failed(String)
    }
    var phase: Phase
    var modelId: String
    var name: String?
    var loadSeconds: Double?
    var busy: Bool
    var cameraFrameAge: Double?
    var last: TaskAgentReply?
    var lastLatency: Double?
    var lastError: String?
    var answered = 0
}

/// Runs the on-phone MCP agent: takes prompts from Rust, answers them with the model one at
/// a time off the control queue, and keeps a small copy of the camera image for `look`.
final class TaskAgentRunner {
    #if canImport(MLXVLM) && canImport(MLXLMCommon) && canImport(MLXHuggingFace)
    static let available = true
    static let modelId = QwenTaskAgent.modelId
    #else
    static let available = false
    static let modelId = "none (built without MLX)"
    #endif

    private let lock = NSLock()
    private var loaded: TaskAgentModel?
    private var loading = false
    private var progress = 0.0
    private var loadError: String?
    private var loadSeconds: Double?
    private var busy = false
    private var frame: CIImage?
    private var frameTakenAt: TimeInterval = 0
    private var frameTakenUptime: TimeInterval?
    private var last: TaskAgentReply?
    private var lastLatency: Double?
    private var lastError: String?
    private var answered = 0
    private let context = CIContext(options: [.useSoftwareRenderer: false])

    /// The model once it has loaded; nil before then or when the app was built without MLX.
    var model: TaskAgentModel? {
        lock.lock(); defer { lock.unlock() }
        return loaded
    }

    func status() -> TaskAgentStatus {
        lock.lock(); defer { lock.unlock() }
        let phase: TaskAgentStatus.Phase
        if !Self.available { phase = .unavailable }
        else if loaded != nil { phase = .ready }
        else if loading { phase = .downloading(progress) }
        else if let loadError { phase = .failed(loadError) }
        else { phase = .notLoaded }
        return TaskAgentStatus(
            phase: phase, modelId: Self.modelId, name: loaded?.name, loadSeconds: loadSeconds, busy: busy,
            cameraFrameAge: frameTakenUptime.map { ProcessInfo.processInfo.systemUptime - $0 },
            last: last, lastLatency: lastLatency, lastError: lastError, answered: answered)
    }

    /// Load the model in the background and call `ready` with its name once it can answer.
    /// A second call while loading does nothing; a failed load can be retried.
    func load(ready: @escaping (String) -> Void) {
        #if canImport(MLXVLM) && canImport(MLXLMCommon) && canImport(MLXHuggingFace)
        lock.lock()
        guard loaded == nil, !loading else { lock.unlock(); return }
        loading = true; progress = 0; loadError = nil
        lock.unlock()
        let started = ProcessInfo.processInfo.systemUptime
        Task.detached(priority: .utility) { [weak self] in
            do {
                let model = try await QwenTaskAgent.load { fraction in
                    self?.lock.lock(); self?.progress = fraction; self?.lock.unlock()
                }
                self?.lock.lock()
                self?.loaded = model; self?.loading = false
                self?.loadSeconds = ProcessInfo.processInfo.systemUptime - started
                self?.lock.unlock()
                ready(model.name)
            } catch {
                self?.lock.lock(); self?.loading = false; self?.loadError = String(describing: error); self?.lock.unlock()
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
        lock.lock()
        frame = CIImage(cgImage: copy); frameTakenAt = timestamp
        frameTakenUptime = ProcessInfo.processInfo.systemUptime
        lock.unlock()
    }

    /// The image `look` would attach now, for the debug page.
    func lookImage() -> CGImage? {
        lock.lock(); let frame = frame; lock.unlock()
        return frame.flatMap { context.createCGImage($0, from: $0.extent) }
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
                let reply = try await model.respond(system: prompt.system, prompt: prompt.prompt, image: image)
                let latency = ProcessInfo.processInfo.systemUptime - started
                _ = controller.submitAgentReply(promptId: prompt.promptId, reply: reply.text, latencySeconds: latency)
                self?.finish(reply: reply, latency: latency, error: nil)
            } catch {
                _ = controller.failAgentPrompt(promptId: prompt.promptId, reason: String(describing: error))
                self?.finish(reply: nil, latency: nil, error: String(describing: error))
            }
        }
    }

    /// Run one prompt with the agent's real instructions and tools, without a rover and
    /// without running the tool. Returns the raw reply and what Rust would parse from it.
    func test(prompt: String, withCamera: Bool) async -> (reply: TaskAgentReply?, parsed: String, seconds: Double, error: String?) {
        lock.lock()
        guard let model = loaded, !busy else {
            let why = loaded == nil ? "The model has not loaded yet." : "The model is answering a rover prompt; try again in a moment."
            lock.unlock()
            return (nil, "", 0, why)
        }
        busy = true
        let image = withCamera ? frame : nil
        lock.unlock()
        let started = ProcessInfo.processInfo.systemUptime
        do {
            let reply = try await model.respond(system: agentSystemPromptText(), prompt: prompt, image: image)
            let seconds = ProcessInfo.processInfo.systemUptime - started
            finish(reply: reply, latency: seconds, error: nil)
            return (reply, agentParseReply(reply: reply.text), seconds, nil)
        } catch {
            finish(reply: nil, latency: nil, error: String(describing: error))
            return (nil, "", ProcessInfo.processInfo.systemUptime - started, String(describing: error))
        }
    }

    /// One turn of the general chat on the debug page. Streams the reply into `onText`.
    /// Rover prompts wait while a chat reply is being written, so keep replies short
    /// during a task; cancelling the calling task stops the reply.
    func chat(_ message: String, image: CIImage?, thinking: Bool,
              onText: @escaping @Sendable (String) -> Void) async -> (reply: TaskAgentReply?, seconds: Double, error: String?) {
        lock.lock()
        guard let model = loaded, !busy else {
            let why = loaded == nil ? "The model has not loaded yet." : "The model is answering a rover prompt; try again in a moment."
            lock.unlock()
            return (nil, 0, why)
        }
        busy = true
        lock.unlock()
        let started = ProcessInfo.processInfo.systemUptime
        do {
            let reply = try await model.chat(message, image: image, thinking: thinking, onText: onText)
            let seconds = ProcessInfo.processInfo.systemUptime - started
            finish(reply: reply, latency: seconds, error: nil)
            return (reply, seconds, nil)
        } catch is CancellationError {
            finish(reply: nil, latency: nil, error: nil)
            return (nil, ProcessInfo.processInfo.systemUptime - started, nil)
        } catch {
            finish(reply: nil, latency: nil, error: String(describing: error))
            return (nil, ProcessInfo.processInfo.systemUptime - started, String(describing: error))
        }
    }

    /// The camera frame `look` would attach now, for a chat message.
    func chatImage() -> CIImage? {
        lock.lock(); defer { lock.unlock() }
        return frame
    }

    /// Forget the general chat's history.
    func resetChat() async {
        lock.lock(); let model = loaded; lock.unlock()
        await model?.resetChat()
    }

    private func finish(reply: TaskAgentReply?, latency: Double?, error: String?) {
        lock.lock()
        busy = false
        if let reply { last = reply; lastLatency = latency; answered += 1 }
        lastError = error
        lock.unlock()
    }
}
