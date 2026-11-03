import Foundation
#if canImport(FoundationModels)
import FoundationModels
#endif

/// A language model the phone runs for L5 tasks. Rust writes every prompt and
/// validates every reply; the model only returns text.
protocol TaskLanguageModel: AnyObject {
    var name: String { get }
    func respond(system: String, prompt: String) async throws -> String
}

#if canImport(FoundationModels)
/// Apple's on-device model (Foundation Models, iOS 26+). Text only: the map reaches it as
/// the symbolic grid and the frontier, obstacle and landmark lists Rust builds.
@available(iOS 26.0, *)
final class AppleOnDeviceTaskModel: TaskLanguageModel {
    let name = "apple-foundation-models"
    static var available: Bool {
        if case .available = SystemLanguageModel.default.availability { return true }
        return false
    }
    func respond(system: String, prompt: String) async throws -> String {
        // A fresh session per step: the prompt already carries the task history,
        // and the on-device context window is small.
        let session = LanguageModelSession(instructions: system)
        let response = try await session.respond(to: prompt, options: GenerationOptions(temperature: 0.2))
        return response.content
    }
}
#endif

/// Takes L5 queries from Rust and answers them with the on-phone model, one at a time,
/// off the control queue. A slow model only delays the next waypoint: the rover holds
/// while Rust waits, and Rust times the query out on its own clock.
final class TaskModelRunner {
    let model: TaskLanguageModel
    private let lock = NSLock()
    private var busy = false

    init(model: TaskLanguageModel) { self.model = model }

    /// The best model this phone can run, or nil when none is available.
    static func onDevice() -> TaskModelRunner? {
        #if canImport(FoundationModels)
        if #available(iOS 26.0, *), AppleOnDeviceTaskModel.available {
            return TaskModelRunner(model: AppleOnDeviceTaskModel())
        }
        #endif
        return nil
    }

    /// Call from the control queue. Never blocks; at most one query is in flight.
    func service(_ controller: MobileController) {
        lock.lock()
        guard !busy, let query = controller.takeTaskQuery() else { lock.unlock(); return }
        busy = true
        lock.unlock()
        let model = self.model
        Task.detached(priority: .utility) { [weak self] in
            let started = ProcessInfo.processInfo.systemUptime
            do {
                let text = try await model.respond(system: query.system, prompt: query.prompt)
                _ = controller.submitTaskResponse(queryId: query.queryId, text: text, latencySeconds: ProcessInfo.processInfo.systemUptime - started)
            } catch {
                _ = controller.failTaskQuery(queryId: query.queryId, reason: String(describing: error))
            }
            self?.finish()
        }
    }

    private func finish() {
        lock.lock(); busy = false; lock.unlock()
    }
}
