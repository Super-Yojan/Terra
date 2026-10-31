import os
import SwiftUI
import UIKit

/// The on-phone L5 agent (Qwen 3.5 2B over the rover's MCP tools). The Chat tab is a general
/// chat with the model (no tools). The Debug tab shows model download and load,
/// speed and memory, why it is or is not being prompted, each prompt with the raw reply and
/// the tool call Rust ran, and every tool call from any session. A test prompt runs the
/// model with its real instructions and tools without a rover and without running the tool.
struct AgentDebugView: View {
    @ObservedObject var brain: PhoneController
    @State private var model: TaskAgentStatus?
    @State private var trace: [String: Any] = [:]
    @State private var memory = PhoneMemory.read()
    @State private var lookImage: CGImage?
    @State private var testPrompt = AgentDebugView.samplePrompt
    @State private var testCamera = false
    @State private var testRunning = false
    @State private var testResult: (reply: TaskAgentReply?, parsed: String, seconds: Double, error: String?)?
    @State private var tab = Tab.chat
    @StateObject private var chat = AgentChat()

    enum Tab: String, CaseIterable { case chat = "Chat", debug = "Debug" }

    var body: some View {
        VStack(spacing: 0) {
            Picker("View", selection: $tab) {
                ForEach(Tab.allCases, id: \.self) { Text($0.rawValue).tag($0) }
            }
            .pickerStyle(.segmented)
            .padding(.horizontal, 16)
            .padding(.vertical, 8)
            switch tab {
            case .chat:
                AgentChatView(brain: brain, chat: chat, status: model)
            case .debug:
                Form {
                    modelSection
                    agentSection
                    taskSection
                    testSection
                    exchangesSection
                    callsSection
                    cameraSection
                }
            }
        }
        .navigationTitle("Agent")
        .navigationBarTitleDisplayMode(.inline)
        .toolbar {
            if tab == .chat {
                ToolbarItem(placement: .topBarTrailing) {
                    Button { chat.newChat(brain: brain) } label: { Image(systemName: "square.and.pencil") }
                        .disabled(chat.messages.isEmpty && !chat.running)
                        .accessibilityLabel("New chat")
                }
            }
        }
        .task {
            var ticks = 0
            while !Task.isCancelled {
                model = brain.taskAgent.status()
                memory = PhoneMemory.read()
                if ticks % 2 == 0 {
                    brain.agentDebugJSON { json in
                        trace = (json.data(using: .utf8).flatMap { try? JSONSerialization.jsonObject(with: $0) } as? [String: Any]) ?? [:]
                    }
                }
                if ticks % 4 == 0 { lookImage = brain.taskAgent.lookImage() }
                ticks += 1
                try? await Task.sleep(nanoseconds: 500_000_000)
            }
        }
    }

    // MARK: Model

    private var modelSection: some View {
        Section {
            LabeledContent("State", value: phaseText)
            if case .downloading(let fraction) = model?.phase {
                ProgressView(value: fraction)
                Text(fraction < 0.999 ? String(format: "Downloading %.0f %%", fraction * 100) : "Loading weights into memory")
                    .font(.caption).foregroundStyle(.secondary)
            }
            LabeledContent("Model", value: model?.modelId ?? TaskAgentRunner.modelId).font(.footnote)
            if let seconds = model?.loadSeconds { LabeledContent("Load time", value: String(format: "%.1f s", seconds)) }
            LabeledContent("App memory", value: memory.footprintText)
            LabeledContent("Still available", value: memory.availableText)
            if let last = model?.last {
                LabeledContent("Last answer", value: answerText(last, latency: model?.lastLatency))
            }
            if let error = model?.lastError {
                Text(error).font(.caption).foregroundStyle(.red)
            }
            if canLoad {
                Button(model?.phase == .notLoaded ? "Download and load model" : "Retry load") { brain.loadTaskAgent() }
            }
        } header: {
            Text("Model")
        } footer: {
            Text("The model is not shipped in the app. The first load downloads about 1.5 GB from Hugging Face and caches it on the phone; later loads read the cache. In phone mode the rover offers the loaded agent to ARGOS as \"MCP agent\".")
        }
    }

    private var canLoad: Bool {
        switch model?.phase {
        case .notLoaded, .failed: return true
        default: return false
        }
    }

    private var phaseText: String {
        switch model?.phase {
        case .unavailable: return "Not in this build (no MLX packages)"
        case .notLoaded: return "Not loaded"
        case .downloading: return "Loading"
        case .ready: return model?.busy == true ? "Ready · answering" : "Ready"
        case .failed(let why): return "Failed: \(why)"
        case nil: return "–"
        }
    }

    // MARK: Agent and task (from Rust)

    private var agentSection: some View {
        Section {
            LabeledContent("Now", value: trace["status"] as? String ?? "–")
            LabeledContent("Advertised as", value: trace["agent"] as? String ?? "none")
            LabeledContent("Runs on this phone", value: (trace["in_process"] as? Bool) == true ? "yes" : "no")
            if let controller = trace["controller"] as? [String: Any] {
                LabeledContent("Controls task", value: "\(controller["task_id"] as? String ?? "?") · \(controller["session"] as? String ?? "?")")
            }
            LabeledContent("Replies without a tool call", value: "\(intValue(trace["bad_replies"])) of \(intValue(trace["bad_reply_limit"]))")
            LabeledContent("Prompts answered", value: "\(model?.answered ?? 0)")
        } header: {
            Text("Agent")
        } footer: {
            Text("Rust decides when to prompt. It does not prompt while the rover drives a plan, waits for the operator's answer, or waits for drive approval.")
        }
    }

    @ViewBuilder private var taskSection: some View {
        Section("Task") {
            if let task = trace["task"] as? [String: Any] {
                LabeledContent("Task", value: task["instruction"] as? String ?? "–")
                LabeledContent("Phase", value: "\(task["phase"] as? String ?? "–") · \(task["reason"] as? String ?? "")")
                LabeledContent("Run by", value: (task["agent"] as? String) == "mcp" ? "MCP agent" : "Built-in model")
                LabeledContent("Steps", value: "\(intValue(task["step"])) of \(intValue(task["max_steps"]))")
                if let remaining = task["remaining_seconds"] as? Double {
                    LabeledContent("Time left", value: String(format: "%.0f s", remaining))
                }
                if let question = task["question"] as? String {
                    LabeledContent("Asked operator", value: question)
                }
                if let drive = task["drive_request"] as? [String: Any] {
                    LabeledContent("Drive request", value: String(format: "%.0f s · %@", drive["seconds"] as? Double ?? 0, drive["reason"] as? String ?? ""))
                }
                if let left = task["drive_remaining_seconds"] as? Double {
                    LabeledContent("Raw drive", value: String(format: "approved · %.0f s left", left))
                }
                if let plan = task["plan"] as? [[String: Any]], !plan.isEmpty {
                    LabeledContent("Plan", value: plan.map { "\($0["label"] as? String ?? "?") (\(String(format: "%.1f, %.1f", $0["x"] as? Double ?? 0, $0["y"] as? Double ?? 0)))" }.joined(separator: " → "))
                }
            } else {
                Text("No task yet. Start one in ARGOS: Task · L5, pick MCP agent.").foregroundStyle(.secondary)
            }
        }
    }

    // MARK: Test prompt

    private var testSection: some View {
        Section {
            TextEditor(text: $testPrompt)
                .font(.footnote.monospaced())
                .frame(minHeight: 140)
            Toggle("Attach camera frame", isOn: $testCamera)
            HStack {
                Button(testRunning ? "Running…" : "Run test prompt") {
                    testRunning = true
                    let prompt = testPrompt, camera = testCamera, agent = brain.taskAgent
                    Task {
                        let result = await agent.test(prompt: prompt, withCamera: camera)
                        testResult = result
                        testRunning = false
                    }
                }
                .disabled(testRunning || model?.phase != .ready)
                Spacer()
                Button("Reset") { testPrompt = AgentDebugView.samplePrompt; testResult = nil }
            }
            if let result = testResult {
                if let error = result.error { Text(error).font(.caption).foregroundStyle(.red) }
                if let reply = result.reply {
                    LabeledContent("Answer", value: answerText(reply, latency: result.seconds))
                    textBlock("Raw reply", reply.text)
                    textBlock("Rust would run", parsedText(result.parsed))
                }
            }
        } header: {
            Text("Test prompt")
        } footer: {
            Text("Uses the agent's real instructions and tool list. Nothing is sent to the rover and no tool runs, so this works with no rover and no task.")
        }
    }

    // MARK: History

    private var exchangesSection: some View {
        Section {
            let exchanges = (trace["exchanges"] as? [[String: Any]] ?? []).reversed()
            if exchanges.isEmpty { Text("No prompts yet").foregroundStyle(.secondary) }
            ForEach(Array(exchanges.enumerated()), id: \.offset) { _, exchange in
                DisclosureGroup {
                    textBlock("Prompt", exchange["prompt"] as? String ?? "")
                    if let reply = exchange["reply"] as? String { textBlock("Raw reply", reply) }
                    if let result = exchange["result"] as? String { textBlock("Tool result", result) }
                    if let error = exchange["error"] as? String { Text(error).font(.caption).foregroundStyle(.red) }
                } label: {
                    exchangeLabel(exchange)
                }
            }
            Button("Clear history", role: .destructive) { brain.clearAgentDebug() }
        } header: {
            Text("Prompts and replies")
        } footer: {
            Text("Newest first, last 30. Rust parses the tool call from the raw reply and runs it with the same checks as any MCP client.")
        }
    }

    private func exchangeLabel(_ exchange: [String: Any]) -> some View {
        let id = intValue(exchange["prompt_id"])
        let tool = exchange["tool"] as? String
        let ok = exchange["ok"] as? Bool
        let failed = exchange["error"] != nil || ok == false
        let pending = exchange["reply"] == nil && exchange["error"] == nil
        let latency = (exchange["latency_seconds"] as? Double).map { String(format: " · %.1f s", $0) } ?? ""
        let camera = (exchange["attach_image"] as? Bool) == true ? " · camera" : ""
        return HStack {
            Image(systemName: pending ? "hourglass" : failed ? "xmark.octagon" : "checkmark.circle")
                .foregroundStyle(pending ? Color.secondary : failed ? Color.red : Color.green)
            VStack(alignment: .leading) {
                Text("#\(id) \(tool ?? (pending ? "answering…" : "no tool call"))")
                Text("\(timeText(exchange["asked_at"]))\(latency)\(camera)").font(.caption).foregroundStyle(.secondary)
            }
        }
    }

    private var callsSection: some View {
        Section {
            let calls = (trace["calls"] as? [[String: Any]] ?? []).reversed()
            if calls.isEmpty { Text("No tool calls yet").foregroundStyle(.secondary) }
            ForEach(Array(calls.enumerated()), id: \.offset) { _, call in
                VStack(alignment: .leading, spacing: 2) {
                    HStack {
                        Image(systemName: (call["ok"] as? Bool) == true ? "checkmark.circle" : "xmark.octagon")
                            .foregroundStyle((call["ok"] as? Bool) == true ? Color.green : Color.red)
                        Text(call["tool"] as? String ?? "?").bold()
                        Text(jsonText(call["arguments"])).font(.caption.monospaced()).lineLimit(1)
                        Spacer()
                        Text(call["session"] as? String ?? "").font(.caption).foregroundStyle(.secondary)
                    }
                    Text(call["result"] as? String ?? "").font(.caption).lineLimit(3).foregroundStyle(.secondary)
                    Text(timeText(call["at"])).font(.caption2).foregroundStyle(.tertiary)
                }
            }
        } header: {
            Text("Tool calls · all sessions")
        } footer: {
            Text("Newest first, last 30. \"in-process\" is the on-phone agent; anything else is another MCP client.")
        }
    }

    private var cameraSection: some View {
        Section {
            if let lookImage {
                Image(decorative: lookImage, scale: 1).resizable().scaledToFit()
                if let age = model?.cameraFrameAge { Text(String(format: "%.0f s old", age)).font(.caption).foregroundStyle(.secondary) }
            } else {
                Text("No frame yet. The agent keeps one about once a second in phone mode once the model has loaded.")
                    .foregroundStyle(.secondary)
            }
            Button("Start phone sensors") { brain.startPhone() }
        } header: {
            Text("Camera frame for look")
        } footer: {
            Text("This is what the model sees when it calls look. No rover is needed: phone mode runs ARKit, and the motors stay off without a Bluetooth link.")
        }
    }

    // MARK: Formatting

    private func answerText(_ reply: TaskAgentReply, latency: Double?) -> String {
        var parts: [String] = []
        if let latency { parts.append(String(format: "%.1f s", latency)) }
        if let tokens = reply.generatedTokens { parts.append("\(tokens) tokens out") }
        if let tokens = reply.promptTokens { parts.append("\(tokens) in") }
        if let speed = reply.tokensPerSecond { parts.append(String(format: "%.1f tok/s", speed)) }
        return parts.isEmpty ? "–" : parts.joined(separator: " · ")
    }

    private func parsedText(_ json: String) -> String {
        guard let data = json.data(using: .utf8), let value = try? JSONSerialization.jsonObject(with: data) as? [String: Any] else { return json }
        if (value["ok"] as? Bool) == true {
            return "\(value["tool"] as? String ?? "?") \(jsonText(value["arguments"]))"
        }
        return "Nothing: \(value["error"] as? String ?? "?")"
    }

    private func textBlock(_ title: String, _ body: String) -> some View {
        VStack(alignment: .leading, spacing: 4) {
            Text(title).font(.caption.bold()).foregroundStyle(.secondary)
            Text(body).font(.caption.monospaced()).textSelection(.enabled)
        }
    }

    private func jsonText(_ value: Any?) -> String {
        guard let value, JSONSerialization.isValidJSONObject(value),
              let data = try? JSONSerialization.data(withJSONObject: value, options: [.sortedKeys]) else { return "{}" }
        return String(data: data, encoding: .utf8) ?? "{}"
    }

    private func timeText(_ value: Any?) -> String {
        guard let seconds = value as? Double else { return "" }
        return Date(timeIntervalSince1970: seconds).formatted(date: .omitted, time: .standard)
    }

    private func intValue(_ value: Any?) -> Int {
        (value as? NSNumber)?.intValue ?? 0
    }

    static let samplePrompt = """
    TASK: Find the blue bucket and stop next to it.
    STEP 1 of 20 · 0 of 300 s used.
    ROVER: x 0.0, y 0.0, heading 0°.
    MAP: F1 frontier 2.1 m ahead (bearing 0°). F2 frontier 3.4 m at bearing +80°. O1 obstacle 1.2 m at bearing -30°. L1 blue bucket 3 at x 2.0, y 1.0.
    STATE: planning (awaiting_agent).

    Call exactly one tool.
    """
}

/// App memory as iOS counts it against the app's limit, and how much more it may use.
struct PhoneMemory {
    var footprint: UInt64?
    var available: UInt64

    static func read() -> PhoneMemory {
        var info = task_vm_info_data_t()
        var count = mach_msg_type_number_t(MemoryLayout<task_vm_info_data_t>.size / MemoryLayout<natural_t>.size)
        let result = withUnsafeMutablePointer(to: &info) {
            $0.withMemoryRebound(to: integer_t.self, capacity: Int(count)) {
                task_info(mach_task_self_, task_flavor_t(TASK_VM_INFO), $0, &count)
            }
        }
        return PhoneMemory(footprint: result == KERN_SUCCESS ? info.phys_footprint : nil,
                           available: UInt64(os_proc_available_memory()))
    }
    var footprintText: String { footprint.map(Self.megabytes) ?? "–" }
    var availableText: String { Self.megabytes(available) }
    private static func megabytes(_ bytes: UInt64) -> String {
        String(format: "%.0f MB", Double(bytes) / 1_048_576)
    }
}
