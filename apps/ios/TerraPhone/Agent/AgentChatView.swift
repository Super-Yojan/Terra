import CoreImage
import SwiftUI
import UIKit

/// One message in the agent chat.
struct AgentChatMessage: Identifiable, Equatable {
    enum Role { case user, assistant }
    let id = UUID()
    var role: Role
    /// What the user typed, or the model's raw reply so far (may hold a `<think>` block).
    var text: String
    var thinking = false
    var withCamera = false
    var withRover = false
    var streaming = false
    var stopped = false
    var error: String?
    var stats: String?
}

/// The general chat on the agent page: the transcript, the turn in flight, and the options
/// for the next message. The model keeps its own history; a new chat clears both.
@MainActor
final class AgentChat: ObservableObject {
    @Published var messages: [AgentChatMessage] = []
    @Published var draft = ""
    @Published var attachCamera = false
    @Published var shareRover = false
    @Published var think = false
    @Published private(set) var running = false
    private var turn: Task<Void, Never>?
    /// The model's history may be from an earlier visit; clear it before the first message.
    private var modelHistoryFresh = false
    /// Bumped for each turn and each new chat, so a cancelled turn ending late changes nothing.
    private var generation = 0

    func send(brain: PhoneController) {
        let text = draft.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !text.isEmpty, !running else { return }
        draft = ""
        let agent = brain.taskAgent
        let image = attachCamera ? agent.chatImage() : nil
        let thinking = think
        messages.append(AgentChatMessage(role: .user, text: text, withCamera: image != nil, withRover: shareRover))
        let reply = AgentChatMessage(role: .assistant, text: "", thinking: thinking, streaming: true)
        messages.append(reply)
        let id = reply.id
        running = true
        generation += 1
        let generation = generation
        let shareRover = shareRover
        let resetFirst = !modelHistoryFresh
        modelHistoryFresh = true
        turn = Task { [weak self] in
            if resetFirst { await agent.resetChat() }
            var prompt = text
            if shareRover, let context = await Self.roverContext(brain) {
                prompt = "ROVER CONTEXT\n\(context)\n\nQUESTION\n\(text)"
            }
            let result = await agent.chat(prompt, image: image, thinking: thinking) { partial in
                Task { @MainActor in self?.update(id) { $0.text = partial } }
            }
            self?.update(id) { message in
                if let reply = result.reply {
                    message.text = reply.text
                    message.stats = Self.statsText(reply, seconds: result.seconds)
                } else if let error = result.error {
                    message.error = error
                } else {
                    message.stopped = true
                }
                message.streaming = false
            }
            self?.endTurn(generation)
        }
    }

    /// Change a reply that is still streaming; a late chunk after the end is dropped.
    private func update(_ id: UUID, _ change: (inout AgentChatMessage) -> Void) {
        guard let index = messages.firstIndex(where: { $0.id == id }), messages[index].streaming else { return }
        change(&messages[index])
    }

    func stop() {
        turn?.cancel()
    }

    /// Clear the transcript and the model's history. Waits for a cancelled reply to end first,
    /// since the model answers one thing at a time.
    func newChat(brain: PhoneController) {
        let previous = turn
        previous?.cancel()
        messages = []
        draft = ""
        modelHistoryFresh = true
        running = true
        generation += 1
        let generation = generation
        let agent = brain.taskAgent
        turn = Task { [weak self] in
            await previous?.value
            await agent.resetChat()
            self?.endTurn(generation)
        }
    }

    private func endTurn(_ generation: Int) {
        guard generation == self.generation else { return }
        running = false
        turn = nil
    }

    private static func roverContext(_ brain: PhoneController) async -> String? {
        await withCheckedContinuation { continuation in
            brain.agentChatRoverContext { continuation.resume(returning: $0) }
        }
    }

    private static func statsText(_ reply: TaskAgentReply, seconds: Double) -> String {
        var parts = [String(format: "%.1f s", seconds)]
        if let tokens = reply.generatedTokens { parts.append("\(tokens) tokens") }
        if let speed = reply.tokensPerSecond { parts.append(String(format: "%.1f tok/s", speed)) }
        return parts.joined(separator: " · ")
    }
}

/// A Claude-style chat with the on-phone model: a scrolling transcript, the reply streaming
/// in as it is written, and a composer at the bottom. The chat has no tools, so it cannot
/// drive the rover; L5 tasks and drive approval still go through ARGOS.
struct AgentChatView: View {
    @ObservedObject var brain: PhoneController
    @ObservedObject var chat: AgentChat
    let status: TaskAgentStatus?
    @FocusState private var composerFocused: Bool

    private var ready: Bool { status?.phase == .ready }
    private var canLoad: Bool {
        switch status?.phase {
        case .notLoaded, .failed: return true
        default: return false
        }
    }

    var body: some View {
        VStack(spacing: 0) {
            ScrollViewReader { proxy in
                ScrollView {
                    LazyVStack(alignment: .leading, spacing: 18) {
                        if chat.messages.isEmpty { emptyState }
                        ForEach(chat.messages) { message in
                            AgentChatRow(message: message).id(message.id)
                        }
                        Color.clear.frame(height: 1).id("bottom")
                    }
                    .padding(.horizontal, 16)
                    .padding(.top, 12)
                }
                .scrollDismissesKeyboard(.interactively)
                .onChange(of: chat.messages) {
                    withAnimation(.easeOut(duration: 0.15)) { proxy.scrollTo("bottom", anchor: .bottom) }
                }
            }
            composer
        }
        .background(Color(.systemBackground))
    }

    // MARK: Empty state

    private var emptyState: some View {
        VStack(spacing: 14) {
            Image(systemName: "sparkle")
                .font(.system(size: 34, weight: .medium))
                .foregroundStyle(Color.orange)
            Text("How can I help?")
                .font(.system(.title2, design: .serif))
            Text(ready ? "Ask \(status?.name ?? "the model") anything. It runs on this phone and cannot move the rover."
                       : "Load the model first, in the Debug tab.")
                .font(.subheadline).foregroundStyle(.secondary)
                .multilineTextAlignment(.center)
            if canLoad {
                Button("Download and load model") { brain.loadTaskAgent() }
                    .buttonStyle(.borderedProminent).tint(.orange)
            }
            if ready {
                VStack(spacing: 8) {
                    ForEach(Self.suggestions, id: \.self) { suggestion in
                        Button {
                            chat.draft = suggestion
                            composerFocused = true
                        } label: {
                            Text(suggestion)
                                .font(.subheadline)
                                .frame(maxWidth: .infinity, alignment: .leading)
                                .padding(.horizontal, 14).padding(.vertical, 10)
                                .background(RoundedRectangle(cornerRadius: 12).stroke(Color(.separator)))
                        }
                        .buttonStyle(.plain)
                    }
                }
                .padding(.top, 6)
            }
        }
        .frame(maxWidth: .infinity)
        .padding(.top, 60)
    }

    static let suggestions = [
        "What can you tell me about yourself?",
        "Explain PID control in three sentences.",
        "What do you see in the camera?",
        "Summarise the rover's state.",
    ]

    // MARK: Composer

    private var composer: some View {
        VStack(alignment: .leading, spacing: 8) {
            TextField(ready ? "Message the agent" : "Model not loaded", text: $chat.draft, axis: .vertical)
                .lineLimit(1...6)
                .focused($composerFocused)
                .disabled(!ready)
                .submitLabel(.send)
                .onSubmit { chat.send(brain: brain) }
            HStack(spacing: 6) {
                chip("camera", "Camera", on: $chat.attachCamera)
                chip("car", "Rover", on: $chat.shareRover)
                chip("brain", "Think", on: $chat.think)
                Spacer()
                if chat.running {
                    Button { chat.stop() } label: {
                        Image(systemName: "stop.fill")
                            .font(.system(size: 13, weight: .bold))
                            .frame(width: 32, height: 32)
                            .background(Circle().fill(Color.primary))
                            .foregroundStyle(Color(.systemBackground))
                    }
                    .accessibilityLabel("Stop")
                } else {
                    let empty = chat.draft.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty
                    Button { chat.send(brain: brain) } label: {
                        Image(systemName: "arrow.up")
                            .font(.system(size: 15, weight: .bold))
                            .frame(width: 32, height: 32)
                            .background(Circle().fill(empty || !ready ? Color(.systemGray4) : Color.orange))
                            .foregroundStyle(.white)
                    }
                    .disabled(empty || !ready)
                    .accessibilityLabel("Send")
                }
            }
        }
        .padding(12)
        .background(
            RoundedRectangle(cornerRadius: 22, style: .continuous)
                .fill(Color(.secondarySystemBackground))
                .shadow(color: .black.opacity(0.06), radius: 6, y: 2)
        )
        .padding(.horizontal, 12)
        .padding(.top, 6)
        .padding(.bottom, 8)
    }

    private func chip(_ symbol: String, _ title: String, on: Binding<Bool>) -> some View {
        Button { on.wrappedValue.toggle() } label: {
            Label(title, systemImage: symbol)
                .font(.caption.weight(.medium))
                .padding(.horizontal, 10).padding(.vertical, 6)
                .background(Capsule().fill(on.wrappedValue ? Color.orange.opacity(0.18) : Color.clear))
                .overlay(Capsule().stroke(on.wrappedValue ? Color.orange : Color(.separator)))
                .foregroundStyle(on.wrappedValue ? Color.orange : Color.secondary)
        }
        .buttonStyle(.plain)
        .accessibilityAddTraits(on.wrappedValue ? .isSelected : [])
    }
}

/// One transcript row: the user's message as a bubble on the right, the model's reply as
/// plain text on the left with its thinking folded away above it.
private struct AgentChatRow: View {
    let message: AgentChatMessage
    @State private var showThinking = false

    var body: some View {
        switch message.role {
        case .user: userRow
        case .assistant: assistantRow
        }
    }

    private var userRow: some View {
        VStack(alignment: .trailing, spacing: 4) {
            Text(message.text)
                .textSelection(.enabled)
                .padding(.horizontal, 14).padding(.vertical, 10)
                .background(RoundedRectangle(cornerRadius: 18, style: .continuous).fill(Color(.tertiarySystemFill)))
            if message.withCamera || message.withRover {
                Text([message.withCamera ? "camera frame" : nil, message.withRover ? "rover state" : nil]
                        .compactMap { $0 }.joined(separator: " · "))
                    .font(.caption2).foregroundStyle(.secondary)
            }
        }
        .frame(maxWidth: .infinity, alignment: .trailing)
        .padding(.leading, 40)
    }

    private var assistantRow: some View {
        let parts = AgentChatText.split(message.text, thinking: message.thinking, streaming: message.streaming)
        return VStack(alignment: .leading, spacing: 8) {
            if let thought = parts.thought, !thought.isEmpty {
                DisclosureGroup(isExpanded: $showThinking) {
                    Text(thought)
                        .font(.footnote).foregroundStyle(.secondary)
                        .textSelection(.enabled)
                        .padding(.leading, 10)
                        .overlay(Rectangle().fill(Color(.separator)).frame(width: 2), alignment: .leading)
                } label: {
                    Text(parts.answer.isEmpty && message.streaming ? "Thinking…" : "Thought process")
                        .font(.footnote).foregroundStyle(.secondary)
                }
                .tint(.secondary)
            }
            if parts.answer.isEmpty && message.streaming && parts.thought == nil {
                TypingDots()
            }
            ForEach(Array(AgentChatText.blocks(parts.answer).enumerated()), id: \.offset) { _, block in
                switch block {
                case .prose(let text):
                    Text(AgentChatText.markdown(text)).textSelection(.enabled)
                case .code(let code):
                    ScrollView(.horizontal, showsIndicators: false) {
                        Text(code).font(.footnote.monospaced()).textSelection(.enabled).padding(10)
                    }
                    .background(RoundedRectangle(cornerRadius: 10).fill(Color(.secondarySystemBackground)))
                }
            }
            if let error = message.error {
                Label(error, systemImage: "exclamationmark.triangle").font(.caption).foregroundStyle(.red)
            }
            if !message.streaming {
                HStack(spacing: 12) {
                    Button { UIPasteboard.general.string = parts.answer } label: {
                        Image(systemName: "doc.on.doc").font(.caption)
                    }
                    .buttonStyle(.plain).foregroundStyle(.secondary)
                    .accessibilityLabel("Copy reply")
                    if message.stopped { Text("Stopped").font(.caption2).foregroundStyle(.secondary) }
                    if let stats = message.stats { Text(stats).font(.caption2).foregroundStyle(.tertiary) }
                }
            }
        }
        .frame(maxWidth: .infinity, alignment: .leading)
    }
}

/// Three dots that pulse while the model has not written anything yet.
private struct TypingDots: View {
    var body: some View {
        TimelineView(.animation(minimumInterval: 0.2)) { context in
            let step = Int(context.date.timeIntervalSinceReferenceDate * 3) % 3
            HStack(spacing: 5) {
                ForEach(0..<3) { i in
                    Circle().fill(Color.orange.opacity(i == step ? 0.9 : 0.3)).frame(width: 7, height: 7)
                }
            }
        }
        .padding(.vertical, 6)
        .accessibilityLabel("Writing")
    }
}

/// Text handling for model replies: Qwen's `<think>` block, fenced code, inline Markdown.
enum AgentChatText {
    enum Block { case prose(String), code(String) }

    /// Split a raw reply into its thinking and its answer. With thinking on, the chat template
    /// may open `<think>` in the prompt, so everything before `</think>` is thought.
    static func split(_ raw: String, thinking: Bool, streaming: Bool) -> (thought: String?, answer: String) {
        if let close = raw.range(of: "</think>") {
            let thought = raw[..<close.lowerBound].replacingOccurrences(of: "<think>", with: "")
            return (trimmed(thought), trimmed(raw[close.upperBound...]))
        }
        if raw.contains("<think>") || (thinking && streaming) {
            return (trimmed(raw.replacingOccurrences(of: "<think>", with: "")), "")
        }
        return (nil, trimmed(raw))
    }

    /// Prose and fenced code blocks, in order. An unclosed fence (still streaming) is code.
    static func blocks(_ text: String) -> [Block] {
        var blocks: [Block] = []
        let pieces = text.components(separatedBy: "```")
        for (i, piece) in pieces.enumerated() {
            if i % 2 == 0 {
                let prose = trimmed(piece)
                if !prose.isEmpty { blocks.append(.prose(prose)) }
            } else {
                // Drop the language tag on the fence's first line.
                var lines = piece.components(separatedBy: "\n")
                if let first = lines.first, !first.contains(" "), lines.count > 1 { lines.removeFirst() }
                let code = lines.joined(separator: "\n").trimmingCharacters(in: .newlines)
                if !code.isEmpty { blocks.append(.code(code)) }
            }
        }
        return blocks
    }

    /// Inline Markdown (bold, italics, code, links) with line breaks kept; lists and headings
    /// stay as typed, which reads fine for a small model's replies.
    static func markdown(_ text: String) -> AttributedString {
        let options = AttributedString.MarkdownParsingOptions(interpretedSyntax: .inlineOnlyPreservingWhitespace)
        return (try? AttributedString(markdown: text, options: options)) ?? AttributedString(text)
    }

    private static func trimmed<S: StringProtocol>(_ text: S) -> String {
        text.trimmingCharacters(in: .whitespacesAndNewlines)
    }
}
