import OSLog
import ARKit
import AVFoundation
import Combine
import CoreMotion
import CoreLocation
import Foundation
import QuartzCore
import simd

// On-device task agent: model loading, debug output and rover context for chat.
extension PhoneController {
    /// Download (first time) and load the on-phone agent model. In phone mode the rover
    /// then offers it to ARGOS as the MCP agent.
    func loadTaskAgent() {
        taskAgent.load { [weak self] name in
            self?.controlQueue.async {
                guard let self, self.mode == .phone else { return }
                try? self.controller?.setTaskAgent(name: name, inProcess: true)
            }
        }
    }
    /// Rust's agent trace (status, prompts, replies, tool calls), delivered on the main queue.
    func agentDebugJSON(_ deliver: @escaping (String) -> Void) {
        controlQueue.async {
            let json = self.controller?.agentDebugJson() ?? ""
            DispatchQueue.main.async { deliver(json) }
        }
    }
    func clearAgentDebug() { controlQueue.async { self.controller?.clearAgentDebug() } }
    /// The rover's live state for the agent chat, read through Rust's read-only MCP tools
    /// (get_state, get_task) so the chat sees what an agent would. Delivered on the main queue.
    func agentChatRoverContext(_ deliver: @escaping (String?) -> Void) {
        controlQueue.async {
            guard let controller = self.controller else {
                DispatchQueue.main.async { deliver(nil) }
                return
            }
            // mcp_handle must run off the control queue.
            DispatchQueue.global(qos: .userInitiated).async {
                func call(_ tool: String) -> String? {
                    let request = #"{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"\#(tool)","arguments":{}}}"#
                    guard let data = controller.mcpHandle(session: "debug-chat", request: request).data(using: .utf8),
                          let response = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
                          let result = response["result"] as? [String: Any],
                          let content = result["content"] as? [[String: Any]] else { return nil }
                    return content.compactMap { $0["text"] as? String }.joined(separator: "\n")
                }
                let parts = ["get_state", "get_task"].compactMap { tool in call(tool).map { "\(tool): \($0)" } }
                DispatchQueue.main.async { deliver(parts.isEmpty ? nil : parts.joined(separator: "\n")) }
            }
        }
    }
}
