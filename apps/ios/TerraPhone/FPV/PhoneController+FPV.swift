import OSLog
import ARKit
import AVFoundation
import Combine
import CoreMotion
import CoreLocation
import Foundation
import QuartzCore
import simd

// FPV live video over WebRTC and the shared-camera handoff with ARKit.
extension PhoneController {
    /// Stop the live view. The fleet session, arming gate, and velocity commands stay as they are.
    func stopFpv() {
        controlQueue.async {
            self.fpvFailure = nil
            self.closeFpv(publish: true)
        }
    }
    private func fpvNow() -> UInt64 {
        UInt64(ProcessInfo.processInfo.systemUptime * 1000)
    }
    func drainFpvSignaling() {
        guard let controller else { return }
        let now = fpvNow()
        if controller.dashboardStatus() != "connected" {
            if controller.fpvHasSession() || fpvRunning {
                fpvFailure = nil
                closeFpv(publish: false)
            }
            return
        }
        for signal in controller.takeWebrtcSignaling() {
            applyFpv(controller.fpvReceive(kind: signal.kind, payload: signal.payload, nowMs: now))
        }
        applyFpv(controller.fpvTick(nowMs: now))
    }
    /// Apply video-signaling effects from `fpv-signaling`. None of these arm or drive.
    private func applyFpv(_ effects: [FpvEffect]) {
        for effect in effects {
            switch effect.kind {
            case "publish":
                publishFpv(effect.topic, effect.payload)
            case "create_offer":
                fpvFailure = nil
                fpvRunning = true
                let settings = FpvIceStore.load()
                TerraLog.fpv.notice("FPV connecting; \(settings.redactedSummary(), privacy: .public)")
                fpvVideo.start(session: effect.session, settings: settings, sharedCamera: readSharedCamera())
            case "apply_answer":
                fpvVideo.applyAnswer(session: effect.session, sdp: effect.sdp)
            case "add_ice":
                fpvVideo.addIce(session: effect.session, candidate: effect.candidate, sdpMid: effect.sdpMid, sdpMLineIndex: Int(effect.sdpMLineIndex))
            case "end_of_candidates":
                break
            case "stop_media":
                fpvRunning = false
                fpvVideo.stopMedia()
            case "show":
                showFpv(effect.state, detail: effect.detail)
            default:
                break
            }
        }
    }
    func handleFpvEvent(_ event: FpvVideoEvent) {
        guard let controller else { return }
        let now = fpvNow()
        switch event {
        case .offer(let session, let sdp):
            applyFpv(controller.fpvNoteOffer(session: session, sdp: sdp, nowMs: now))
        case .ice(let session, let candidate, let sdpMid, let sdpMLineIndex):
            applyFpv(controller.fpvNoteIce(session: session, candidate: candidate, sdpMid: sdpMid, sdpMLineIndex: Int32(sdpMLineIndex), nowMs: now))
        case .gatheringComplete(let session):
            applyFpv(controller.fpvNoteIce(session: session, candidate: "", sdpMid: "", sdpMLineIndex: 0, nowMs: now))
        case .state(let session, let state):
            if state == "failed" { fpvFailure = fpvFailure ?? "The video connection failed" }
            applyFpv(controller.fpvNoteMedia(session: session, state: state, nowMs: now))
        case .cameraUnavailable(let session, let reason):
            fpvFailure = reason
            applyFpv(controller.fpvNoteMedia(session: session, state: "failed", nowMs: now))
        }
    }
    func closeFpv(publish: Bool, clearFailure: Bool = true) {
        if clearFailure { fpvFailure = nil }
        if let controller {
            applyFpv(controller.fpvStop(publish: publish))
        } else {
            fpvRunning = false
            fpvVideo.stopMedia()
            if fpvFailure == nil { showFpvIdle() }
        }
    }
    private func showFpv(_ state: String, detail: String) {
        switch state {
        case "connecting":
            DispatchQueue.main.async {
                guard !self.fpvLive else { return }
                self.fpvStatus = "Connecting"
                self.fpvLive = false
                self.fpvActive = true
            }
        case "connected":
            DispatchQueue.main.async {
                self.fpvStatus = "Live · streaming"
                self.fpvLive = true
                self.fpvActive = true
            }
        case "failed":
            let reason = fpvFailure ?? Self.fpvFailureText(detail)
            fpvFailure = reason
            TerraLog.fpv.error("FPV failed: \(reason, privacy: .public)")
            DispatchQueue.main.async {
                self.fpvStatus = "Failed · \(reason)"
                self.fpvLive = false
                self.fpvActive = true
            }
        case "closed":
            if fpvFailure == nil { showFpvIdle() }
        default:
            break
        }
    }
    private static func fpvFailureText(_ detail: String) -> String {
        switch detail {
        case "timed_out": return "The video timed out"
        case "offer_too_large": return "The video offer is too large to send"
        case "answer_too_large": return "The video answer is too large to send"
        case "empty_sdp": return "The video description was empty"
        default: return "The video connection failed"
        }
    }
    private func showFpvIdle() {
        DispatchQueue.main.async {
            self.fpvStatus = "Idle"
            self.fpvLive = false
            self.fpvActive = false
        }
    }
    private func publishFpv(_ kind: String, _ payload: String) {
        guard let controller else { return }
        do { try controller.publishWebrtcSignaling(kind: kind, payload: payload) }
        catch { TerraLog.fpv.error("FPV signaling was not published") }
    }
    func updateSharedCamera(_ value: Bool) {
        setSharedCameraFlag(value)
        controlQueue.async {
            if self.fpvRunning { self.fpvVideo.setSharedCamera(value) }
        }
    }
    func setSharedCameraFlag(_ value: Bool) {
        fpvCameraLock.lock()
        sharedCameraForFpv = value
        fpvCameraLock.unlock()
    }
    func readSharedCamera() -> Bool {
        fpvCameraLock.lock()
        defer { fpvCameraLock.unlock() }
        return sharedCameraForFpv
    }
}
