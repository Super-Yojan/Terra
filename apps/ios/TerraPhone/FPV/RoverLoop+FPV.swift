import CoreVideo
import Foundation

/// Rear-camera WebRTC FPV signaling and video bridge for `RoverLoop`.
///
/// The Rust `fpv-signaling` state machine (inside `TerraAv`) owns session
/// IDs, timeouts, and Zenoh topic formatting. `FpvVideoCall` owns the
/// `RTCPeerConnection` and H.264 encoder. This extension glues them together
/// on `plannerQueue` so drive control and FPV video stay completely decoupled.
extension RoverLoop {
    /// Save STUN/TURN settings for future FPV sessions.
    func updateFpvIce(_ settings: FpvIceSettings) {
        _ = FpvIceStore.save(
            stunURL: settings.stunURL,
            turnURL: settings.turnURL,
            turnUsername: settings.turnUsername,
            turnCredential: settings.turnCredential
        )
        let saved = FpvIceStore.load()
        DispatchQueue.main.async { self.fpvIce = saved }
        plannerQueue.async { self.activeFpvIce = saved }
    }

    /// Forward an ARKit rear-camera frame into the active WebRTC call (if any).
    /// Called on `terra.sensors.ar` at ~60 Hz; returns immediately when idle.
    func sendFpvFrame(pixelBuffer: CVPixelBuffer, timestampSeconds: TimeInterval) {
        fpvLock.lock()
        let call = fpvActiveCall
        fpvLock.unlock()
        call?.consume(pixelBuffer: pixelBuffer, timestamp: timestampSeconds)
    }

    /// Stop the active FPV session (if any) and notify the dashboard.
    /// Does not affect drive commands or arming.
    func stopFpv() {
        plannerQueue.async {
            self.stopFpvLocked(publish: true)
        }
    }

    /// Called on `plannerQueue` at 20 Hz from `plannerTick()`.
    /// Drains inbound Zenoh signaling messages, advances timeouts, and applies effects.
    func pollFpvLocked() {
        let now = fpvNowMs()
        if !av.baseConnected() {
            if av.fpvHasSession() {
                let effects = av.fpvStop(publish: false)
                applyFpvEffectsLocked(effects)
            }
            return
        }

        let signals = av.takeFpvSignals()
        for signal in signals {
            let effects = av.fpvReceive(kind: signal.kind, payload: signal.payload, nowMs: now)
            applyFpvEffectsLocked(effects)
        }

        if av.fpvHasSession() {
            let effects = av.fpvTick(nowMs: now)
            applyFpvEffectsLocked(effects)
        }
    }

    /// Must be called on `plannerQueue`.
    func stopFpvLocked(publish: Bool) {
        let effects = av.fpvStop(publish: publish)
        applyFpvEffectsLocked(effects)
        clearFpvCallLocked()
        updateFpvUi(state: "idle", detail: "")
    }

    private func ensureFpvCallLocked() -> FpvVideoCall {
        if let call = fpvCall {
            return call
        }
        let call = FpvVideoCall()
        call.onEvent = { [weak self] event in
            self?.plannerQueue.async {
                self?.handleFpvVideoEventLocked(event)
            }
        }
        fpvCall = call
        return call
    }

    private func clearFpvCallLocked() {
        fpvLock.lock()
        fpvActiveCall = nil
        fpvLock.unlock()
        fpvCall?.stopMedia()
    }

    private func applyFpvEffectsLocked(_ effects: [FpvEffect]) {
        for effect in effects {
            switch effect.kind {
            case "publish":
                av.publishFpvSignal(topic: effect.topic, payload: effect.payload)
            case "create_offer":
                let call = ensureFpvCallLocked()
                fpvLock.lock()
                fpvActiveCall = call
                fpvLock.unlock()
                print("[FPV] creating WebRTC offer for session \(effect.session)")
                call.start(session: effect.session, settings: activeFpvIce, sharedCamera: true)
            case "apply_answer":
                print("[FPV] applying WebRTC answer for session \(effect.session)")
                fpvCall?.applyAnswer(session: effect.session, sdp: effect.sdp)
            case "add_ice":
                fpvCall?.addIce(
                    session: effect.session,
                    candidate: effect.candidate,
                    sdpMid: effect.sdpMid,
                    sdpMLineIndex: Int(effect.sdpMLineIndex)
                )
            case "end_of_candidates":
                break
            case "stop_media":
                print("[FPV] stopping media for session \(effect.session)")
                clearFpvCallLocked()
            case "show":
                updateFpvUi(state: effect.state, detail: effect.detail)
            default:
                break
            }
        }
    }

    private func handleFpvVideoEventLocked(_ event: FpvVideoEvent) {
        let now = fpvNowMs()
        let effects: [FpvEffect]
        switch event {
        case let .offer(session, sdp):
            effects = av.fpvNoteOffer(session: session, sdp: sdp, nowMs: now)
        case let .ice(session, candidate, sdpMid, sdpMLineIndex):
            effects = av.fpvNoteIce(
                session: session,
                candidate: candidate,
                sdpMid: sdpMid,
                sdpMLineIndex: Int32(sdpMLineIndex),
                nowMs: now
            )
        case let .gatheringComplete(session):
            effects = av.fpvNoteIce(
                session: session,
                candidate: "",
                sdpMid: "",
                sdpMLineIndex: 0,
                nowMs: now
            )
        case let .state(session, state):
            print("[FPV] media state=\(state) session=\(session)")
            effects = av.fpvNoteMedia(session: session, state: state, nowMs: now)
        case let .cameraUnavailable(session, reason):
            print("[FPV] camera unavailable (\(reason)) session=\(session)")
            effects = av.fpvNoteMedia(session: session, state: "failed", nowMs: now)
        }
        applyFpvEffectsLocked(effects)
    }

    private func updateFpvUi(state: String, detail: String) {
        let label = fpvStatusLabel(state: state, detail: detail)
        let live = state == "live"
        let active = state == "connecting" || state == "live" || state == "reconnecting"
        DispatchQueue.main.async {
            if self.fpvStatus != label { self.fpvStatus = label }
            if self.fpvLive != live { self.fpvLive = live }
            if self.fpvActive != active { self.fpvActive = active }
        }
    }

    private func fpvStatusLabel(state: String, detail: String) -> String {
        switch state {
        case "connecting": return "Connecting"
        case "live": return "Live"
        case "reconnecting": return "Reconnecting"
        case "failed": return detail.isEmpty ? "Failed" : "Failed: \(detail)"
        case "stopped": return "Stopped"
        default: return "Idle"
        }
    }

    private func fpvNowMs() -> UInt64 {
        UInt64(max(0, ProcessInfo.processInfo.systemUptime * 1000.0))
    }
}
