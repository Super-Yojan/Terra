import AVFoundation
import CoreMedia
import CoreVideo
import Foundation
import WebRTC

/// Events from the send-only rear-camera peer connection.
/// The video path does not carry drive commands. A data channel, if the dashboard opens one, is ignored.
enum FpvVideoEvent: Equatable {
    case offer(session: String, sdp: String)
    case ice(session: String, candidate: String, sdpMid: String, sdpMLineIndex: Int)
    case gatheringComplete(session: String)
    case state(session: String, state: String)
    case cameraUnavailable(session: String, reason: String)
}

/// One send-only H.264 (VP8 fallback) video call. Audio stays disabled.
final class FpvVideoCall: NSObject, RTCPeerConnectionDelegate {
    var onEvent: ((FpvVideoEvent) -> Void)?
    private let queue = DispatchQueue(label: "terra.fpv.video")
    private var peer: RTCPeerConnection?
    private var videoSource: RTCVideoSource?
    private var frameCapturer: RTCVideoCapturer?
    private var dedicatedCapturer: RTCCameraVideoCapturer?
    private var sessionID = ""
    private var sharedCamera = false
    private var lastFrameTime = -Double.infinity
    private let frameLock = NSLock()
    private var pendingBuffer: CVPixelBuffer?
    private var pendingTime: TimeInterval = 0
    private var frameScheduled = false

    private static let factory: RTCPeerConnectionFactory = {
        RTCInitializeSSL()
        let audio = RTCAudioSession.sharedInstance()
        audio.useManualAudio = true
        audio.isAudioEnabled = false
        let encoder = RTCDefaultVideoEncoderFactory()
        let decoder = RTCDefaultVideoDecoderFactory()
        if let h264 = RTCDefaultVideoEncoderFactory.supportedCodecs().first(where: { $0.name.caseInsensitiveCompare("H264") == .orderedSame }) {
            encoder.preferredCodec = h264
        }
        return RTCPeerConnectionFactory(encoderFactory: encoder, decoderFactory: decoder)
    }()

    func start(session: String, settings: FpvIceSettings, sharedCamera: Bool) {
        queue.async {
            self.stopLocked()
            self.sessionID = session
            self.sharedCamera = sharedCamera
            self.lastFrameTime = -Double.infinity
            guard let peer = self.makePeer(settings: settings), let source = self.videoSource else {
                self.emit(.cameraUnavailable(session: session, reason: "The video call could not be created"))
                return
            }
            self.peer = peer
            if sharedCamera {
                self.frameCapturer = RTCVideoCapturer(delegate: source)
            } else {
                self.startDedicatedCapture(session: session)
            }
            self.createOffer(session: session)
        }
    }

    func setSharedCamera(_ shared: Bool) {
        queue.async {
            guard self.peer != nil, let source = self.videoSource else {
                self.sharedCamera = shared
                return
            }
            self.sharedCamera = shared
            if shared {
                self.stopDedicatedCapture()
                if self.frameCapturer == nil {
                    self.frameCapturer = RTCVideoCapturer(delegate: source)
                }
            } else if self.dedicatedCapturer == nil {
                self.startDedicatedCapture(session: self.sessionID)
            }
        }
    }

    /// Keep the newest rear-camera frame from ARKit. Frames are dropped while a previous one is still queued.
    func consume(pixelBuffer: CVPixelBuffer, timestamp: TimeInterval) {
        frameLock.lock()
        pendingBuffer = pixelBuffer
        pendingTime = timestamp
        let schedule = !frameScheduled
        if schedule { frameScheduled = true }
        frameLock.unlock()
        guard schedule else { return }
        queue.async { self.drainFrames() }
    }

    func applyAnswer(session: String, sdp: String) {
        queue.async {
            guard session == self.sessionID, let peer = self.peer else { return }
            let description = RTCSessionDescription(type: .answer, sdp: sdp)
            peer.setRemoteDescription(description) { [weak self] error in
                guard let self else { return }
                if error != nil {
                    self.queue.async {
                        guard session == self.sessionID else { return }
                        self.emit(.state(session: session, state: "failed"))
                    }
                }
            }
        }
    }

    func addIce(session: String, candidate: String, sdpMid: String, sdpMLineIndex: Int) {
        queue.async {
            guard session == self.sessionID, let peer = self.peer, !candidate.isEmpty else { return }
            let ice = RTCIceCandidate(sdp: candidate, sdpMLineIndex: Int32(sdpMLineIndex), sdpMid: sdpMid.isEmpty ? nil : sdpMid)
            peer.add(ice) { _ in }
        }
    }

    func stopMedia() {
        queue.async { self.stopLocked() }
    }

    private func makePeer(settings: FpvIceSettings) -> RTCPeerConnection? {
        let source = Self.factory.videoSource()
        source.adaptOutputFormat(toWidth: 1280, height: 720, fps: 30)
        videoSource = source
        let track = Self.factory.videoTrack(with: source, trackId: "fpv-rear")
        let configuration = RTCConfiguration()
        configuration.iceServers = settings.iceServers().map { server in
            if server.username.isEmpty && server.credential.isEmpty {
                return RTCIceServer(urlStrings: server.urls)
            }
            return RTCIceServer(urlStrings: server.urls, username: server.username, credential: server.credential)
        }
        configuration.sdpSemantics = .unifiedPlan
        configuration.continualGatheringPolicy = .gatherContinually
        configuration.bundlePolicy = .maxBundle
        configuration.rtcpMuxPolicy = .require
        let constraints = RTCMediaConstraints(mandatoryConstraints: nil, optionalConstraints: nil)
        guard let peer = Self.factory.peerConnection(with: configuration, constraints: constraints, delegate: self),
              let transceiver = peer.addTransceiver(with: track) else { return nil }
        var directionError: NSError?
        transceiver.setDirection(.sendOnly, error: &directionError)
        preferH264(transceiver)
        applyBitrate(transceiver.sender)
        return peer
    }

    private func preferH264(_ transceiver: RTCRtpTransceiver) {
        let capabilities = Self.factory.rtpSenderCapabilities(forKind: kRTCMediaStreamTrackKindVideo)
        let preferred = capabilities.codecs.filter { codecRank($0) < 9 }.sorted { codecRank($0) < codecRank($1) }
        guard !preferred.isEmpty else { return }
        _ = try? transceiver.setCodecPreferences(preferred, error: ())
    }

    private func codecRank(_ codec: RTCRtpCodecCapability) -> Int {
        let name = codec.name.lowercased()
        let profile = codec.parameters["profile-level-id"]?.lowercased() ?? ""
        if name == "h264", profile.hasPrefix("42e0") { return 0 }
        if name == "h264" { return 1 }
        if name == "vp8" { return 2 }
        return 9
    }

    private func applyBitrate(_ sender: RTCRtpSender) {
        let parameters = sender.parameters
        for encoding in parameters.encodings {
            encoding.isActive = true
            encoding.maxBitrateBps = NSNumber(value: 1_500_000)
            encoding.minBitrateBps = NSNumber(value: 150_000)
            encoding.maxFramerate = NSNumber(value: 30)
        }
        parameters.degradationPreference = NSNumber(value: RTCDegradationPreference.balanced.rawValue)
        sender.parameters = parameters
    }

    private func createOffer(session: String) {
        let constraints = RTCMediaConstraints(
            mandatoryConstraints: [
                kRTCMediaConstraintsOfferToReceiveAudio: kRTCMediaConstraintsValueFalse,
                kRTCMediaConstraintsOfferToReceiveVideo: kRTCMediaConstraintsValueFalse,
            ],
            optionalConstraints: nil
        )
        peer?.offer(for: constraints) { [weak self] description, error in
            guard let self else { return }
            self.queue.async {
                guard session == self.sessionID else { return }
                guard let peer = self.peer, error == nil, let description else {
                    self.emit(.state(session: session, state: "failed"))
                    return
                }
                peer.setLocalDescription(description) { [weak self] setError in
                    guard let self else { return }
                    self.queue.async {
                        guard session == self.sessionID, setError == nil, let sdp = self.peer?.localDescription?.sdp, !sdp.isEmpty else {
                            self.emit(.state(session: session, state: "failed"))
                            return
                        }
                        self.emit(.offer(session: session, sdp: sdp))
                    }
                }
            }
        }
    }

    private func startDedicatedCapture(session: String) {
        switch AVCaptureDevice.authorizationStatus(for: .video) {
        case .authorized:
            openDedicatedCamera(session: session)
        case .notDetermined:
            AVCaptureDevice.requestAccess(for: .video) { [weak self] granted in
                self?.queue.async {
                    guard let self, session == self.sessionID else { return }
                    if granted { self.openDedicatedCamera(session: session) }
                    else { self.emit(.cameraUnavailable(session: session, reason: "Camera access is off")) }
                }
            }
        default:
            emit(.cameraUnavailable(session: session, reason: "Camera access is off"))
        }
    }

    private func openDedicatedCamera(session: String) {
        guard session == sessionID, peer != nil, let source = videoSource else { return }
        guard let device = RTCCameraVideoCapturer.captureDevices().first(where: { $0.position == .back }) else {
            emit(.cameraUnavailable(session: session, reason: "The rear camera is unavailable"))
            return
        }
        guard let chosen = format(for: device) else {
            emit(.cameraUnavailable(session: session, reason: "The rear camera has no supported format"))
            return
        }
        let capturer = RTCCameraVideoCapturer(delegate: source)
        dedicatedCapturer = capturer
        frameCapturer = nil
        capturer.startCapture(with: device, format: chosen.format, fps: chosen.fps) { [weak self] error in
            self?.queue.async {
                guard let self, self.dedicatedCapturer === capturer else { return }
                if error != nil {
                    self.emit(.cameraUnavailable(session: session, reason: "The rear camera is already in use"))
                }
            }
        }
    }

    private func format(for device: AVCaptureDevice) -> (format: AVCaptureDevice.Format, fps: Int)? {
        let formats = RTCCameraVideoCapturer.supportedFormats(for: device)
        let ranked = formats.compactMap { format -> (AVCaptureDevice.Format, Int)? in
            let dimensions = CMVideoFormatDescriptionGetDimensions(format.formatDescription)
            guard format.videoSupportedFrameRateRanges.contains(where: { $0.maxFrameRate >= 30 }) else { return nil }
            let distance = abs(Int(dimensions.width) - 1280) + abs(Int(dimensions.height) - 720)
            return (format, distance)
        }
        if let best = ranked.min(by: { $0.1 < $1.1 })?.0 { return (best, 30) }
        return formats.first.map { ($0, 30) }
    }

    private func stopDedicatedCapture() {
        dedicatedCapturer?.stopCapture()
        dedicatedCapturer = nil
    }

    private func stopLocked() {
        stopDedicatedCapture()
        peer?.close()
        peer = nil
        videoSource = nil
        frameCapturer = nil
        sessionID = ""
        frameLock.lock()
        pendingBuffer = nil
        frameScheduled = false
        frameLock.unlock()
    }

    private func drainFrames() {
        while true {
            frameLock.lock()
            let buffer = pendingBuffer
            let timestamp = pendingTime
            pendingBuffer = nil
            if buffer == nil {
                frameScheduled = false
                frameLock.unlock()
                return
            }
            frameLock.unlock()
            guard let buffer, sharedCamera, let source = videoSource, let capturer = frameCapturer else { continue }
            if timestamp >= lastFrameTime, timestamp - lastFrameTime < (1.0 / 30.0) { continue }
            lastFrameTime = timestamp
            // Portrait mount: sensor buffer is landscape, and the phone's top edge is up.
            let frame = RTCVideoFrame(buffer: RTCCVPixelBuffer(pixelBuffer: buffer), rotation: ._90, timeStampNs: Int64((timestamp * 1_000_000_000).rounded()))
            source.capturer(capturer, didCapture: frame)
        }
    }

    private func emit(_ event: FpvVideoEvent) {
        onEvent?(event)
    }

    func peerConnection(_ peerConnection: RTCPeerConnection, didChange stateChanged: RTCSignalingState) {}

    func peerConnection(_ peerConnection: RTCPeerConnection, didAdd stream: RTCMediaStream) {}

    func peerConnection(_ peerConnection: RTCPeerConnection, didRemove stream: RTCMediaStream) {}

    func peerConnectionShouldNegotiate(_ peerConnection: RTCPeerConnection) {}

    func peerConnection(_ peerConnection: RTCPeerConnection, didChange newState: RTCIceConnectionState) {
        let state: String
        switch newState {
        case .new, .checking, .disconnected: state = "connecting"
        case .connected, .completed: state = "connected"
        case .failed: state = "failed"
        case .closed: state = "closed"
        case .count: return
        @unknown default: state = "failed"
        }
        let reported = state
        queue.async {
            guard peerConnection === self.peer, !self.sessionID.isEmpty else { return }
            self.emit(.state(session: self.sessionID, state: reported))
        }
    }

    func peerConnection(_ peerConnection: RTCPeerConnection, didChange newState: RTCIceGatheringState) {
        guard newState == .complete else { return }
        queue.async {
            guard peerConnection === self.peer, !self.sessionID.isEmpty else { return }
            self.emit(.gatheringComplete(session: self.sessionID))
        }
    }

    func peerConnection(_ peerConnection: RTCPeerConnection, didGenerate candidate: RTCIceCandidate) {
        let sdp = candidate.sdp
        let mid = candidate.sdpMid ?? ""
        let index = Int(candidate.sdpMLineIndex)
        queue.async {
            guard peerConnection === self.peer, !self.sessionID.isEmpty else { return }
            self.emit(.ice(session: self.sessionID, candidate: sdp, sdpMid: mid, sdpMLineIndex: index))
        }
    }

    func peerConnection(_ peerConnection: RTCPeerConnection, didRemove candidates: [RTCIceCandidate]) {}

    func peerConnection(_ peerConnection: RTCPeerConnection, didOpen dataChannel: RTCDataChannel) {
        // Video only. The channel is not read and is not a drive path.
    }
}
