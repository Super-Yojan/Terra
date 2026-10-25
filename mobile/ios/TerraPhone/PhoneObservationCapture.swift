import Foundation
import os

/// Exposure-clock sampling resets when localization establishes a new coordinate frame.
/// The unfair lock donates priority to a lower-QoS holder, so the AR frame thread never inverts on it.
final class ObservationCapturePolicy {
    private let lock = OSAllocatedUnfairLock()
    private var interval: Double
    init(interval: Double = 1) { self.interval = interval }
    func configure(interval: Double) {
        guard interval.isFinite, interval >= 0.01, interval <= 3600 else { return }
        lock.lock(); defer { lock.unlock() }; self.interval = interval
    }
    private var frameID = ""
    private var last = -Double.infinity
    func shouldCapture(exposure: Double, frameID: String) -> Bool {
        lock.lock(); defer { lock.unlock() }
        guard exposure.isFinite, exposure >= 0 else { return false }
        if self.frameID != frameID { self.frameID = frameID; last = -Double.infinity }
        guard exposure - last >= interval else { return false }
        last = exposure
        return true
    }
}

#if os(iOS)
import ARKit
import CoreImage

final class PhoneObservationCapture: @unchecked Sendable {
    private let policy = ObservationCapturePolicy()
    private let mapPolicy = ObservationCapturePolicy(interval: 0.5)
    /// In-flight slot count. A semaphore signalled from the utility queue makes the AR frame
    /// thread's non-blocking wait a priority inversion; an unfair lock carries a priority donation.
    private let inFlight = OSAllocatedUnfairLock(initialState: 0)
    private func acquireSlot() -> Bool {
        inFlight.withLock { count in
            guard count < 2 else { return false }
            count += 1
            return true
        }
    }
    private func releaseSlot() { inFlight.withLock { $0 -= 1 } }
    private let queue = DispatchQueue(label: "terra.observation.capture", qos: .utility)
    private let imageContext = CIContext()
    func configure(evidenceSeconds: Double, mapSeconds: Double) {
        policy.configure(interval: evidenceSeconds); mapPolicy.configure(interval: mapSeconds)
    }
    func captureMap(grid: OccupancyGrid, exposure: Double, frameID: String, revision: UInt64, rover: String, controller: MobileController) {
        guard mapPolicy.shouldCapture(exposure: exposure, frameID: frameID) else { return }
        guard acquireSlot() else { controller.recordObservationGap(reason: "map_capture_queue_full"); return }
        let run = controller.observationRunId()
        queue.async {
            defer { self.releaseSlot() }
            let metadata: [String: Any] = ["rover_id": rover, "session_id": run, "exposure_ns": UInt64(exposure * 1e9), "frame_id": frameID, "evidence_type": "occupancy", "map_revision": revision, "occupancy": ["width": grid.width, "height": grid.height, "origin_x": grid.originX, "origin_y": grid.originY, "resolution": grid.resolution]]
            guard let data = try? JSONSerialization.data(withJSONObject: metadata) else { controller.recordObservationGap(reason: "map_manifest_encoding_failed"); return }
            let attachment = ObservationAttachment(name: "occupancy", data: Data(grid.occupancy.map { UInt8(bitPattern: $0) }), encoding: "i8", dimensions: [grid.width, grid.height], units: "occupancy_percent")
            if !controller.recordMappingEvidence(manifestJson: String(decoding: data, as: UTF8.self), attachments: [attachment]) { controller.recordObservationGap(reason: "map_capture_rejected") }
        }
    }
    func capture(frame: ARFrame, depth: [Float], width: Int, height: Int, grid: OccupancyGrid?, manifest: [String: Any], controller: MobileController) {
        guard policy.shouldCapture(exposure: frame.timestamp, frameID: manifest["frame_id"] as? String ?? "") else { return }
        guard acquireSlot() else { controller.recordObservationGap(reason: "camera_capture_queue_full"); return }
        queue.async {
            defer { self.releaseSlot() }
            var metadata = manifest
            let image = CIImage(cvPixelBuffer: frame.capturedImage)
            var attachments = [ObservationAttachment(name: "depth", data: depth.withUnsafeBytes { Data($0) }, encoding: "f32le", dimensions: [UInt32(width), UInt32(height)], units: "metres")]
            let validity = depth.map { $0.isFinite && $0 > 0 ? UInt8(1) : UInt8(0) }
            attachments.append(ObservationAttachment(name: "depth_validity", data: Data(validity), encoding: "u8", dimensions: [UInt32(width), UInt32(height)], units: "valid"))
            if let jpeg = self.imageContext.jpegRepresentation(of: image, colorSpace: CGColorSpaceCreateDeviceRGB(), options: [CIImageRepresentationOption(rawValue: kCGImageDestinationLossyCompressionQuality as String): 0.75]) {
                metadata["rgb_exposure_ns"] = UInt64(frame.timestamp * 1e9)
                attachments.append(ObservationAttachment(name: "rgb", data: jpeg, encoding: "jpeg", dimensions: [UInt32(image.extent.width), UInt32(image.extent.height)], units: "rgb"))
            } else { metadata["rgb_missing_reason"] = "encoding_failed" }
            if let grid {
                metadata["occupancy"] = ["width": grid.width, "height": grid.height, "resolution": grid.resolution, "origin_x": grid.originX, "origin_y": grid.originY]
                attachments.append(ObservationAttachment(name: "occupancy", data: Data(grid.occupancy.map { UInt8(bitPattern: $0) }), encoding: "i8", dimensions: [grid.width, grid.height], units: "occupancy_percent"))
            }
            guard let data = try? JSONSerialization.data(withJSONObject: metadata) else { return }
            if !controller.recordMappingEvidence(manifestJson: String(decoding: data, as: UTF8.self), attachments: attachments) { controller.recordObservationGap(reason: "camera_capture_rejected") }
        }
    }
}
#endif

#if os(iOS)
import Security
/// Import a scoped producer token once, then keep it in this device's keychain.
enum ObservationUploadSettings {
    static func load(directory: URL, producer: String) -> (endpoint: String, token: String)? {
        let defaults = UserDefaults.standard
        let service = "org.terra.observability"
        let key = "observationEndpoint/" + producer
        let file = directory.appendingPathComponent("upload.json")
        let query: [String: Any] = [kSecClass as String: kSecClassGenericPassword, kSecAttrService as String: service, kSecAttrAccount as String: producer]
        if let raw = try? Data(contentsOf: file), let config = try? JSONSerialization.jsonObject(with: raw) as? [String: String],
           config["producer_id"] == producer, let endpoint = config["endpoint"], let token = config["token"], !token.isEmpty {
            var add = query; add[kSecValueData as String] = Data(token.utf8); add[kSecAttrAccessible as String] = kSecAttrAccessibleAfterFirstUnlockThisDeviceOnly
            let status: OSStatus
            if SecItemCopyMatching(query as CFDictionary, nil) == errSecSuccess { status = SecItemUpdate(query as CFDictionary, [kSecValueData as String: Data(token.utf8)] as CFDictionary) }
            else { status = SecItemAdd(add as CFDictionary, nil) }
            if status == errSecSuccess { defaults.set(endpoint, forKey: key); try? FileManager.default.removeItem(at: file) }
        }
        var read = query; read[kSecReturnData as String] = true; read[kSecMatchLimit as String] = kSecMatchLimitOne
        var result: CFTypeRef?
        guard SecItemCopyMatching(read as CFDictionary, &result) == errSecSuccess, let data = result as? Data,
              let token = String(data: data, encoding: .utf8), let endpoint = defaults.string(forKey: key) else { return nil }
        return (endpoint, token)
    }
}
#endif
