import OSLog
import ARKit
import AVFoundation
import Combine
import CoreMotion
import CoreLocation
import Foundation
import QuartzCore
import simd

// ARKit session: frame intake, tracking loss, mount calibration and camera-offset fit.
extension PhoneController {
    /// Runs on `arFrameQueue`. ARKit stops delivering camera images once the delegate holds
    /// too many frames, so keep at most one waiting and one being processed.
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        let delivered = CACurrentMediaTime()
        if mode == .phone { taskAgent.offer(pixelBuffer: frame.capturedImage, timestamp: frame.timestamp) }
        if fpvRunning && readSharedCamera() {
            fpvVideo.consume(pixelBuffer: frame.capturedImage, timestamp: frame.timestamp)
        }
        arFrameLock.lock()
        if pendingARFrame != nil { droppedARFrames += 1 }
        let dropped = droppedARFrames
        pendingARFrame = frame
        pendingARFrameDelivered = delivered
        let schedule = !arFrameScheduled
        arFrameScheduled = true
        arFrameLock.unlock()
        if dropped > 0 && dropped % 300 == 0 {
            TerraLog.tracking.notice("AR frames dropped while the control queue was busy; total=\(dropped)")
        }
        if schedule { controlQueue.async { self.drainARFrame() } }
    }
    /// One frame per control-queue block, so control ticks queued meanwhile still run.
    private func drainARFrame() {
        arFrameLock.lock()
        let frame = pendingARFrame
        let delivered = pendingARFrameDelivered, dropped = droppedARFrames
        pendingARFrame = nil
        if frame == nil { arFrameScheduled = false }
        arFrameLock.unlock()
        guard let frame else { return }
        let started = CACurrentMediaTime()
        timingVioPushedAt = .nan
        autoreleasepool { processARFrame(frame) }
        if mode == .phone {
            SensorTimingProbe.shared.record(.ar, sensor: frame.timestamp, os: delivered, queue: started, done: timingVioPushedAt, a: CACurrentMediaTime(), b: Double(dropped))
        }
        controlQueue.async { self.drainARFrame() }
    }
    private func processARFrame(_ frame: ARFrame) {
        guard mode == .phone, frame.timestamp >= phoneStartedAt, let controller else { return }
        guard phoneFrameOrder.accept(frame.timestamp, notBefore: phoneStartedAt) else {
            TerraLog.tracking.warning("Repeated or older AR frame skipped; timestamp=\(frame.timestamp) lastAccepted=\(self.phoneFrameOrder.lastTimestamp ?? 0)")
            return
        }
        let tracked: Bool
        if case .normal = frame.camera.trackingState { tracked = true } else { tracked = false }
        if localizationTracked != tracked {
            TerraLog.tracking.notice("Camera tracking changed; healthy=\(tracked) state=\(String(describing: frame.camera.trackingState), privacy: .public)")
        }
        localizationTracked = tracked
        switch frame.camera.trackingState {
        case .normal: localizationTrackingReason = nil
        case .notAvailable: localizationTrackingReason = "AR tracking unavailable · restart phone tracking"
        case .limited(let reason):
            switch reason {
            case .initializing: localizationTrackingReason = "AR tracking initializing · slowly scan a textured surface"
            case .excessiveMotion: localizationTrackingReason = "AR tracking limited by excessive motion · move the phone more slowly"
            case .insufficientFeatures: localizationTrackingReason = "AR tracking has too few visual features · improve lighting and point the camera at a textured surface"
            case .relocalizing: localizationTrackingReason = "AR tracking relocalizing · point the camera at the previously tracked area"
            @unknown default: localizationTrackingReason = "AR tracking limited · keep the camera steady"
            }
        }
        sensorEpochLock.lock(); let trackingEpoch = sensorEpoch; sensorEpochLock.unlock()
        DispatchQueue.main.async {
            guard self.sensorEpochMatches(trackingEpoch), self.phoneTrackingActive else { return }
            self.phoneTrackingStatus = tracked ? (self.fleetTrackingManaged ? "Tracking active · managed by ARGOS in every mode" : "Phone tracking active") : "Tracking initializing · keep the phone steady"
        }
        updateObjectSearch(frame: frame, tracked: tracked, epoch: trackingEpoch)
        let transform = frame.camera.transform
        phoneHeadingReferenceYaw = PhoneMountGeometry.headingReferenceYaw(
            worldFromCamera: worldFromAR * simd_float3x3(simd_quatf(transform)),
            using: PhoneMountGeometry.headingReference(deviceToBody: mountCalibration.deviceToBody))
        let arPosition = SIMD3(transform.columns.3.x, transform.columns.3.y, transform.columns.3.z)
        let position = worldFromAR * arPosition
        let orientation = simd_quatf(worldFromAR) * simd_quatf(transform) * mountCalibration.arCameraToBody.inverse
        if isCollectingOffsetFit {
            if !tracked {
                isCollectingOffsetFit = false
                offsetFitSamples.removeAll(keepingCapacity: true)
                DispatchQueue.main.async {
                    self.offsetFitInProgress = false
                    self.offsetFitStatus = "Fit cancelled because ARKit tracking was interrupted"
                }
            } else {
                let roverForward = orientation.act(SIMD3<Float>(1, 0, 0))
                offsetFitSamples.append(RotationOffsetSample(
                    timestamp: frame.timestamp,
                    cameraPositionWorld: SIMD2(Double(position.x), Double(position.y)),
                    roverHeadingWorld: atan2(Double(roverForward.y), Double(roverForward.x))
                ))
                if offsetFitSamples.count > 12000 {
                    isCollectingOffsetFit = false
                    offsetFitSamples.removeAll(keepingCapacity: true)
                    DispatchQueue.main.async {
                        self.offsetFitInProgress = false
                        self.offsetFitStatus = "Fit cancelled: turn exceeded the capture duration. Start a new pass."
                    }
                }
            }
        }
        let offset = mountCalibration.isPlausible ? SIMD3(Float(mountCalibration.cameraOffsetForward), Float(mountCalibration.cameraOffsetLeft), Float(mountCalibration.cameraHeight)) : .zero
        let roverPosition = position - orientation.act(offset)
        var velocityReady = false
        if tracked, let last = previousPosition, let previousTime = previousPoseTime {
            let dt = frame.timestamp - previousTime
            if dt > 0 && dt < 0.2 {
                let raw = (roverPosition - last) / Float(dt)
                let alpha = Float(dt / (0.03 + dt))
                filteredVelocity += alpha * (raw - filteredVelocity)
                velocityReady = true
            }
        }
        previousPosition = tracked ? roverPosition : nil
        previousPoseTime = tracked ? frame.timestamp : nil
        if !tracked {
            filteredVelocity = .zero
            if bleActive {
                feedbackTrackingHealthy = false
                let fullyManual = TerraFleetManualDrive.isFullyManual(controller.autonomyBrief())
                if DriveJoystickSafety.trackingLossRequiresStop(feedback: bleFeedback || TerraFleetManualDrive.requiresVioControl(controller.autonomyBrief()), fullyManual: fullyManual) {
                    pauseHardwareMotion(reason: localizationTrackingReason ?? "Camera tracking lost in tracking-dependent mode")
                }
            }
        }
        let q = orientation.vector
        let bodyForward = orientation.act(SIMD3<Float>(1, 0, 0))
        phonePose = (Double(roverPosition.x), Double(roverPosition.y), atan2(Double(bodyForward.y), Double(bodyForward.x)))
        do {
            try controller.pushVio(sample: VioReading(timestamp: frame.timestamp, positionX: Double(roverPosition.x), positionY: Double(roverPosition.y), positionZ: Double(roverPosition.z), quaternionX: Double(q.x), quaternionY: Double(q.y), quaternionZ: Double(q.z), quaternionW: Double(q.w), velocityX: Double(filteredVelocity.x), velocityY: Double(filteredVelocity.y), velocityZ: Double(filteredVelocity.z), tracked: tracked && velocityReady))
            timingLastVio = frame.timestamp; timingVioPushedAt = CACurrentMediaTime()
            if tracked { updatePhoneGround(frame: frame, position: position, bodyUp: orientation.act(SIMD3(0, 0, 1))); updatePhoneMap(frame: frame, position: position, roverPosition: roverPosition, bodyOrientation: orientation) }
            else { DispatchQueue.main.async { self.mapStatus = "Tracking lost · map paused" } }
        } catch { fail(error, context: "AR frame at \(frame.timestamp)") }
    }
    /// Mount measurements feed the live IMU and VIO path, so saving stops the
    /// controller first. stop() queues the switch to .stopped on the serial
    /// control queue ahead of the save below.
    func saveMountCalibration(_ value: MountCalibration) {
        guard value.isPlausible else {
            calibrationStatus = "Calibration not saved: \(CalibrationSaveError.invalidValues.localizedDescription)"; return
        }
        guard !offsetFitInProgress else {
            calibrationStatus = "Calibration not saved: finish the turn pass first."; return
        }
        stop()
        controlQueue.async {
            do {
                guard value.isPlausible else { throw CalibrationSaveError.invalidValues }
                guard self.mode == .stopped else { throw CalibrationSaveError.stopFirst }
                try value.save()
                self.mountCalibration = value
                if let controller = self.controller { self.applyDock(to: controller) }
                let headingOrientation = PhoneMountGeometry.headingReference(deviceToBody: value.deviceToBody).orientation
                DispatchQueue.main.async {
                    self.locationManager.headingOrientation = headingOrientation
                    self.calibrationStatus = value.reviewed ? "Measured calibration saved · reviewed" : "Measured calibration saved · review required"
                }
            } catch {
                DispatchQueue.main.async { self.calibrationStatus = "Calibration not saved: \(error.localizedDescription)" }
            }
        }
    }
    func beginAutomaticOffsetFit() {
        controlQueue.async {
            guard !self.isCollectingOffsetFit else { return }
            guard self.mode == .phone else {
                DispatchQueue.main.async { self.offsetFitStatus = "Start Phone IMU + VIO before fitting the offset" }
                return
            }
            guard self.mountCalibration.cameraHeight > 0 else {
                DispatchQueue.main.async { self.offsetFitStatus = "Enter and save camera height before fitting" }
                return
            }
            if self.firstTurnFit == nil {
                DispatchQueue.main.async { self.fittedCameraOffset = nil }
            }
            self.offsetFitSamples.removeAll(keepingCapacity: true)
            self.isCollectingOffsetFit = true
            DispatchQueue.main.async {
                self.offsetFitInProgress = true
                self.offsetFitStatus = "Turn the rover at least 90° in one direction around its marked reference point"
            }
        }
    }
    func finishAutomaticOffsetFit() {
        controlQueue.async {
            guard self.isCollectingOffsetFit else { return }
            self.isCollectingOffsetFit = false
            DispatchQueue.main.async { self.offsetFitInProgress = false }
            let samples = self.offsetFitSamples
            self.offsetFitSamples.removeAll(keepingCapacity: true)
            guard let fit = RotationOffsetFitter.fit(samples) else {
                DispatchQueue.main.async {
                    self.offsetFitInProgress = false
                    self.offsetFitStatus = "Fit rejected: need stable tracking, ≥90° turn, and ≤5 cm path error. Retry slowly."
                }
                return
            }
            if let first = self.firstTurnFit {
                guard first.turnDirection != fit.turnDirection,
                      hypot(first.forward - fit.forward, first.left - fit.left) <= 0.05 else {
                    self.firstTurnFit = nil
                    DispatchQueue.main.async {
                        self.offsetFitInProgress = false
                        self.offsetFitStatus = "Reverse-turn result disagreed by over 5 cm. Recheck the pivot and tracking, then repeat both turns."
                    }
                    return
                }
                self.firstTurnFit = nil
                let forward = (first.forward + fit.forward) * 0.5
                let left = (first.left + fit.left) * 0.5
                DispatchQueue.main.async {
                    self.fittedCameraOffset = SIMD2(forward, left)
                    self.offsetFitStatus = String(format: "Two-way fit accepted · forward %+.3f m · left %+.3f m · RMS %.1f / %.1f cm",
                                                  forward, left, first.rmsError * 100, fit.rmsError * 100)
                }
            } else {
                self.firstTurnFit = fit
                DispatchQueue.main.async {
                    self.offsetFitStatus = String(format: "First pass accepted · %.1f cm RMS. Now start and perform a turn in the opposite direction.", fit.rmsError * 100)
                }
            }
        }
    }
    func handleTrackingFailure(_ error: Error) {
        sensorEpochLock.lock(); let epoch = sensorEpoch; sensorEpochLock.unlock()
        DispatchQueue.main.async {
            guard self.sensorEpochMatches(epoch), self.phoneTrackingActive else { return }
            if self.fleetTrackingManaged {
                self.fleetTrackingFailed = true
                self.setHardwareFeedback(enabled: false)
                self.phoneTrackingStatus = "Tracking unavailable: " + error.localizedDescription
            } else { self.controlQueue.async { self.fail(error) } }
        }
    }
    func session(_ session: ARSession, didFailWithError error: Error) { handleTrackingFailure(error) }
    func sessionWasInterrupted(_ session: ARSession) { controlQueue.async { self.localizationPolicy.reset(); self.localizationTracked = false; self.phoneHeadingReferenceYaw = nil }; handleTrackingFailure(NSError(domain: "Terra", code: 1, userInfo: [NSLocalizedDescriptionKey: "AR session interrupted"])) }
    /// Confirm (or withdraw) that the phone is on the rover mount. Until confirmed,
    /// only the local debug joystick may drive; fleet teleop and autonomy are held.
    func setPhoneMounted(_ mounted: Bool) {
        controlQueue.async {
            self.phoneMountConfirmed = mounted
            self.mountHoldReported = false
            DispatchQueue.main.async { self.phoneMounted = mounted }
        }
    }
}

private enum CalibrationSaveError: LocalizedError {
    case invalidValues, stopFirst
    var errorDescription: String? { self == .stopFirst ? "Stop the controller before saving mount measurements." : "Check the measured values and camera height." }
}
