import OSLog
import ARKit
import AVFoundation
import Combine
import CoreMotion
import CoreLocation
import Foundation
import QuartzCore
import simd

// Bluetooth rover link: discovery, pairing, automatic connection and output routing.
extension PhoneController {
    func observeBluetooth() {
        bluetooth.onDiscovery = { [weak self] rover, generation in self?.discovered(rover, generation: generation) }
        bluetooth.onLifecycle = { [weak self] event, generation in self?.bluetoothEvent(event, generation: generation) }
        let timer = DispatchSource.makeTimerSource(queue: .main)
        timer.schedule(deadline: .now() + 1, repeating: .seconds(1))
        timer.setEventHandler { [weak self] in self?.serviceAutomaticConnections() }
        timer.resume(); connectionTimer = timer
        bluetooth.objectWillChange.sink { [weak self] _ in
            DispatchQueue.main.async { self?.refreshBluetooth() }
        }.store(in: &subscriptions)
        bluetooth.$replyJSON.sink { [weak self] text in self?.receiveConfiguration(text) }.store(in: &subscriptions)
    }
    private func refreshBluetooth() {
        defer { refreshConnectionPresentation() }
        if configurationGeneration != bluetooth.connectionGeneration {
            configurationGeneration = bluetooth.connectionGeneration
            controlQueue.async { self.target = (0, 0); self.resetServoTargets(); self.bluetooth.stopOutput() }
            actuatorEditor.invalidate(generation: configurationGeneration); editorRequest = nil; retryableEditorRequest = nil; hasRetryableActuatorRequest = false; presetUpload = nil; removeAfterBegin = nil; resetFaultRequest = nil
            configurationStatus = "Connection changed · stage draft again"
        }
        observation.recordChanged(component: "bluetooth", kind: "hardware.state", attributes: ["status": bluetooth.status, "ready": bluetooth.isReady, "output_active": bluetooth.outputActive])
        hardwareStatus = bluetooth.status; hardwareReady = bluetooth.isReady
        if let identifier = bluetooth.connectedIdentifier {
            UserDefaults.standard.set(identifier.uuidString, forKey: "preferredHardwareRover")
        }
        hardwareConfigurationReady = bluetooth.configurationReady
        let remoteStatus = (try? JSONSerialization.jsonObject(with: Data(bluetooth.statusJSON.utf8))) as? [String: Any]
        activeLayoutRevision = remoteStatus?["active_revision"] as? UInt32 ?? 0
        hardwareFault = remoteStatus?["fault"] as? String
        if let index = try? JSONDecoder().decode(ActuatorLayoutIndex.self, from: Data(bluetooth.layoutIndexJSON.utf8)) {
            let invalidated = actuatorEditor.generation != configurationGeneration || actuatorEditor.revision != index.revision
            actuatorEditor.synchronize(index: index, generation: configurationGeneration)
            if invalidated { editorRequest = nil; retryableEditorRequest = nil; hasRetryableActuatorRequest = false; presetUpload = nil; removeAfterBegin = nil; actuatorPageLoading = false }
        }

        hardwareOutputActive = bluetooth.outputActive
        if let fault = hardwareFault { retainStopReason("Pi output fault: \(fault)") }
        capabilitiesJSON = bluetooth.capabilitiesJSON; committedLayoutJSON = bluetooth.driveProfileJSON
        feedbackCompatible = (try? ActuatorDriveProfileSet.decode(json: committedLayoutJSON).supportsFeedback) ?? false
        vioControlCompatible = (try? ActuatorDriveProfileSet.decode(json: committedLayoutJSON).supportsVioControl) ?? false
        let layout = committedLayoutJSON
        let ready = hardwareReady
        let compatible = feedbackCompatible
        if requestedHardwareFeedback != nil && !ready { feedbackConnectionSawUnready = true }
        if let requested = requestedHardwareFeedback, feedbackConnectionSawUnready, bluetooth.connectedIdentifier == requested, ready {
            requestedHardwareFeedback = nil
            if compatible && ARWorldTrackingConfiguration.isSupported && motion.isDeviceMotionAvailable {
                startPhoneSensors()
                hardwareFeedback = true
                controlQueue.async { self.bleFeedback = true; self.target = (0, 0); self.resetServoTargets() }
            } else {
                configurationStatus = "Feedback requires a compatible fresh rover layout and available phone tracking"
                stopHardware()
            }
        }
        controlQueue.async {
            self.bleReady = ready
            self.bleCompatible = compatible
            if self.bleLayout != layout { self.bleLayout = layout; self.target = (0, 0); self.resetServoTargets() }
            if self.bleActive && (!ready || (self.bleFeedback && !compatible)) { self.revokeHardwareMotion(reason: "Bluetooth readiness lost; ready=\(ready) feedbackCompatible=\(compatible)") }
        }
    }
    var connectionNow: TimeInterval { ProcessInfo.processInfo.systemUptime }
    private func automaticConnectionEnabled() -> Bool {
        UserDefaults.standard.object(forKey: "autoConnectHardware") as? Bool != false
    }
    func autoConnectHardware() {
        #if !targetEnvironment(simulator)
        guard automaticConnectionEnabled(), !connectionPolicy.enabled else { return }
        let preferred = savedRovers.selected
        connectionPolicy.begin(preferred: preferred)
        roverConnectionFailed = false
        routerSuppressed = false; routerRetryAt = nil
        if let identifier = bluetooth.connectedIdentifier, connectionPolicy.adoptAuthenticated(identifier) {
            bluetooth.resumeAutomation(generation: connectionPolicy.generation)
        } else { bluetooth.startDiscovery(generation: connectionPolicy.generation) }
        #endif
    }
    func cancelAutoConnection() {
        if connectionPolicy.attempting != nil { stop(); return }
        connectionPolicy.stop(); pairingCandidate = nil; nearbyPairing.removeAll()
        routerSuppressed = true; routerRetryAt = nil; invalidateDashboardAttempt()
        dashboardConnecting = false
        bluetooth.cancelAutomaticConnection()
    }
    func retryAutomaticConnection() {
        cancelAutoConnection(); autoConnectHardware()
    }
    func disconnectHardware() { stop() }
    func forgetHardwareRover() {
        guard let id = savedRovers.selected else { return }
        forgetHardwareRover(id)
    }
    func forgetHardwareRover(_ id: UUID) {
        let wasSelected = savedRovers.selected == id
        if wasSelected { stop() }
        savedRovers.forget(id)
        guard wasSelected else { return }
        roverDisplayName = "Your rover"
        autoConnectHardware()
    }
    func selectHardwareRover(_ id: UUID) {
        guard savedRovers.rovers.contains(where: { $0.id == id }) else { return }
        stop()
        savedRovers.select(id)
        roverDisplayName = savedRovers.rovers.first(where: { $0.id == id })?.name ?? "Terra rover"
        UserDefaults.standard.set(true, forKey: "autoConnectHardware")
        autoConnectHardware()
    }
    func addHardwareRover() {
        stop(); savedRovers.clearSelection()
        roverDisplayName = "New rover"
        UserDefaults.standard.set(true, forKey: "autoConnectHardware")
        autoConnectHardware()
    }
    private func discovered(_ rover: TerraDiscoveredRover, generation: UInt64) {
        guard generation == connectionPolicy.generation else { return }
        roverNames[rover.id] = rover.name
        if rover.id == connectionPolicy.preferred { roverDisplayName = rover.name }
        nearbyPairing.observe(rover, now: connectionNow)
        if !rover.pairing, pairingCandidate?.id == rover.id {
            pairingCandidate = nil; connectionPolicy.decline(rover.id)
        }
        guard let decision = connectionPolicy.discover(rover) else { return }
        switch decision {
        case .offer: pairingCandidate = rover
        case .connect(let id):
            pairingCandidate = nil; connectionPolicy.accept(id, now: connectionNow)
            connectAutomatically(id)
        }
    }
    func respondToPairing(connect: Bool) {
        guard let rover = pairingCandidate else { return }
        pairingCandidate = nil
        if connect {
            connectionPolicy.accept(rover.id, now: connectionNow); connectAutomatically(rover.id)
        } else {
            connectionPolicy.decline(rover.id)
            for candidate in nearbyPairing.available(now: connectionNow) { discovered(candidate, generation: connectionPolicy.generation) }
        }
    }
    private func connectAutomatically(_ id: UUID) {
        let savedPolicy = connectionPolicy
        startBluetooth(identifier: id, feedback: false)
        connectionPolicy = savedPolicy; routerSuppressed = false
    }
    private func bluetoothEvent(_ event: BluetoothLink.Lifecycle, generation: UInt64) {
        guard generation == connectionPolicy.generation, connectionPolicy.enabled else { return }
        switch event {
        case .paired(let id):
            rememberRoverName(id)
            connectionPolicy.paired(id, now: connectionNow)
            bluetooth.startDiscovery(generation: connectionPolicy.generation)
        case .authenticated(let id):
            roverConnectionFailed = false; rememberRoverName(id)
            connectionPolicy.authenticated(id)
        case .closed(let retryable):
            roverConnectionFailed = true
            pairingCandidate = nil
            disconnectDashboardForLifecycle(reason: "bluetooth_closed")
            connectionPolicy.failed(retryable: retryable, now: connectionNow)
        }
    }
    func serviceAutomaticConnections() {
        defer { refreshConnectionPresentation() }
        if connectionPolicy.enabled {
            if let candidate = pairingCandidate,
               !nearbyPairing.available(now: connectionNow).contains(where: { $0.id == candidate.id }) {
                pairingCandidate = nil; connectionPolicy.decline(candidate.id)
            }
            if connectionPolicy.attemptExpired(now: connectionNow) {
                bluetooth.disconnect(); disconnectDashboardForLifecycle(reason: "connection_attempt_expired")
                connectionPolicy.failed(retryable: true, now: connectionNow)
            }
            if connectionPolicy.retryDue(now: connectionNow) {
                bluetooth.startDiscovery(generation: connectionPolicy.generation)
            }
        }
        guard bluetooth.connectedIdentifier != nil, bluetooth.configurationReady,
              !routerSuppressed, !dashboardConnected, !dashboardConnecting else { return }
        if let due = routerRetryAt, connectionNow < due { return }
        let defaults = UserDefaults.standard
        let endpoint = TerraFleetDefaults.savedEndpoint
        guard !endpoint.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty else {
            dashboardStatus = "Set router endpoint in Settings"; return
        }
        routerRetryAt = nil
        connectDashboard(endpoint: endpoint, prefix: defaults.string(forKey: "dashboardTopicPrefix") ?? "terra/phone", roverID: defaults.string(forKey: "dashboardRoverID") ?? "0")
    }
    private func rememberRoverName(_ id: UUID) {
        let name = roverNames[id] ?? savedRovers.rovers.first(where: { $0.id == id })?.name ?? "Terra rover"
        roverDisplayName = name
        savedRovers.remember(id: id, name: name)
    }
    func setHardwareFeedback(enabled: Bool) {
        if enabled {
            guard bluetooth.connectedIdentifier != nil else { return }
            guard feedbackCompatible, hardwareReady, ARWorldTrackingConfiguration.isSupported,
                  motion.isDeviceMotionAvailable else {
                configurationStatus = "Feedback requires a compatible layout and available phone tracking"; return
            }
            stopHardware(); startPhoneSensors(); hardwareFeedback = true
            controlQueue.async { self.bleActive = true; self.bleFeedback = true; self.target = (0, 0); self.resetServoTargets() }
        } else {
            invalidateDashboardAttempt(); dashboardConnecting = false
            sensorEpochLock.lock(); sensorEpoch += 1; sensorEpochLock.unlock()
            motion.stopDeviceMotionUpdates(); session.pause(); stopHardware()
            locationManager.stopUpdatingLocation(); locationManager.stopUpdatingHeading()
            phoneTrackingActive = false
            updateSharedCamera(false)
            hardwareFeedback = false
            controlQueue.async {
                self.bleFeedback = false; self.bleActive = true; self.target = (0, 0)
                self.resetServoTargets(); self.mode = .bluetoothManual; self.startTimer()
            }
            source = "Bluetooth · normalized manual effort"
        }
    }
    private func startBluetooth(identifier: UUID, feedback: Bool) {
        let connectionGeneration = connectionPolicy.generation
        stop()
        requestedHardwareFeedback = feedback ? identifier : nil
        feedbackConnectionSawUnready = false
        sensorEpochLock.lock(); let epoch = sensorEpoch; sensorEpochLock.unlock()
        hardwareActive = true; hardwareFeedback = false
        controlQueue.async {
            guard self.sensorEpochMatches(epoch) else { return }
            self.bleActive = true; self.bleFeedback = false
            self.target = (0, 0); self.resetServoTargets()
            self.mode = .bluetoothManual; self.tick = 0; self.startTimer()
            self.bluetooth.connect(identifier: identifier, generation: connectionGeneration)
            DispatchQueue.main.async { self.source = feedback ? "Bluetooth · waiting for fresh feedback-compatible layout" : "Bluetooth · normalized manual effort" }
        }
    }
    func stopHardware() {
        controlQueue.async { self.revokeHardwareMotion(reason: "output stop requested") }
    }
    func routeHardware(left: Double, right: Double, now: TimeInterval, manualTarget: DriveJoystickCommand? = nil, closedLoop: Bool = false, localDebug: Bool = false, rustFrame: String? = nil) {
        guard bleActive, bleReady, !dashboardEmergencyStop, !bleFeedback || bleCompatible else { return }
        guard phoneMountConfirmed || localDebug else {
            // Report once per hold; neutral output stays latched until the mount is confirmed.
            if !mountHoldReported {
                mountHoldReported = true
                pauseHardwareMotion(reason: Self.mountHoldReason)
            }
            return
        }
        do {
            // Resolve numeric values before erasing their types into Any. Otherwise
            // Swift can infer feedback's neutral literal as Int inside this dictionary.
            let forward: Double = manualTarget?.forward ?? (bleFeedback ? 0 : target.forward)
            let turn: Double = manualTarget?.yaw ?? (bleFeedback ? 0 : target.yaw)
            let input: [String: Any] = ["left_effort": left, "right_effort": right, "forward": forward, "turn": turn, "servo_positions": servoTargets]
            let profile = try ActuatorDriveProfileSet.decode(json: bleLayout)
            let routed = try closedLoop ? profile.routeClosedLoop(left: left, right: right, servos: servoTargets) : profile.route(input: input)
            compareRustFrame(rustFrame, with: routed, now: now)
            let values = String(decoding: try JSONSerialization.data(withJSONObject: routed), as: UTF8.self)
            bluetooth.drive(valuesJSON: values, producedAt: now)
        } catch {
            let detail = "Actuator routing failed: \(error); left=\(left) right=\(right) feedback=\(bleFeedback)"
            TerraLog.control.error("\(detail, privacy: .public)")
            DispatchQueue.main.async { self.configurationStatus = detail }
            revokeHardwareMotion(reason: detail)
        }
    }
}
