import OSLog
import ARKit
import AVFoundation
import Combine
import CoreMotion
import CoreLocation
import Foundation
import QuartzCore
import simd

// ARGOS fleet dashboard connection, connection presentation and fleet-managed tracking.
extension PhoneController {
    func connectDashboard(endpoint: String, prefix: String, roverID: String) {
        observation.setContext(roverID: roverID)
        controller?.setObservabilityRover(roverId: roverID)
        observation.record(component: "router", kind: "connection.requested", attributes: [:])
        guard !dashboardConnecting, !dashboardConnected else { return }
        let attempt = dashboardAttemptToken()
        guard let id = UInt64(roverID.trimmingCharacters(in: .whitespacesAndNewlines)) else {
            dashboardStatus = "Enter a non-negative rover ID"; routerSuppressed = true; return
        }
        guard let settings = TerraRouterSettings.parse(endpoint: endpoint, prefix: prefix, roverID: roverID) else {
            dashboardStatus = "Enter a TCP router endpoint and valid topic prefix"; routerSuppressed = true; return
        }
        let endpoint = settings.endpoint, prefix = settings.prefix
        let fleetUser = UserDefaults.standard.string(forKey: "dashboardRouterUser") ?? "terra"
        let fleetPassword = TerraFleetDefaults.password
        dashboardConnecting = true; dashboardStatus = "Connecting…"
        sensorEpochLock.lock(); let epoch = sensorEpoch; sensorEpochLock.unlock()
        controlQueue.async {
            guard self.sensorEpochMatches(epoch), self.dashboardAttemptMatches(attempt) else { return }
            if self.mode == .bluetoothManual, self.controller == nil {
                do { self.controller = try self.makeController() }
                catch {
                    DispatchQueue.main.async {
                        guard self.dashboardAttemptMatches(attempt) else { return }
                        self.dashboardConnecting = false; self.dashboardStatus = error.localizedDescription
                        self.routerRetryAt = self.connectionNow + self.connectionPolicy.nextRouterDelay()
                    }
                    return
                }
            }
            guard (self.mode == .phone || self.mode == .simulation || self.mode == .bluetoothManual), let controller = self.controller else {
                DispatchQueue.main.async {
                    guard self.sensorEpochMatches(epoch), self.dashboardAttemptMatches(attempt) else { return }
                    self.dashboardConnecting = false
                    self.dashboardStatus = "Start Phone IMU + VIO or Bluetooth feedback control first"
                }
                return
            }
            let continuingMission = controller.acceptedOnboardMission()
            // Connecting the router preserves the current local drive session.
            // Network setup must not block local AR and control updates.
            self.routerQueue.async {
            do {
                try controller.reconnectDashboardAuthenticated(endpoint: endpoint, prefix: prefix, roverId: id, user: fleetUser, password: fleetPassword)
                guard self.sensorEpochMatches(epoch), self.dashboardAttemptMatches(attempt) else {
                    try? controller.detachDashboardLink(); return
                }
                DispatchQueue.main.async {
                    guard self.sensorEpochMatches(epoch), self.dashboardAttemptMatches(attempt) else { return }
                    self.observation.record(component: "router", kind: "connection.established", attributes: ["onboard_mission_retained": continuingMission])
                    self.connectionPolicy.routerConnected(); self.routerRetryAt = nil
                    self.dashboardConnecting = false; self.dashboardConnected = true
                    self.dashboardStatus = continuingMission ? "Router restored · onboard mission retained" : "Router connected · rover \(id) · hardware remains stopped"
                    self.fleetTrackingFailed = false
                    self.refreshConnectionPresentation()
                }
            } catch {
                guard self.sensorEpochMatches(epoch), self.dashboardAttemptMatches(attempt) else { return }
                let failure = error as NSError
                self.observation.record(component: "router", kind: "connection.failed", attributes: ["error_domain": failure.domain, "error_code": failure.code])
                DispatchQueue.main.async {
                    guard self.sensorEpochMatches(epoch), self.dashboardAttemptMatches(attempt) else { return }
                    self.dashboardConnecting = false; self.dashboardConnected = false
                    self.dashboardStatus = error.localizedDescription
                    self.routerRetryAt = self.connectionNow + self.connectionPolicy.nextRouterDelay()
                }
            }
            }
        }
    }
    func disconnectDashboard() {
        observation.record(component: "router", kind: "connection.disconnect_requested", attributes: ["source": "operator"])
        routerSuppressed = true; routerRetryAt = nil; invalidateDashboardAttempt()
        controlQueue.async {
            self.closeFpv(publish: true)
            self.target = (0, 0); self.revokeHardwareMotion(reason: "operator disconnected dashboard")
            try? self.controller?.disconnectDashboard()
            DispatchQueue.main.async {
                self.dashboardConnecting = false; self.dashboardConnected = false
                self.dashboardStatus = "Disconnected · remote intent cleared"
                self.waypointActive = false; self.waypointDistance = 0; self.waypointStatus = "No goal"
            }
        }
    }
    func serviceManualDashboard() {
        drainFpvSignaling()
        guard let controller else { return }
        let attempt = dashboardAttemptToken()
        // Preserve the AR/controller clock when switching to manual hardware.
        // Do not repeatedly detach/reset authority once the router is disconnected.

        if TerraManualRouterSafety.emergencyStopped(statusJSON: controller.autonomyBrief()), !dashboardEmergencyStop {
            observation.recordSafetyRequest(reset: false, source: "router_authority", route: "bluetooth")
            dashboardEmergencyStop = true; target = (0, 0); resetServoTargets()
            bluetooth.stopOutput()
            DispatchQueue.main.async { self.autonomyReason = "Router emergency stop · reset explicitly" }
        }
        if controller.dashboardStatus() != "connected" {
            observation.recordChanged(component: "router", kind: "connection.unavailable", attributes: ["status": controller.dashboardStatus(), "mode": "bluetooth_manual"])
            if controller.dashboardStatus() == "failed" { try? controller.handleDashboardLinkLoss() }
            DispatchQueue.main.async {
                guard self.dashboardAttemptMatches(attempt), self.dashboardConnected else { return }
                self.dashboardConnected = false; self.dashboardConnecting = false
                self.dashboardStatus = "Router connection lost · retrying"
                self.routerRetryAt = self.connectionNow + self.connectionPolicy.nextRouterDelay()
            }
        }
    }
    @discardableResult func applyDashboardSettings(endpoint: String, prefix: String, roverID: String) -> Bool {
        guard let settings = TerraRouterSettings.parse(endpoint: endpoint, prefix: prefix, roverID: roverID) else { return false }
        let defaults = UserDefaults.standard
        defaults.set(settings.endpoint, forKey: "dashboardRouterEndpoint")
        defaults.set(settings.prefix, forKey: "dashboardTopicPrefix")
        defaults.set(String(settings.roverID), forKey: "dashboardRoverID")
        routerSuppressed = false; routerRetryAt = nil
        disconnectDashboardForLifecycle(reason: "settings_changed")
        serviceAutomaticConnections(); refreshConnectionPresentation()
        return true
    }
    func retryFleetConnection() {
        routerSuppressed = false; routerRetryAt = nil
        let defaults = UserDefaults.standard
        let endpoint = TerraFleetDefaults.savedEndpoint
        let prefix = defaults.string(forKey: "dashboardTopicPrefix") ?? "terra/phone"
        let id = defaults.string(forKey: "dashboardRoverID") ?? "0"
        connectDashboard(endpoint: endpoint, prefix: prefix, roverID: id)
        refreshConnectionPresentation()
    }
    func refreshConnectionPresentation() {
        let defaults = UserDefaults.standard
        var facts = TerraConnectionFacts()
        facts.radio = bluetooth.radioState
        facts.automatic = connectionPolicy.enabled
        facts.pairingOffered = pairingCandidate != nil
        facts.attempting = connectionPolicy.attempting != nil
        facts.authenticated = bluetooth.connectedIdentifier != nil
        facts.configurationAvailable = hardwareConfigurationReady
        facts.hardwareReady = hardwareReady
        facts.hardwareFault = hardwareFault != nil
        facts.roverFailed = roverConnectionFailed
        facts.fleetConfigured = TerraRouterSettings.parse(endpoint: TerraFleetDefaults.savedEndpoint, prefix: defaults.string(forKey: "dashboardTopicPrefix") ?? "terra/phone", roverID: defaults.string(forKey: "dashboardRoverID") ?? "0") != nil
        facts.fleetAttempting = dashboardConnecting; facts.fleetConnected = dashboardConnected
        facts.fleetPaused = routerSuppressed; facts.fleetFailed = routerRetryAt != nil
        #if targetEnvironment(simulator)
        facts.simulator = true
        #endif
        synchronizeFleetTracking(facts)
        #if DEBUG && targetEnvironment(simulator)
        // Explicit simulator screenshot fixtures never run on a physical device.
        if let preview = ProcessInfo.processInfo.environment["TERRA_UI_PREVIEW_STATE"] {
            facts = TerraConnectionFacts()
            facts.radio = .ready; facts.automatic = true
            roverDisplayName = "Preview rover"
            switch preview {
            case "connecting": facts.attempting = true; facts.fleetConfigured = true
            case "connected", "attention", "retrying":
                facts.authenticated = true; facts.configurationAvailable = true
                facts.hardwareReady = preview != "attention"; facts.hardwareFault = preview == "attention"
                facts.fleetConfigured = true; facts.fleetConnected = preview == "connected"
                facts.fleetFailed = preview == "retrying"
            default: break
            }
        }
        #endif
        connectionFacts = facts
    }
    func invalidateDashboardAttempt() {
        dashboardGenerationLock.lock(); dashboardGeneration &+= 1; dashboardGenerationLock.unlock()
    }
    private func dashboardAttemptToken() -> UInt64 {
        dashboardGenerationLock.lock(); defer { dashboardGenerationLock.unlock() }; return dashboardGeneration
    }
    private func dashboardAttemptMatches(_ token: UInt64) -> Bool { dashboardAttemptToken() == token }
    func disconnectDashboardForLifecycle(reason: String) {
        observation.record(component: "router", kind: "connection.disconnect_requested", attributes: ["source": "connection_policy", "reason": reason])
        invalidateDashboardAttempt()
        controlQueue.async {
            self.closeFpv(publish: true)
            try? self.controller?.disconnectDashboard()
        }
        dashboardConnected = false; dashboardConnecting = false
        dashboardStatus = "Disconnected"
    }
    func retryPhoneTracking() {
        fleetTrackingFailed = false
        refreshConnectionPresentation()
    }
    private func synchronizeFleetTracking(_ facts: TerraConnectionFacts) {
        let retainingLocalSession = fleetTrackingManaged && facts.canRetainTrackingDuringRouterRetry && !routerSuppressed
        if facts.shouldTrackForFleet || retainingLocalSession {
            fleetTrackingManaged = true
            let feedback = feedbackCompatible && hardwareReady
            if phoneTrackingActive {
                if hardwareFeedback != feedback {
                    stopHardware(); hardwareFeedback = feedback
                    controlQueue.async {
                        self.bleFeedback = feedback; self.target = (0, 0); self.resetServoTargets()
                    }
                }
                return
            }
            guard !fleetTrackingFailed else { return }
            let permission = AVCaptureDevice.authorizationStatus(for: .video)
            guard permission != .denied, permission != .restricted else {
                phoneTrackingStatus = "Allow camera access in iPhone Settings to start tracking"
                return
            }
            guard ARWorldTrackingConfiguration.isSupported, motion.isDeviceMotionAvailable else {
                phoneTrackingStatus = "Phone tracking is unavailable on this device"
                return
            }
            stopHardware()
            startPhoneSensors()
            hardwareFeedback = feedback
            phoneTrackingStatus = "Tracking starting · managed by ARGOS in every mode"
            controlQueue.async {
                self.bleActive = true; self.bleFeedback = feedback
                self.target = (0, 0); self.resetServoTargets()
            }
        } else if fleetTrackingManaged {
            fleetTrackingManaged = false; fleetTrackingFailed = false
            setHardwareFeedback(enabled: false)
            phoneTrackingStatus = "Tracking starts after ARGOS connects"
        }
    }
}
