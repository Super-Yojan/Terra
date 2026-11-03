import OSLog
import ARKit
import AVFoundation
import Combine
import CoreMotion
import CoreLocation
import Foundation
import QuartzCore
import simd

// CoreLocation callbacks and localization publication to the fleet.
extension PhoneController {
    func locationManagerDidChangeAuthorization(_ manager: CLLocationManager) {
        let reason: String?
        switch manager.authorizationStatus {
        case .denied: reason = "Location permission denied · allow Terra in Settings"
        case .restricted: reason = "Location permission restricted"
        case .notDetermined: reason = "Waiting for location permission"
        default: reason = nil // Approximate GPS is displayed with its reported accuracy.
        }
        TerraLog.tracking.notice("Location authorization: \(reason ?? "Location access allowed", privacy: .public)")
        controlQueue.async {
            self.locationPermissionReason = reason
            if manager.authorizationStatus == .denied || manager.authorizationStatus == .restricted {
                try? self.controller?.invalidateOutdoorSensors()
            }
        }
    }
    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        guard let fix = locations.last else { return }
        let delivered = CACurrentMediaTime(), sensed = SensorTimingProbe.mediaTime(of: fix.timestamp)
        controlQueue.async {
            SensorTimingProbe.shared.record(.gps, sensor: sensed, os: delivered, queue: CACurrentMediaTime(), a: fix.horizontalAccuracy, b: fix.speed)
            if self.mode == .phone { self.locationFix = fix }
        }
    }
    func locationManager(_ manager: CLLocationManager, didUpdateHeading heading: CLHeading) {
        let delivered = CACurrentMediaTime(), sensed = SensorTimingProbe.mediaTime(of: heading.timestamp)
        controlQueue.async {
            SensorTimingProbe.shared.record(.heading, sensor: sensed, os: delivered, queue: CACurrentMediaTime(), a: heading.headingAccuracy)
            if self.mode == .phone { self.locationHeading = heading }
        }
    }
    func locationManager(_ manager: CLLocationManager, didFailWithError error: Error) {
        let detail = error.localizedDescription
        TerraLog.tracking.error("Location error: \(detail, privacy: .public)")
        // Temporary acquisition failures do not erase a previously measured fix.
        // Its original timestamp still enforces the freshness limit.
        if (error as? CLError)?.code == .denied {
            controlQueue.async { self.locationFix = nil; self.locationHeading = nil; self.locationPermissionReason = "Location permission denied"; try? self.controller?.invalidateOutdoorSensors() }
        }
    }
    func publishLocalization(_ controller: MobileController) {
        let now = Date().timeIntervalSince1970
        let monotonic = CACurrentMediaTime()
        try? controller.enableOutdoorGps()
        if let fix = locationFix, outdoorFixTime != fix.timestamp {
            outdoorFixTime = fix.timestamp
            try? controller.pushGps(sample: GpsFix(timestamp: monotonic - (now - fix.timestamp.timeIntervalSince1970), latitude: fix.coordinate.latitude, longitude: fix.coordinate.longitude, accuracy: fix.horizontalAccuracy), now: monotonic)
            // Course is travel direction; the core pairs it with AR travel, so no mount rotation here.
            if fix.course >= 0, fix.courseAccuracy >= 0, fix.speed >= 0 {
                try? controller.pushCourse(sample: CourseFix(timestamp: monotonic - (now - fix.timestamp.timeIntervalSince1970), course: fix.course, accuracy: fix.courseAccuracy, speed: fix.speed), now: monotonic)
            }
        }
        if let heading = locationHeading, outdoorHeadingTime != heading.timestamp,
           let reference = phoneHeadingReferenceYaw, let pose = phonePose,
           previousPoseTime.map({ monotonic - $0 >= 0 && monotonic - $0 <= 0.35 }) == true {
            outdoorHeadingTime = heading.timestamp
            // Pair compass phone-right reference with that same AR axis, then rotate to rover forward.
            let corrected = PhoneMountGeometry.roverTrueHeading(compass: heading.trueHeading, roverYaw: pose.yaw, referenceYaw: reference)
            try? controller.pushHeading(sample: HeadingFix(timestamp: monotonic - (now - heading.timestamp.timeIntervalSince1970), trueHeading: corrected, accuracy: heading.headingAccuracy), now: monotonic)
        }
        guard now - lastLocalizationPublish >= 1 else { return }
        lastLocalizationPublish = now
        let poseFresh = previousPoseTime.map { CACurrentMediaTime() - $0 < 1 } == true
        let decoder = JSONDecoder(); decoder.keyDecodingStrategy = .convertFromSnakeCase
        if let policy = try? decoder.decode(PhoneConnectivityPolicy.self, from: Data(controller.connectivityPolicySettings().utf8)) {
            localizationPolicy.setDisplayFreshnessSeconds(policy.gpsDisplayFreshnessSeconds)
        }
        let report = localizationPolicy.update(PhoneLocalizationInput(now: now,
            latitude: locationFix?.coordinate.latitude, longitude: locationFix?.coordinate.longitude,
            fixTime: locationFix?.timestamp.timeIntervalSince1970, accuracy: locationFix?.horizontalAccuracy,
            heading: locationHeading?.trueHeading, headingAccuracy: locationHeading?.headingAccuracy,
            headingTime: locationHeading?.timestamp.timeIntervalSince1970,
            localX: phonePose?.x ?? 0, localY: phonePose?.y ?? 0, headingReferenceYaw: phoneHeadingReferenceYaw, tracked: localizationTracked && poseFresh,
            permissionReason: !CLLocationManager.locationServicesEnabled() ? "Location Services are off · enable them in iPhone Settings" : locationPermissionReason,
            trackingReason: localizationTrackingReason ?? (!poseFresh ? "AR tracking pose stale · waiting for a fresh camera position" : nil)))
        sensorEpochLock.lock(); let reportEpoch = sensorEpoch; sensorEpochLock.unlock()
        DispatchQueue.main.async {
            guard self.sensorEpochMatches(reportEpoch), self.phoneTrackingActive else { return }
            self.localizationReport = report
        }
        if report.reason != lastLocalizationReason {
            lastLocalizationReason = report.reason
            TerraLog.tracking.notice("Localization: \(report.reason, privacy: .public); GPS accuracy=\(report.gpsAccuracy ?? -1) age=\(report.gpsFixAge ?? -1); heading accuracy=\(report.headingAccuracy ?? -1)")
        }
        if let data = try? JSONEncoder().encode(report), let json = String(data: data, encoding: .utf8) {
            try? controller.publishLocalization(payload: json)
        }
    }
}
