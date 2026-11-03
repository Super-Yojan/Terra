import Foundation
struct PhoneLocalizationInput {
 var now: Double
 var latitude: Double?
 var longitude: Double?
 var fixTime: Double?
 var accuracy: Double?
 var heading: Double?
 var headingAccuracy: Double?
 var headingTime: Double?
 var localX: Double
 var localY: Double
 var headingReferenceYaw: Double?
 var tracked: Bool
 var permissionReason: String? = nil
 var trackingReason: String? = nil
}
struct PhoneLocalizationReport: Codable {
 var version = 1
 var frameID: String
 var mode: String
 var reason: String
 var tracking: String
 var issues: [String] = []
 var gpsAccuracy: Double?
 var gpsLatitude: Double?
 var gpsLongitude: Double?
 var gpsFixAge: Double?
 var gpsObservedAt: Double?
 var gpsFresh: Bool
 var headingAccuracy: Double?
 var originLatitude: Double?
 var originLongitude: Double?
 var anchorX: Double?
 var anchorY: Double?
 var rotation: Double?
}
struct PhoneConnectivityPolicy: Decodable {
 var gpsDisplayFreshnessSeconds: Double
 var fleetDisplayStaleSeconds: Double
 var autonomousRouterLoss: String
}
struct PhoneLocalizationPolicy {
 private var displayFreshnessSeconds: Double
 init(displayFreshnessSeconds: Double) {
  precondition(displayFreshnessSeconds.isFinite && displayFreshnessSeconds > 0)
  self.displayFreshnessSeconds = displayFreshnessSeconds
 }
 private var frameID = UUID().uuidString
 mutating func setDisplayFreshnessSeconds(_ value: Double) {
  guard value.isFinite && value > 0 else { return }
  displayFreshnessSeconds = value
 }
 var identifier: String { frameID }
 mutating func reset() { frameID = UUID().uuidString }
 mutating func update(_ input: PhoneLocalizationInput) -> PhoneLocalizationReport {
  let fixAge = input.fixTime.flatMap { timestamp -> Double? in
   let age = input.now - timestamp
   return age.isFinite && age >= -1 ? max(0, age) : nil
  }
  let freshFix = fixAge.map { $0 <= displayFreshnessSeconds } == true
  let accuracyValid = input.accuracy.map { $0.isFinite && $0 >= 0 } == true
  let coordinatesValid = input.latitude.map { $0.isFinite && abs($0) <= 90 } == true
   && input.longitude.map { $0.isFinite && abs($0) <= 180 } == true
  var issues: [String] = []
  if let permission = input.permissionReason { issues.append(permission) }
  if input.fixTime == nil { issues.append("Waiting for GPS fix · move to an open area with a clear view of the sky") }
  else if !freshFix { issues.append(String(format: "GPS fix stale · last known position · display freshness %.1f s", displayFreshnessSeconds)) }
  if input.fixTime != nil && !accuracyValid { issues.append("GPS accuracy unavailable · waiting for a valid location measurement") }
  if input.fixTime != nil && !coordinatesValid { issues.append("GPS coordinates unavailable or outside the supported latitude/longitude range") }
  let measurementValid = input.permissionReason == nil && fixAge != nil && accuracyValid && coordinatesValid
  let available = measurementValid && freshFix
  let accuracy = input.accuracy.flatMap { $0.isFinite ? $0 : nil }
  let reason = available
   ? String(format: "GPS available · accuracy %.1f m · fix %.1f s old", locale: Locale(identifier: "en_US_POSIX"), accuracy!, fixAge!)
   : issues.joined(separator: "; ")
  // Raw coordinates are observations only. No north/local transform is inferred.
  return PhoneLocalizationReport(frameID: frameID, mode: available ? "gps" : "local", reason: reason,
   tracking: input.tracked ? "normal" : "limited", issues: issues, gpsAccuracy: accuracy,
   gpsLatitude: measurementValid ? input.latitude : nil, gpsLongitude: measurementValid ? input.longitude : nil,
   gpsFixAge: fixAge, gpsObservedAt: measurementValid ? input.fixTime : nil, gpsFresh: available, headingAccuracy: input.headingAccuracy.flatMap { $0.isFinite ? $0 : nil },
   originLatitude: nil, originLongitude: nil, anchorX: nil, anchorY: nil, rotation: nil)
 }

}

/// A local supporting plane in the fixed, Z-up mapping world frame.
/// Signed height above ground is `normal · point - offset`, in metres.
struct PhoneGroundPlane {
 var normal: SIMD3<Double>
 var offset: Double
}

/// Calibrate the world origin and camera clearance from a stable measured floor.
/// Keep the world offset fixed and follow terrain elevation with a level plane
/// below the camera. Never move world points to flatten the terrain.
struct PhoneGroundPolicy {
 private let core = MobileGroundPolicy()
 var offset: Double? { core.offset() }
 var clearance: Double? { core.clearance() }
 mutating func reset() { core.reset() }
 mutating func observe(height: Double?, cameraHeight: Double, area: Double, now: Double, upZ: Double = 1) -> Double? {
  core.observe(height: height, cameraHeight: cameraHeight, area: area, now: now, upZ: upZ)
 }
 mutating func mount(clearance: Double, cameraHeight: Double) -> Double? {
  core.mount(clearance: clearance, cameraHeight: cameraHeight)
 }
 func groundPlane(cameraPosition: SIMD3<Double>, bodyUp: SIMD3<Double>) -> PhoneGroundPlane? {
  guard let value = core.plane(x: cameraPosition.x, y: cameraPosition.y, z: cameraPosition.z, upX: bodyUp.x, upY: bodyUp.y, upZ: bodyUp.z) else { return nil }
  return PhoneGroundPlane(normal: SIMD3(value.normalX, value.normalY, value.normalZ), offset: value.offset)
 }
}
