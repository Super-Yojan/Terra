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
 var headingAccuracy: Double?
 var originLatitude: Double?
 var originLongitude: Double?
 var anchorX: Double?
 var anchorY: Double?
 var rotation: Double?
}
struct PhoneLocalizationPolicy {
 private var frameID = UUID().uuidString
 var identifier: String { frameID }
 mutating func reset() { frameID = UUID().uuidString }
 mutating func update(_ input: PhoneLocalizationInput) -> PhoneLocalizationReport {
  let fixAge = input.fixTime.flatMap { timestamp -> Double? in
   let age = input.now - timestamp
   return age.isFinite && age >= -1 ? max(0, age) : nil
  }
  let freshFix = fixAge.map { $0 <= 3 } == true
  let accuracyValid = input.accuracy.map { $0.isFinite && $0 >= 0 } == true
  let coordinatesValid = input.latitude.map { $0.isFinite && abs($0) <= 90 } == true
   && input.longitude.map { $0.isFinite && abs($0) <= 180 } == true
  var issues: [String] = []
  if let permission = input.permissionReason { issues.append(permission) }
  if input.fixTime == nil { issues.append("Waiting for GPS fix · move to an open area with a clear view of the sky") }
  else if !freshFix { issues.append("GPS fix stale · waiting for an update within 3 s") }
  if input.fixTime != nil && !accuracyValid { issues.append("GPS accuracy unavailable · waiting for a valid location measurement") }
  if input.fixTime != nil && !coordinatesValid { issues.append("GPS coordinates unavailable or outside the supported latitude/longitude range") }
  let available = issues.isEmpty
  let accuracy = input.accuracy.flatMap { $0.isFinite ? $0 : nil }
  let reason = available
   ? String(format: "GPS available · accuracy %.1f m · fix %.1f s old", locale: Locale(identifier: "en_US_POSIX"), accuracy!, fixAge!)
   : issues.joined(separator: "; ")
  // Raw coordinates are observations only. No north/local transform is inferred.
  return PhoneLocalizationReport(frameID: frameID, mode: available ? "gps" : "local", reason: reason,
   tracking: input.tracked ? "normal" : "limited", issues: issues, gpsAccuracy: accuracy,
   gpsLatitude: available ? input.latitude : nil, gpsLongitude: available ? input.longitude : nil,
   gpsFixAge: fixAge, headingAccuracy: input.headingAccuracy.flatMap { $0.isFinite ? $0 : nil },
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
/// Keep the world offset fixed, but follow terrain elevation and rover pitch/roll
/// with a local supporting plane. Never move world points to flatten the terrain.
struct PhoneGroundPolicy {
 private(set) var offset: Double?
 private var clearance: Double?
 private var candidate: Double?
 private var since: Double?
 mutating func reset() { offset = nil; clearance = nil; candidate = nil; since = nil }
 mutating func observe(height: Double?, cameraHeight: Double, area: Double, now: Double, upZ: Double = 1) -> Double? {
  if let offset { return offset }
  guard let height, height.isFinite, cameraHeight.isFinite, now.isFinite,
        upZ.isFinite, upZ >= 0.5, upZ <= 1.000001,
        area.isFinite, area >= 0.4, cameraHeight-height >= 0.1, cameraHeight-height <= 3 else {
   candidate = nil; since = nil; return nil
  }
  if candidate.map({ abs($0-height) <= 0.03 }) != true { candidate = height; since = now }
  if let since, now-since >= 1 {
   offset = -height
   clearance = (cameraHeight-height) * upZ
  }
  return offset
 }

 /// Use the calibrated rigid camera-to-ground distance and the current body up
 /// axis. Translation up/down a ramp changes this plane, not the world origin.
 /// The rover must remain supported on the terrain; this is not a depth plane fitter.
 func groundPlane(cameraPosition: SIMD3<Double>, bodyUp: SIMD3<Double>) -> PhoneGroundPlane? {
  guard let offset, let clearance,
        cameraPosition.x.isFinite, cameraPosition.y.isFinite, cameraPosition.z.isFinite,
        bodyUp.x.isFinite, bodyUp.y.isFinite, bodyUp.z.isFinite else { return nil }
  let length = hypot(hypot(bodyUp.x, bodyUp.y), bodyUp.z)
  guard length.isFinite, length >= 1e-12 else { return nil }
  let normal = bodyUp / length
  guard normal.z >= 0.5 else { return nil }
  let camera = cameraPosition + SIMD3(0, 0, offset)
  return PhoneGroundPlane(normal: normal,
   offset: normal.x * camera.x + normal.y * camera.y + normal.z * camera.z - clearance)
 }
}
