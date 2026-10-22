import Foundation
@main struct LocalizationTests {
 static func main() {
  func input(_ now: Double) -> PhoneLocalizationInput {
   PhoneLocalizationInput(now: now, latitude: 38, longitude: -77, fixTime: now, accuracy: 4, heading: nil, headingAccuracy: nil, headingTime: nil, localX: .nan, localY: .nan, headingReferenceYaw: nil, tracked: false)
  }
  var policy = PhoneLocalizationPolicy(displayFreshnessSeconds: 3)
  var failures = 0
  func expect(_ condition: Bool, _ message: String) {
   if !condition { print("FAIL: " + message); failures += 1 }
  }
  let fix = policy.update(input(0))
  expect(fix.mode == "gps", "publish raw GPS immediately without AR, compass or eight-second alignment")
  expect(fix.issues.isEmpty, "compass and AR failures do not block raw GPS")
  expect(fix.gpsLatitude == 38 && fix.gpsLongitude == -77, "preserve measured coordinates")
  expect(fix.originLatitude == nil && fix.rotation == nil && fix.anchorX == nil, "raw GPS must not invent a local transform")
  var coarse = input(1); coarse.accuracy = 45
  expect(policy.update(coarse).mode == "gps", "show coarse GPS with its reported uncertainty")
  expect(policy.update(coarse).reason.contains("45"), "display measured GPS accuracy")
  var stale = input(10); stale.fixTime = 6
  let old = policy.update(stale)
  expect(old.mode == "local" && old.gpsLatitude == 38, "retain stale observations without presenting them as live GPS")
  for age in [0.0, 3, 8.2, 120] {
   var measured = input(200); measured.fixTime = 200-age
   let report = policy.update(measured)
   expect(report.gpsLatitude == 38 && report.gpsLongitude == -77, "retain coordinates at every age")
   expect(report.gpsObservedAt == 200-age, "retain original observation time")
   expect(report.gpsFresh == (age <= 3), "freshness is a label only")
  }
  var relaxed = PhoneLocalizationPolicy(displayFreshnessSeconds: 10)
  expect(relaxed.update(stale).gpsFresh, "display policy is injectable")
  relaxed.setDisplayFreshnessSeconds(3)
  expect(!relaxed.update(stale).gpsFresh, "runtime updates change freshness without clearing observations")
  expect(old.reason.contains("stale"), "explain stale GPS")
  var denied = input(11); denied.permissionReason = "Location permission denied"
  expect(policy.update(denied).reason.contains("permission denied"), "show location permission errors")
  expect(policy.update(denied).gpsLatitude == nil, "permission failures cannot advertise a live position")
  for value in [-1.0, Double.nan, Double.infinity] {
   var bad = input(12); bad.accuracy = value
   let report = policy.update(bad)
   expect(report.mode == "local" && report.gpsLatitude == nil, "invalid accuracy cannot produce a live position")
   expect((try? JSONEncoder().encode(report)) != nil, "bad sensor values must not suppress diagnostic publication")
  }
  var badCoordinate = input(13); badCoordinate.latitude = .nan
  expect(policy.update(badCoordinate).gpsLatitude == nil, "invalid GPS coordinates cannot become a map position")
  var missing = input(14); missing.fixTime = nil
  expect(policy.update(missing).reason.contains("Waiting for GPS"), "explain missing GPS")
  let identifier = policy.identifier; policy.reset()
  expect(policy.identifier != identifier, "tracking restart still changes the local frame ID")
  expect(policy.update(input(15)).mode == "gps", "restarting tracking does not introduce an alignment wait")
  if failures > 0 { exit(1) }
  print("Raw GPS: immediate publication, no alignment, accuracy, freshness and invalid fixes passed")
 }
}
