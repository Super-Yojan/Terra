// Source-level lifecycle guard: PhoneController depends on iOS/ARKit, so this
// regression checks its real mutation sites without mocking a separate counter.
// Run from the repository root: swift apps/ios/Tests/PhoneMapPublicationTests.swift
import Foundation

// The class body declares the counter; the mapping extension advances and publishes it.
let source = try [
    "apps/ios/TerraPhone/Phone/PhoneController.swift",
    "apps/ios/TerraPhone/Map/PhoneController+Mapping.swift",
].map { try String(contentsOfFile: $0, encoding: .utf8) }.joined(separator: "\n")
let uses = source.components(separatedBy: .newlines).filter {
    $0.contains("occupancyPublicationRevision")
}
precondition(uses.count == 3, "Publication revision must have only initialization, increment and publication use")
precondition(uses[0].contains("var occupancyPublicationRevision: UInt64 = 0"))
precondition(uses[1].contains("occupancyPublicationRevision += 1"))
precondition(uses[2].contains("revision:occupancyPublicationRevision"))
precondition(!source.contains("revision:UInt64(tick)"), "Map identity must not use resettable control tick")
let prepareStart = source.range(of: "func prepare(")!.lowerBound
let prepareEnd = source.range(of: "func startTimer(", range: prepareStart..<source.endIndex)!.lowerBound
let prepare = source[prepareStart..<prepareEnd]
precondition(prepare.contains("preserveRuntime") && prepare.contains("tick = 0"))
precondition(!prepare.contains("occupancyPublicationRevision"), "prepare must preserve map publication identity")
let publishStart = source.range(of: "func publishMap(")!.lowerBound
let publishEnd = source.range(of: "func updateSimulatedMap(", range: publishStart..<source.endIndex)!.lowerBound
let publish = source[publishStart..<publishEnd]
precondition(publish.contains(uses[1]) && publish.contains(uses[2]))
print("Phone map publication: revision survives prepare/preserveRuntime and advances only at publication")
