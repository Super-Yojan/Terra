// Source-level lifecycle guard: PhoneController depends on iOS/ARKit, so this
// regression checks its real mutation sites without mocking a separate counter.
// Run from the repository root: swift tests/ios/PhoneMapPublicationTests.swift
import Foundation

let source = try String(contentsOfFile: "mobile/ios/TerraPhone/PhoneController.swift", encoding: .utf8)
let uses = source.components(separatedBy: .newlines).filter {
    $0.contains("occupancyPublicationRevision")
}
precondition(uses.count == 3, "Publication revision must have only initialization, increment and publication use")
precondition(uses[0].contains("private var occupancyPublicationRevision: UInt64 = 0"))
precondition(uses[1].contains("occupancyPublicationRevision += 1"))
precondition(uses[2].contains("revision:occupancyPublicationRevision"))
precondition(!source.contains("revision:UInt64(tick)"), "Map identity must not use resettable control tick")
let prepareStart = source.range(of: "private func prepare(")!.lowerBound
let prepareEnd = source.range(of: "private func startTimer(", range: prepareStart..<source.endIndex)!.lowerBound
let prepare = source[prepareStart..<prepareEnd]
precondition(prepare.contains("preserveRuntime") && prepare.contains("tick = 0"))
precondition(!prepare.contains("occupancyPublicationRevision"), "prepare must preserve map publication identity")
let publishStart = source.range(of: "private func publishMap(")!.lowerBound
let publishEnd = source.range(of: "private func updateSimulatedMap(", range: publishStart..<source.endIndex)!.lowerBound
let publish = source[publishStart..<publishEnd]
precondition(publish.contains(uses[1]) && publish.contains(uses[2]))
print("Phone map publication: revision survives prepare/preserveRuntime and advances only at publication")
