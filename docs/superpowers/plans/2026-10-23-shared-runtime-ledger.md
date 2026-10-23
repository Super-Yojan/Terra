# Shared runtime execution ledger

Checkpoint: all three remote SHAs verified (see implementation plan).
Baseline: seven shared turning tests and simulator velocity-force test passed;
Zorvane native build/browser check passed. Full workspace tests have obsolete
hardware and target-search fixtures and have not been declared green.
Execution: inline in existing user-approved branches and sibling workspaces.
Ruling: retain existing checkouts rather than isolate into new worktrees — the
approved source paths explicitly link the sibling Terra workspace; cost if wrong:
additional coordination required with concurrent edits. Check status before commits.

Runtime extraction in progress: new pure terra-runtime owns estimator/controller,
map snapshots, command execution admission, navigation and outdoor policies, search,
explicit link/recording evidence, direct effort mixing. Phone Brain wraps Runtime;
its control step now delegates and publication remains outside. Phone 27 unit tests
and first 3 deterministic runtime tests pass. Old fixtures still being migrated.
Zorvane control state now owns Runtime; synthetic sensors ingested in FixedPreUpdate,
shared autonomy in FixedUpdate, forces in FixedPostUpdate (100 Hz). Native check
passes. First full simulator unit run: 81 passed, 2 old ingress/harness failures,
5 ignored. Both failures require migration to bound requests/shared runtime state.
Ruling: MissionObjective and GoalScorer require Sync as well as Send so the runtime
can be stored safely in Bevy ECS components — no unsafe traits; pure implementations
and scorer closure bounds updated. Cost: downstream custom scorers must be Sync.
No migrated application has been deployed; existing binaries/checkpoints untouched.

Ruling: accept equal control timestamps, reject backwards timestamps — the phone
facade already permits a second evaluation after a local request at the same
sensor timestamp. The existing velocity controller still returns neutral for
zero elapsed integration time. Cost if wrong: duplicate evaluations need stricter
API documentation, but never advance integration with fabricated time.

Verification update: Terra workspace passed (including live localhost integrations)
before the final local-dispatch consolidation; the focused mobile/runtime suite
passes after consolidation. Simulator workspace passed with five pre-existing
ignored GPU/local tests. Actual phone-facade versus pure-runtime replay passes.
Added shared ground policy and shared mapping handles; Swift manual motor mixing
removed in favor of runtime efforts. iOS XCFramework generation/build succeeded;
will rebuild latest source and compile the app before deployment. ARGOS full test
migration still in progress: obsolete arm-status/command fixtures replaced by
neutral-readiness and rejection checks, preserving modern stream/stop coverage.
Ruling: browser recorder retains bounded queued events through disconnect and its
local failed flag alone reports recording failure — router loss is not local
storage failure. Cost: a prolonged outage can still fill the existing bounded
queue, which remains an explicit recording failure.

Final fresh-context review: four important findings, all reproduced before fixing:
- Domain rejection was logged as admission success: rejected_domain_requests_do_not_emit_successful_admission RED→GREEN.
- Mapping mount used absolute terrain elevation: translated_terrain_preserves_depth_map RED→GREEN (0, +5, -5 metres).
- Fast transport replacement retained drive authority: runtime_selects_teleop_and_latches_stop_per_rover RED→GREEN; connection generation is separate from subscription epoch.
- Attached map policy missing from configuration identity: attached_map_policy_changes_configuration_identity RED→GREEN.
Additional final regressions: reset_starts_a_new_clock_and_discards_queued_remote_commands,
a_new_recording_does_not_execute_an_old_queued_stop, and settings_observe_the_preceding_motor_output RED→GREEN.
Actual mobile-facade and simulator sensor-adapter replay tests pass. Native versus
WebAssembly replay: 200 control ticks, discrete statuses exact, maximum output
error 0. Final review had no deferred minor findings.
Timing literals for hardware/config admission and IMU/VIO defaults now reference
central named constants, retaining every previous duration. Runtime/config/mode
metadata appears in phone manifests and simulator control/diagnostic records.
Ruling: live physical rollout is not simulated by build/test success — software
release can be prepared and pushed, but neutral-output/controlled drive/fault runs
remain explicitly unverified until performed on the connected rover. Cost if wrong:
a passing software replay cannot establish real sensor calibration or link behavior.

Pre-commit verification: full Terra workspace 312 passed / 0 failed / 3 ignored;
Zorvane 116 passed / 0 failed / 5 ignored; ARGOS 152 passed / 0 failed / 6 ignored.
The two ignored Terra live TCP tests were then run explicitly: both passed.
The ignored production phone-client→simulator motor-force/disconnect test passed.
Swift bindings smoke and signed TerraPhone device build passed. Native Zorvane,
ARGOS workspace builds and browser-bridge WebAssembly build passed. Native and
browser dependency metadata resolves all 12 Terra packages to the one sibling
checkout (no old Git-pinned packages). Source revisions will be embedded again by
post-commit app builds before pushing the matching Zorvane source pin.
No physical motors were exercised. No Pi image change and no hosted Rerun restart.
The post-extraction replay corpus is a reproducible regression reference; it is
not a claim that an exhaustive pre-extraction golden recording was captured.
