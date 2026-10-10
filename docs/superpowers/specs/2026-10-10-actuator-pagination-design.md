# Actuator configuration pagination

Date: 2026-10-10
Status: Implemented and reviewed; software verification passed. Physical Bluetooth verification remains pending.

## Outcome

TerraPhone reads and writes one actuator's configuration per request. Opening the configuration screen must not download the complete layout or calibration for every actuator. The rover retains its complete persisted layout and validates all output assignments before applying changes. This replaces oversized configuration transfers that can exceed the existing 100 ms assembly deadline.

## Source layout and scope

Update the Python rover protocol, configuration store, BLE transport and tests; the Swift Bluetooth link, controller, actuator models and Views editor; and Rust mobile actuator helpers when needed for compact runtime descriptors. Update protocol documentation and shared fixtures. No changes to ownership, pairing, motor watchdog timing, emergency stop, or hardware interlock policy.

## Protocol discovery and revision consistency

Keep the existing control envelope and request/reply correlation. Advertise `actuator_pagination_v1` through capabilities. New TerraPhone requires this capability for paginated editing and shows a rover-update message when absent rather than silently fetching a complete layout.

All reads include `expected_revision`; every response includes its revision. A mismatch returns `stale_revision`. A missing layout is represented by `layout_available: false`, revision 0, and an empty actuator list. No cached actuator may be associated with a different layout revision or Bluetooth connection generation.

## Read operations

- `layout_index`, empty payload: return layout availability, active revision, actuator count and ordered `{id, name, kind}` entries. Maximum 16 entries; no calibration, limits, routes or complete actuator objects.
- `read_actuator`, `{expected_revision, actuator_id}`: return one complete active actuator, or `actuator_not_found`. The editor requests it only when selected and caches it for that revision.
- `read_drive_profile`, `{expected_revision, actuator_id}`: return one compact runtime descriptor containing ID, kind, limits, route, and safe output. Exclude calibration, port assignments, and descriptive metadata.

No new read response contains a full actuator array. The index contains summaries only. Requests are serialized and correlated; loading another page never replaces an in-progress actuator edit.

## Driving synchronization

Driving currently depends on complete layout JSON for routing, safe frames, servo ranges and feedback compatibility. Replace that dependency with runtime descriptors. At owner admission, fetch the index and then one runtime descriptor per actuator sequentially. Synchronization is complete only after all descriptors match one active revision and pass shape/ID/route/limit/safe-output checks. Block arming during incomplete synchronization. Abort and restart on revision or connection changes.

Build routing, safe frames and feedback compatibility from descriptors directly. Do not manufacture fake calibration/port values to satisfy the old layout validator. The rover remains responsible for calibrations and hardware validity. The phone can store all compact descriptors for drive routing; this is distinct from eagerly loading editable configuration.

## Rover-side draft operations

One bounded draft exists for the admitted owner connection. It is a deep copy of the active layout, or an empty revision-0 layout when none exists. Its opaque token and version identify the exact draft. Disconnect, explicit discard, or an active revision change invalidates it.

- `begin_layout_edit`, `{expected_revision, mode}`: mode `existing` copies the active layout; mode `replace` starts empty for presets. Return token, base revision and version 0.
- `stage_actuator`, `{edit_token, edit_version, actuator}`: insert or replace exactly one complete actuator, validate its schema and per-actuator values, advance draft version, and return ID/version only.
- `remove_actuator`, `{edit_token, edit_version, actuator_id}`: remove one actuator, advance version, and return ID/version only.
- `validate_layout_edit`, `{edit_token, edit_version}`: validate the whole draft against capabilities, resource conflicts, IDs, limits and safety rules. Return acknowledgment of the exact token/version/base revision, or bounded validation errors.
- `commit_layout_edit`, `{edit_token, edit_version, base_revision}`: require the previously acknowledged exact draft, unchanged active revision, disarmed state and all current configuration gates. Revalidate, apply using the existing persistence/backend reconciliation contract, advance revision once and return new revision only.
- `discard_layout_edit`, `{edit_token}`: drop the draft; active configuration is unchanged.

Changing any draft entry invalidates validation acknowledgment. Reject wrong tokens/versions without mutation. An unsuccessful actuator update leaves the previous draft version intact. A commit consumes its draft even on persistence failure; existing fault/reconciliation behavior applies. Request replay retains the existing response cache semantics so a lost acknowledgment cannot increment revision twice. Session changes invalidate old acknowledgments.

Whole-layout port conflicts can be temporarily present while editing, but block validation and commit. Active outputs are never reconfigured during staging. Presets are sent as individually acknowledged actuator updates into a replacement draft, never as a full-layout payload.

## Phone editor

Show the index as an actuator list and a selected-actuator detail editor. On selection fetch only that actuator's full configuration. Provide loading, error and explicit retry states. Keep unsent changes visible; switching actuator asks the user to save to the draft or discard those local changes.

Save sends one actuator to the rover draft. Add and Remove update the draft individually. Keep a local draft index reflecting acknowledged updates; it does not require loading untouched actuator details. Presets use the replacement flow above.

Validate checks the entire rover-side draft, then Apply commits the acknowledged version. Display conflicts by actuator/port without fetching unrelated actuator configurations. After commit invalidate edited-object caches and old runtime descriptors, fetch the new index and runtime profiles sequentially, and retain disarmed state. No automatic arm follows a configuration commit.

## Transport deadlines and bounds

Pagination reduces payload sizes but one actuator still spans multiple ATT fragments. Use a configurable per-assembler timeout: 100 ms for drive and binary priority commands, 2 seconds total for configuration JSON. Route-table expiry must match the selected assembler, not use a global 100 ms value. Maintain clock-regression rejection, ordered fragments, size/count limits and bounded concurrent routes.

Give configuration requests a 5-second acknowledgment deadline from the first fragment through the correlated reply, including reply reassembly. Keep the existing 2-second per-write acknowledgment deadline. Do not extend drive freshness (100 ms), motor watchdog (200 ms), or motion-status freshness (300 ms). Keep status notifications compact.

Bound each paginated request/reply to 4080 bytes (within the 4096-byte budget). At conservative 20-byte ATT capacity, 16 payload bytes per fragment and 255 fragments carry at most 4080 bytes. Bound error reports, draft count (one), actuator count (16), response cache and request queue. Unsupported/expired transfers return a clear configuration error and permit retry; log operation, request ID and timing without full payloads.

## Compatibility and deployment

Preserve legacy `read_layout`, `stage_layout`, and `commit_layout` for older clients initially; accepting one workflow invalidates a pending draft from the other workflow. The new app uses only paginated configuration operations. Capability-negotiated runtime synchronization prevents new phones from treating older rovers as ready.

Build/test the updated ARM64 rover in Docker. Install the rover update before the new app, preserving owner/name/layout files. Deployment is a separate step after software verification. No owner reset or live test motor movement is required.

## Verification

Python tests: index contains no full configurations; each read/write contains at most one actuator; revision/token/version mismatch rejection; add/remove/preset flow; validation conflict detection; draft invalidation on disconnect; gate and armed-state rejection; atomic commit and rollback; replay of commit; no full-layout commit reply; legacy/pagination isolation.

Transport tests: configuration fragments arriving after 100 ms but before 2 seconds succeed; drive and priority fragments still expire at 100 ms; expired configuration/routes are reclaimed; oversized messages and clock regression fail; bounded replies fit conservative ATT capacity.

Swift/Rust tests: compact profiles produce equivalent safe values and routing for motors, ESCs, servos and mixed layouts; missing/duplicate/invalid profiles block arm; no eager editable-configuration fetch; page errors preserve local edits; revision/generation invalidation; acknowledgment correlation; presets transfer sequentially; apply never arms.

Run rover tests, relevant Rust/Swift tests and the iPhone build. Docker bundle checks remain required. Physical acceptance: open two actuator pages, edit/save each independently, validate/apply, reconnect and verify persistence, and induce a timeout to confirm a visible retryable error without a service restart.
