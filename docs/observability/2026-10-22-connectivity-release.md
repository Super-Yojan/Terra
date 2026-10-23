# Connectivity and recording release — 22 October 2026

Implemented the approved logging, rover visibility and autonomous router-loss plans. GCP router, collector, TLS, Pi forwarding and `/rerun/` routes are unchanged.

## Evidence

- GPUI's original 51,650 pending events drained to **0**, without clearing its journal. Its journal contains over 68,000 events and remains below its retention quota.
- The reviewed native suites passed **180 tests**, with no failures and three ignored cases (two existing socket tests and a subprocess fixture invoked by its parent test).
- ARGOS core, Zenoh and GPUI suites passed **103 tests**, with no failures. All five affected Swift policy runners passed.
- Native device library, native GPUI and signed iPhone Debug builds succeeded.
- Simulated 120-second router outages preserve accepted local and geographic waypoints and search identity. Exploration survives its first failed-router tick and still expires its budget. Supervised execution completes an approved action and withholds the next approval.
- Real loopback router replacement preserves the accepted goal/run while rejecting a held, never-seen old goal and an old safety reset after a newer emergency stop. Local stale inputs, disarm/hardware loss and emergency-stop checks still take effect.
- GPS tests retain coordinates and original source time at ages 0, 3, 8.2 and 120 seconds. Regression tests cover no-pose reconnect, permission revocation, runtime display overrides, retired ID reuse with old and missing localization, and new frame/run boundaries.

The final signed phone update was installed in place. Its CoreDevice data-container UUID remained `5CDF4270-1553-4AE9-AF4A-69F714A0E8DD`, confirming the update preserved the existing application data. iOS reported Locked during launch attempts; live phone queue-drain verification remains pending. Device live logging and an operator-supervised low-speed physical outage test must not be inferred from simulation results. No rover was armed or driven during this work.

## Decisions and their costs

1. Work in the existing feature checkouts because both apps use cross-repository paths and installed-device build locations; preserve each stage with commits. Cost if wrong: less filesystem isolation from concurrent editing.
2. Reclaim quota in bounded passes of at most 64 evictions, service acknowledgments between passes, and pause new batches while over quota. A committed batch can temporarily exceed the logical quota by one bounded batch plus its explicit loss marker. Cost: bounded temporary logical overshoot; physical SQLite allocation remains separate from logical retention.
3. Keep the writer handle alive until its queued records drain and serialize its retirement with canonical-path registration. Cost: worker lifetime can extend beyond its caller; avoids overlapping writers and lost admitted records.
4. Reader-only operations use read-only connections; HTTP never runs inside storage transactions/control callbacks. Cost: transient local failures retry independently of accepted cloud receipts.
5. Commit storage and accepted-delivery retry together because the shared handle owns their receipt state. Cost: larger review unit; their regressions remain independently exercised.
6. Clear fleet operator intent and pending hardware actions on transport replacement, retaining token replay caches and mission identity. Cost: remote operators must issue fresh commands after reconnect.
7. Keep coordinate-frame identity beyond presentation expiry, invalidate incompatible frame/run/retired placement, and wait for authoritative membership on a replacement client. Cost: a genuinely new run or reused ID may show Position unavailable until new evidence arrives.
8. Add passive presence rows without geographic coordinates. Retain GPS-only previous markers when pose is missing, explicitly disabling pose-dependent control/transform use. Cost: these rows/markers cannot grant control from missing telemetry.
9. Retire the remote authority revision on link loss/replacement and require current run/revision on new remote work and resets. Cost: old unbound operator builds must be updated; stop/cancel/disarm remain usable.
10. Apply display-policy overrides independently of command leases, sensor limits and cached-authority issuance. Cost: a position may remain visible while its telemetry is too old to authorize a new command.
11. Do not induce production disk exhaustion or operate motors for verification. Cost: real disk-full behavior and physical run acceptance remain bounded by automated failure coverage and operator verification.

## Review disposition

The fresh-context review's four Important findings were accepted: delayed command generations, no-pose marker retention, retirement/reused-ID invalidation, and effective display overrides. Each received regression coverage and was fixed. Additional coverage checks now inject actual detector loss and check supervised approval boundaries. No minor findings were deferred. Actual phone launch/queue verification and physical motor behavior remain deployment checks; logging internals retain their earlier dedicated review and tests.

## Deployment receipt

- Native/controller commit: `7c12252`; initial logging commit: `dc1e53d`; phone GPS defaults/report commit: `cec617a`.
- ARGOS final reviewed commit: `aa1c82a`; first fleet-presence commit: `23dd21c`.
- GPUI installed in the existing `/private/tmp/Terra Argos.app` bundle. Built binary SHA-256: `5c4cae31eff98ace1064f4414bd4b5b73e6586edfc6cef1f24ba67d6ae23e1ea`. Installed signed binary SHA-256: `f7706c43407f9584a8b67f8b1a0a7a2d82aedaed042efa16479e9a7a5ce0b3be` (ad-hoc bundle signing changes the signature bytes).
- Phone bundle: `org.terra.robotics.phone`, signed Debug device build, installed successfully without uninstalling the prior app or clearing its data.
- Defaults: GPS display freshness 3 s; fleet stale presentation 90 s; autonomous router loss continues an accepted mission. Motion sensor and manual intent limits are unchanged.
- Existing feature branches retained; no merge, shared-branch push, or infrastructure change is part of this release step.

Final live receipt: GPUI journal **70,157 total / 0 pending** after the final installed build; latest 100 uploader records contain 20 completed uploads and no upload failures. GPUI is open and its subscriber is waiting for rover fleet telemetry. Final phone launch again returned iOS `Locked`; phone live delivery and physical driving remain unverified. Phone lifecycle/UI commit: `71892cd`.
