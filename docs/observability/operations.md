# Operational history and Rerun

The observer is independent of drive control and the experiment recorder. It
records normal events, transitions, command admission, faults and sampled state.
It does not change timeouts, arming, safety decisions, Zenoh routing or TLS.
See [coverage](coverage.md) and [event schema](event-schema.md) for exact scope.

## Status and deployment boundary

The GCP collector was deployed and verified on 2026-10-21. See the
[deployment record](deployments/2026-10-21-gcp.md) for the host, limits and checks.
This Mac's ARGOS and Zorvane upload settings are provisioned; relaunch the rebuilt
apps to enable uploads. Phone and Pi direct logging are provisioned; physical driving acceptance
remains outstanding. See the direct-device deployment record for USB and HAT checks. The deployment verification was synthetic, not a field run.
The existing router remains on TLS port 7448. The new collector listens only on
127.0.0.1:7450 behind `/observability/*` on the existing HTTPS host. It has its
own user, database, limits and credentials. No cloud service-account key is
needed by a phone, Pi or dashboard.

## Recording configuration

Native sources open their journals at startup. On macOS the default root is
`~/Library/Application Support/Terra/Observability/<role>`; Linux uses
`$XDG_STATE_HOME/terra/observability/<role>` or `~/.local/state/terra/observability/<role>`.
An adjacent `.producer` file keeps the source identity stable across restarts.
Do not delete it during an upgrade. Event boot IDs change at process startup;
old queued events retain their old identity and rover/session.

Set these native process environment variables before launch:

```text
TERRA_OBSERVABILITY_DB=/private/state/events.db
TERRA_OBSERVABILITY_PRODUCER=dashboard-laptop
TERRA_OBSERVABILITY_SETTINGS=/private/config/source-settings.json
TERRA_OBSERVABILITY_ENDPOINT=https://HOST/observability
TERRA_OBSERVABILITY_UPLOAD_TOKEN_FILE=/private/config/dashboard-laptop.token
```

Native apps also read a private `upload.json` beside `events.db` when neither
upload environment variable is set. Its fields are `endpoint` and `token_file`;
a relative token path resolves beside that file. This makes normal app launches
upload automatically after provisioning, without adding credentials to launch
arguments. Keep the directory mode0700 and both config/token files mode0600.
Phone imports use the separate Keychain procedure below.

Only the first three environment variables are needed for local recording; a missing upload endpoint
keeps events local. `TERRA_OBSERVABILITY_DISABLED=1` disables native recording.
Copy [source-settings.example.json](source-settings.example.json) to change
recording rates and budgets in one place. Partial overrides are accepted;
unknown keys or invalid values reject observer setup without changing control.
These are diagnostic rates, not safety timeout or control-loop settings.

Defaults: source events 256 MiB, attachments 512 MiB, event retention seven days;
control summaries 10 Hz, map snapshots 2 Hz, camera/depth evidence 1 Hz. Pi
events default to 512 MiB. Events and retained evidence are sampled continuously,
so a normal uninterrupted recording contains the requested ten seconds before
and after a trigger without waiting for the trigger to begin recording.
Startup, process death, missing sensors, queue overflow or quota pressure can
leave gaps; no ten-second window is guaranteed in those conditions. Attachments
use a separate oldest-first byte budget, not an additional seven-day expiry.

Admission to a bounded memory queue is not disk durability. The worker commits
SQLite WAL transactions; upload acknowledges only after the collector commits.
Disk stalls, encoding and network retries stay off the control loop. Power loss
can lose admitted but uncommitted events. Overflow, persistence failures and
retention losses surface in health counters/gap records when recording resumes.
SQLite quotas count logical event bytes; pages/WAL/indexes use additional disk.
Set a filesystem quota and leave free headroom on deployed volumes.

## Phone provisioning

The phone uses its app container's `Library/Application Support/Observability`.
Find its source identity in `events.db` (`id.producer_id` in a payload) or the
`observationProducerID` app default, prefixed with `phone-`. Provision that exact
identity. The rover ID is the dashboard rover selection and is captured when an
event is created; later selection does not rewrite queued records.

Using Xcode's app-container download/replace tools, place the corresponding
`<producer>.upload.json` file into that directory as `upload.json`, then relaunch
while stationary. The app verifies the producer identity, imports its token
into Keychain and removes the import file after success. Upload health reports
an unconfigured uploader if no credential is available. Never embed tokens in
the app source. Optional `settings.json` in the same directory has exactly the
shared settings fields shown above. The capture worker records JPEG, exact
confidence-filtered depth, validity, native intrinsics, exposure time, mount/
world transforms, ground normal/offset, measured clearance and mapper thresholds.

## Scoped credentials and GCP release

From ARGOS, `bash packaging/gcp/build-observability-release.sh` builds a
credential-free Linux x86_64 bundle using cargo-zigbuild/Zig and the installed
`x86_64-unknown-linux-gnu` Rust target. It prints a private local archive path
and includes file checksums. This prepares files only; it never contacts GCP.

Alternatively, build on a Linux host for the GCP machine's architecture from
the reviewed Terra checkout. A macOS native binary will not run on that host:

```sh
cargo build --release -p terra-observability-collector --locked
python3 ../ARGOS/packaging/gcp/provision-observability.py \
  --output /private/observation-credentials --endpoint https://HOST/observability \
  --rover 4 --producer phone-EXACT_ID --producer pi-rover4 --producer dashboard-laptop
```

This produces private per-source tokens/import files, a separate reader token,
and `credentials.json` containing only hashes. Writers can upload only their
producer and listed rovers plus `_producer` source-health events. Readers can
read/export only listed rovers and source-health context; they cannot ingest.
Protect raw token files, transfer them privately and never commit them. Existing
tokens survive rerunning provisioning. For revocation remove the old hash from
the collector file; retain the source queue while replacing its token.

Prepare `/tmp/argos-release` on the GCP host with:

```text
bin/terra-observability-collector     Linux executable
observability/credentials.json       hashed scopes only
replay/                             contents of Terra/tools/observability, including uv.lock
gcp/                                reviewed ARGOS/packaging/gcp files
```

Exclude `.venv`, test output and caches from `replay/`. Install `uv` and Python
3.12+ first. Review the existing Caddy host name against `gcp/Caddyfile` before
installation. The optional `gcp/install-observability.sh` installs only the
collector/exporter, validates the proxy config and reloads Caddy. It does not
restart the Zenoh router or modify router passwords, listeners or certificate
material. It backs up the previous collector binary, hash file and proxy config;
failed proxy installation restores the prior proxy files.

Collector service overrides accept `TERRA_OBSERVABILITY_EVENT_BYTES` (default
10 GiB), `TERRA_OBSERVABILITY_ATTACHMENT_BYTES` (5 GiB), and
`TERRA_OBSERVABILITY_RETENTION_SECONDS` (30 days), and
`TERRA_OBSERVABILITY_EXPORT_BYTES` (2 GiB total retained export files). Periodic retention records
loss summaries. Export conversion runs one job at a time, with time/source limits
and a bounded export history. Completed jobs release temporary source attachment
copies; oversized jobs fail and clean up rather than consume the event/attachment
budget. As with SQLite, use a filesystem quota for a strict physical disk limit.
The observer can read only allowlisted router/TLS
unit journals. Router tracing is enabled in the rebuilt router binary; installing
the observer alone does not replace/restart an already-running router binary.

Before release, back up the collector SQLite database with SQLite's backup API
(or stop only the collector before copying its DB/WAL) and preserve its attachment
directory. Do not copy a live DB alone. To roll back, stop only the collector,
restore its `.previous` binary/hash file, restore Caddyfile and snippet from the
reported `proxy-backup.*` directory, validate Caddy, reload Caddy, then restart
the collector. Preserve the database and source queues; do not restore an older
database over new evidence without saving the newer one first. Check router
TLS/health separately after any host change.

## Raspberry Pi observer

The independent forwarder runs as `terra-observability` with journal-read access
and no actuator privileges. It reads only `terra-rover.service` and
`bluetooth.service`, persists a cursor/queue, and uploads independently of the
phone. Provision `/etc/terra-observability-pi/upload.token` and `uploader.env`:

```text
TERRA_OBSERVABILITY_PRODUCER=pi-rover4
TERRA_OBSERVABILITY_ROVER=4
TERRA_OBSERVABILITY_ENDPOINT=https://HOST/observability
TERRA_OBSERVABILITY_UPLOAD_TOKEN_FILE=/etc/terra-observability-pi/upload.token
TERRA_OBSERVABILITY_EVENT_BYTES=536870912
TERRA_OBSERVABILITY_RETENTION_SECONDS=604800
```

From a reviewed Terra source bundle run `sudo bash packaging/rover/install-observability.sh`.
It installs only the Python standard-library observer and restarts only that
service. The rover daemon must separately be rebuilt/upgraded to emit the new
structured safety/command events. Apply `terra-rover-observability.conf` as an
optional daemon drop-in during that controlled upgrade; provide `source.env`
with the same producer/rover and a writable source DB path. Do not add a second
direct uploader when using the forwarder: it preserves original structured IDs
from journald. Plain Bluetooth journal messages keep stable cursor-derived IDs.
Journal rotation can remove unforwarded entries; source local journals remain
the fallback. A Pi rebuilt binary and physical run are not verified by a macOS
test or simulator build.

### Offline Pi access

Internet is not required for the Pi's local journal or for a local SSH connection.
An Ethernet link, a shared local Wi-Fi network, or supported USB Ethernet gadget
mode can provide installation and log retrieval access. A USB cable alone does
not enable SSH: confirm the board model and OS first, then configure gadget mode
on the existing installation. Do not reimage the rover's SD card for logging.

On supported Raspberry Pi OS Trixie installations, Raspberry Pi provides
[`rpi-usb-gadget`](https://github.com/raspberrypi/rpi-usb-gadget) for USB networking.
Use the board's supported device-mode port and a data-capable cable. Mac Internet
Sharing can subsequently give the Pi an upload route; without that route, its
queue remains local until connectivity returns. USB networking and internet
sharing are separate configuration steps. Preserve the rover's power arrangement.

Once connected, inspect the installed daemon version before upgrading. Install
the independent forwarder and its credentials, then enable structured source
logging in the rover daemon during a stationary upgrade. An older compiled
daemon will not acquire the new safety events merely by installing the forwarder.

### Emergency-stop investigation coverage

The phone records `safety.requested` with operator/router source and selected
route, Bluetooth stop-command queue success/failure, raw hardware safety fields
including emergency latch and last sequence, and router/Bluetooth connection
transitions. Hardware fields use an allowlist to exclude ownership credentials.
Pi `command.accepted`/`command.rejected` records include command sequence, session,
revision, and resulting emergency-stop/armed/stop-reason state. Joining these
records distinguishes a requested stop from an already latched stop and a lost
link. A queued Bluetooth command is not proof of delivery; the Pi acceptance
event is the receiving-side evidence.

Before the next diagnostic run, verify a real phone source and Pi source appear
in the collector, then interrupt only the observer's upload network to confirm
backlog delivery. Never exercise emergency stop by issuing motion commands as
part of a logging installation. Missing phone/Pi events remain a coverage gap,
even if dashboard state continues to arrive.

## Replay and outage diagnosis

See [replay commands](https://github.com/Super-Yojan/Terra/blob/main/tools/observability/README.md). During an outage,
leave the original source queue/identity intact. Restore connectivity and
credentials; retries use the original IDs/timestamps and duplicate delivery is
idempotent. Camera attachments transfer before their metadata is acknowledged.
Temporary collector storage failures return a retryable server response.
Corrupt/evicted/permanently rejected attachments create causal gap records and
do not block later metadata. An unavailable network or invalid credential keeps
the original batch pending. A queue is for diagnostic evidence; expired drive
commands are never replayed by this system.

Inspect the first motion pause/disarm event, command admission, actual data ages,
hardware state, router connection state and upload backlog separately. Compare
each hop's evidence before blaming the router. In Rerun compare the camera and
depth to the captured ground plane and obstacle height band. Simulator RGB has
no exposure tag and is shown as separate receipt-only evidence. Per-device time
and sequence order remain authoritative; RTT clock estimates have uncertainty
and do not measure clock drift. Nearby service/Pi events without a mission ID
are labelled unassociated context rather than assigned a fabricated session.

Field acceptance should include a stationary reconnect, phone/rover source
identity check, authenticated reader export, flat ground, a known raised object,
camera pitch/roll and an operator-led drive/hold run. Compare before/after
control output with observation enabled/failed. Do not cause motion or reset
pairing remotely to test recording.

## Coordinated source releases

ARGOS and native Zorvane use the sibling `../Terra/crates/terra-observability`
source dependency. Their CI bootstrap scripts pin the reviewed Terra commit;
normal simulator runtime dependencies keep their existing git revision. A clean
local build needs the three repositories side by side. Publish the reviewed
Terra commit before pushing the dependent ARGOS/Zorvane commits or CI cannot
fetch that source. Update both bootstrap pins when changing the shared crate.
The pinned source must be the approved release, not an arbitrary branch tip.

Reviewed recovery behavior includes concurrent attachment quota enforcement,
journal reconnect from the last committed cursor, mission/connection identity
separation, source/collector loss context in exports, and explicit clearing of
missing RGB/depth and superseded frames. An empty camera view after an outage is
intentional: a prior image is never presented as the current depth exposure.


## Offline Pi uploads through a Mac USB connection

Install the independent Pi observer first with `packaging/rover/install-observability.sh`.
It also installs the fixed-path queue helper. Verify a normal SSH login and its
host key before provisioning the Mac relay. The Pi account must permit the
restricted command to run as `terra-observability` without an interactive sudo
prompt (the deployed MasterPi `pi` account already does).

From the Terra checkout on macOS, run:

```sh
python3 packaging/rover/install-usb-relay.py \
  --host pi@raspberrypi.local \
  --endpoint https://argos.34.135.204.210.sslip.io/observability \
  --token-file /absolute/private/path/pi-rover0.token
```

The optional `--control-path` accepts an already authenticated SSH master socket.
The installer creates a protected local key, token and configuration; pins the
previously verified host key; adds an idempotent forced-command public key on
the Pi; and starts the `com.terra.usb-log-relay` login LaunchAgent. That key cannot
open a shell or forward ports. It permits only batches from the observer's fixed
SQLite database and acknowledgments of event identities. It cannot control the rover.

Read `~/Library/Application Support/Terra/usb-log-relay/status.json` for connection
status and the last batch count. `connected=true` means the most recent poll
succeeded; an empty batch does not perform a fresh cloud connectivity check.
Uploads retain original event identities and source times, so collector delivery
is deduplicated after a lost acknowledgment. Retention remains bounded by the
Pi journal's configured storage and age limits.

The Mac must be logged in, awake, connected to the Pi over USB and have internet
access. When unavailable, the observer keeps queued events locally; reconnecting
resumes uploading. Do not queue motion commands using this logging mechanism.

## Browser replay

The Argos HTTPS host serves `/rerun/` with rover/run selection and an embedded
Rerun 0.39.0 viewer. Enter the existing scoped **reader** token; writer tokens do
not grant viewing access. The token stays in page memory, is never put into URLs
or browser storage, and is cleared by sign-out/page close. Recordings are fetched
through authenticated collector exports, then passed to the same-origin viewer
as a temporary Blob URL. Downloads use the same authorized recording.

Choose a source-clock time range. Device events without a mission ID can be
exported with `session_id: null` and an explicit window of at most 24 hours.
Named session exports remain supported. An unset device clock can put source
events on an earlier date than receipt; do not infer physical timing from those
wall clocks. Missing sensor evidence is not reconstructed by replay.

The separate `terra-rerun-viewer.service` serves pinned viewer assets only through
loopback HTTP 9090; its unused SDK listener is bound to loopback 9877. Caddy
exposes `/rerun/viewer/*` for assets and `/rerun/*` for the collector's public UI
shell. Session and artifact API routes still require reader authentication. No
public SDK ingestion route is added. Restarting the collector invalidates its
in-memory export job index; recreate an export after a restart.

### Connectivity display defaults

Edit `crates/terra-types/src/connectivity_policy.rs` for GPS display freshness (3 s) and fleet stale presentation (90 s). The phone obtains these defaults through its native binding; ARGOS uses the same crate. Old GPS observations retain their source timestamp and coordinates and are labelled last known. Neither value changes local sensor admission, remote command expiry or navigation GPS quality checks. Fleet membership remains visible without geographic placement.

### Onboard autonomy during router loss

The default `ConnectivityPolicySettings.autonomous_router_loss` is `continue_accepted_mission`; use `pause` when a deployment requires a policy hold. Set the complete validated JSON object with `MobileController.set_connectivity_policy_settings`. `connectivity_policy_settings` returns the effective values. The controller reports the policy and its revision in control summaries, localization and authority telemetry. The phone refreshes its display policy from the controller; GPUI reads each run's policy. Fleet display overrides never change command admission.

`handle_dashboard_link_loss` handles automatic transport failure. Explicit `disconnect_dashboard`, goal cancel, local hardware loss and emergency stop keep their existing stop semantics. A reconnect preserves mission/run identity and budgets, but does not arm or resume a latched pause. Each execution tick still checks its local inputs.

New remote motion-producing actions, safety resets and approvals must contain the current `run_id` and `authority_revision`. A link failure/replacement retires the old revision without clearing an accepted mission. GPUI and the phone's remote sender attach this binding from observed authority; old unbound clients are rejected with `current_authority_required_upgrade_legacy_client`. Stop/cancel/disarm remain available. Remote operator intent retains its short lease; only logs and evidence are durably queued. The 90-second cached-authority issuance limit lives in `terra-types/src/timing.rs::REMOTE_AUTHORITY_CACHE_SECONDS` and is independent of the similarly valued fleet display default.

For a release check, leave the rover disarmed, open Terra in foreground, verify current phone and Pi events reach the existing collector/Rerun endpoint and pending counts decrease, then inspect a router interruption and recovery. A subsequent low-speed driving test needs an operator with the rover's local stop available. Do not interpret a successful logging fix as proof that every physical Zenoh disconnect cause is resolved.
