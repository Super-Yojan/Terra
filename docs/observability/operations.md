# Operational history and Rerun

The observer is independent of drive control and the experiment recorder. It
records normal events, transitions, command admission, faults and sampled state.
It does not change timeouts, arming, safety decisions, Zenoh routing or TLS.
See [coverage](coverage.md) and [event schema](event-schema.md) for exact scope.

## Status and deployment boundary

Source code, local tests, iOS build and a synthetic Rerun preview are available.
GCP installation, Pi installation and physical field acceptance are separate,
operator-controlled steps. No field run or production deployment is implied.
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

Only the first three are needed for local recording; a missing upload endpoint
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

Build the collector for the GCP machine's Linux architecture from the reviewed
Terra checkout, not the macOS binary:

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
`TERRA_OBSERVABILITY_RETENTION_SECONDS` (30 days). Periodic retention records
loss summaries. Export conversion runs one job at a time, with time/source limits
and a bounded export history. The observer can read only allowlisted router/TLS
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

## Replay and outage diagnosis

See [replay commands](../../tools/observability/README.md). During an outage,
leave the original source queue/identity intact. Restore connectivity and
credentials; retries use the original IDs/timestamps and duplicate delivery is
idempotent. Camera attachments transfer before their metadata is acknowledged.
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
