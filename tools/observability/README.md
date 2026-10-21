# Operational replay

Install from this directory with `uv sync --frozen`. The lockfile pins Rerun
SDK/viewer 0.39.0. Python 3.12 or newer is required. Use matching viewer/SDK
versions when reviewing recordings.

Export a source journal or a directory containing `events.jsonl` and
`attachments/<sha256>`:

```sh
uv run terra-replay export /path/to/events.db --rover 4 --session RUN_ID --output /tmp/run.rrd
uv run rerun /tmp/run.rrd
```

Download a scoped recording from the collector (reader token file mode 0600):

```sh
uv run terra-replay fetch https://HOST/observability --token-file /private/reader.token --rover 4 --session RUN_ID --output-dir /tmp/rover-run
uv run rerun /tmp/rover-run/recording.rrd
```

Find session IDs with authenticated `GET /observability/v1/sessions` or from the
source event journal. Credentials belong in the Authorization header; never put
them in URLs or recordings. The fetch client does not follow redirects.

An export also produces `.events.jsonl`, `.index.json`, and `.manifest.json`.
These retain event identity, source/receipt time, gap count, revision and measured
clock uncertainty. Exporting late-arriving events creates a different revision;
it never renumbers source events or sends drive commands.

The default layout shows event text, motion curves, occupancy, camera/depth and
3D ground/obstacle boundaries. Unknown occupancy is gray. The classifier uses
the captured calibration, ground plane and actual mapper height thresholds.
Missing attachments stay explicitly missing. Simulator RGB is receipt-only and
displayed separately from calibrated depth because its exposure is unavailable.

`wall_clock_approx` is approximate across machines. `estimated_server_time`
uses measured RTT offset and uncertainty where available; clock drift is not
measured. Per-producer boot sequence/source timelines are authoritative for
ordering. Nearby Pi/service events with no mission ID are marked unassociated
context, not assigned to a mission retroactively.

The fixture in `tests/fixtures/demo` is synthetic, not a physical acceptance run:

```sh
uv run terra-replay export tests/fixtures/demo --output /tmp/demo.rrd
uv run rerun /tmp/demo.rrd
uv run pytest tests
```

See [operations](../../docs/observability/operations.md) for provisioning,
retention, outage recovery and deployment boundaries.
