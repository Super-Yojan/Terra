# Outdoor GPS missions

TerraPhone now implements version-1 `outdoor_gps` guidance in the shared Rust core.
Local navigation remains the default. Direct phone Bluetooth driving is unchanged.
The Pi service does not run this controller; deploy the updated phone app and ARGOS together.

The phone supplies original monotonic observation times for measured GPS and true
heading. The heading uses the calibrated rover forward axis paired with Core
Location's configured phone-right reference axis. GPS never modifies the AR pose
or occupancy map. At most twice per second a geographic bearing produces a local
target up to 3 m away; L3's existing route and footprint/braking checks choose the
command. Forward speed is at most 0.3 m/s, or a lower configured controller limit.
Reverse retains the conservative turning-policy cap and rear unknown-space checks.

Source selection uses `navigation/source` with schema_version 1, source (`local`
or `outdoor_gps`), token, run_id and navigation_revision. It requires fresh disarmed
hardware, no pending hardware request and no active mission. Acceptance increments
the navigation revision. An optional arrival_radius_m configures the default in
2–20 m; the default is 5 m. Selection never arms the hardware.

Outdoor `goal` JSON preserves frame `wgs84`, latitude, longitude and token and adds
run_id, navigation_revision and arrival_radius_m. ARGOS binds these to the reviewed
operation and rechecks them just before publication. Terra rejects missing/stale
bindings and queued outdoor goals older than 0.5 s. The radius is immutable for that
mission. Local goal JSON and cancellation (`{"cancel":true}`) are unchanged.

Admission requires L3, ready armed hardware and fresh phone VIO motion feedback, healthy local
pose and fresh occupancy, GPS age ≤3 s and accuracy ≤10 m and ≤arrival radius,
and true-heading age ≤2 s with accuracy ≤30 degrees. Goals beyond 500 m are rejected.
Wheel encoders are not required. L3 uses the phone's VIO/IMU velocity estimate and
requires a differential motor layout. Both direct wheel-effort routes and compatible
manual-mixer routes receive the closed-loop controller output. A missing/stale VIO
pose still blocks admission and motion; direct Bluetooth manual driving is unchanged.

GPS/heading rejection, tracking/map loss, disarm and authority loss latch a pause.
Recovery alone does not resume. The GPUI dashboard's Resume outdoor mission button
sends `navigation/resume` with schema_version, token, run_id and navigation_revision;
Terra rechecks fresh inputs, L3 and armed hardware. Web operators can cancel and
submit a newly reviewed mission. AR frame/run resets invalidate the mission.
Three distinct valid GPS fixes inside the frozen radius confirm arrival, with zero
motion while confirmation is pending. An outside fix resets confirmation.

`navigation/outdoor` and the nested autonomy `navigation.outdoor` report expose
source, supported/acknowledged, navigation_revision, readiness, phase, hold_reason,
source ages/accuracies, raw GPS, destination, remaining_m, arrival_radius_m and
request result/token. `ready` describes sensor readiness, separate from explicit
arming/L3 admission. Unsupported platforms remain local-only. Kotlin bindings are
regenerated, but Android does not advertise outdoor capability without ingestion.

Validation covers mission simulation, source-time/replay rejection, paused recovery,
immutable geographic identity/radius, controller/checked-trace equality, occupied
and unknown map holds, mount-heading conversion and ARGOS wire envelopes. Builds
and simulation are not physical outdoor validation: compass bias, GPS drift,
actuator response and real obstacle trials remain to be measured on hardware.

Sensor recovery permits rearming a paused GPS mission, but keeps local guidance cleared. Arming alone cannot restart it: an explicit Resume with fresh evidence and armed hardware is still required.
