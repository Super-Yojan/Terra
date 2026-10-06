# Terra Bluetooth actuator peripheral

Linux BlueZ and Python 3.10+ are required. Install the `hardware/raspberry-pi` package in a virtual environment (dependency dbus-next 0.2.3); real hardware additionally requires deployment-approved `fusion_hat` 1.14.0. Enable bluetooth.service. Provision a terra-rover system user with I2C access, a private mode-0700 `/var/lib/terra-rover`, and BlueZ system-bus permission to register GATT/advertisements and set adapter properties. Install packaging/terra-rover.service after adapting `/opt/terra` to your installation. BlueZ system-bus policy varies by distribution; grant these actions only to the service account.

The independent physical gate producer writes ASCII `0` for confirmed isolated/open and `1` for closed. Other tokens/read failures forbid motion and configuration. The gate file must be protected from the service/phone and refreshed by supervised hardware input; a stale file is not detected by this adapter. Prefer a directly injected GPIO gate reader for production. Gate closure never arms. Outputs must be independently isolated before service termination: process exit cannot guarantee continuous ESC stop PWM.

Stop the normal service, open/isolate the physical gate, and run locally on a terminal:

```
python3 -m terra_rover --setup-owner --expected-peer AA:BB:CC:DD:EE:FF --setup-seconds 60 --gate-file /run/terra-interlock/gate
```

The expected peer is the phone identity address visible to BlueZ; compare the displayed six-digit number on both devices and type `yes` locally. Setup uses DisplayYesNo, accepts only the specified peer, and requires numeric confirmation. It registers no actuator GATT service. Setup expires within 60 seconds and disables pairability/discoverability on exit. Owner JSON stores bonded identity Address/AddressType with mode 0600; BlueZ retains bond keys. Existing owner storage prevents accidental replacement. Replacing a phone requires stopping service, isolating equipment and explicitly removing owner JSON and the corresponding BlueZ bond before repeating setup. Unexpected phones are never admitted. Normal operation sets Pairable=false.

Run the service or a standalone mock (a Linux Bluetooth radio is still required):

```
python3 -m terra_rover --mock --name 'Terra Rover' --config /var/lib/terra-rover/mock-layout.json --owner /var/lib/terra-rover/owner.json --pwm-ports P0,P1 --mock-gate-closed
```

Mock gate defaults open; --mock-gate-closed explicitly enables its simulated interlock. Real hardware requires --gate-file and defaults to M0–M3 capabilities. Supply --pwm-ports only for physically confirmed exposed P0–P11 ports. Timer/resource conflicts are validated before activation. Persisted layout load fails closed and advertises faults if gate is closed/unavailable; reconnect never restores arming.

## Swift/CoreBluetooth integration

Every UUID ends `-4c2b-4f91-9e3a-1d8c6b2a0f10`:

| Prefix | Purpose | Properties |
|---|---|---|
|7e5a0010|Primary service|—|
|7e5a0011|Drive|Encrypted write with response|
|7e5a0012|Status|Encrypted read, notify|
|7e5a0013|Binary priority commands / JSON control|Encrypted write with response|
|7e5a0014|JSON configuration replies|Encrypted read, notify|

Read status first to trigger link encryption and owner admission, then enable status and reply notifications. StartNotify provides no device identity in BlueZ, so it rejects until an owner has been admitted by an encrypted ReadValue/WriteValue and is the sole connected central. A second connected central disables the admitted session. After disconnect read/admit again and obtain the new session from status before explicit safe DRIVE+ARM. The first status read can precede worker admission; wait for status notification with a non-null session. Notifications are sent only while that admitted bonded owner remains the sole connected central. There is no fabricated Device1.Encrypted property: BlueZ enforces encrypt-read/encrypt-write ATT permissions.

All writes and notifications use `[message_id:u16 LE,index:u8,count:u8,chunk...]`. Assemble independently by characteristic/message ID; clear on disconnect; expire at 100 ms. Status reads and reply reads return unfragmented JSON through ATT read/offset semantics. Notification fragment capacity defaults to minimum ATT value length 20 and learns `mtu-3` from authenticated read/write options. Documents requiring >255 fragments cannot notify at that MTU; retrieve the full last reply via read instead. At MTU 23 the notification logical limit is 4080 bytes; a 16 KiB reply requires ATT capacity at least 69. Subscribe before requesting; use write-with-response and a fresh message ID per logical request.

Drive/control binary logical header is `TA`, version u8=1, kind u8 (drive=1, arm=2, disarm=3, emergency_stop=4), session/revision/sequence u32 LE, followed by `(id:u8,value:f32 LE)` records (drive only, maximum 16 IDs). Control first fragment must contain at least `TA` for binary or JSON opening bytes; subsequent control fragments route using message ID. Drive, priority, JSON assemblers remain independent. Increasing sequence is shared across command kinds; max u32 retires session. Successful ATT write acknowledges transport only. Status notification `{schema_version:1,type:"command_acceptance",session,sequence,accepted}` reports safety acceptance; malformed/incomplete commands produce no acceptance.

Status JSON has type `status`, schema_version, transport generation, session, active_revision, armed, arming, hardware_gate, status_subscribed, fault, emergency_stop, stop_reason, last_sequence, command_age_ms, service_state, configuration_errors, battery=null and battery_reason="unsupported". Status repeats at 10 Hz; client sends fresh full drive at 20 Hz. Drive mailbox retains only latest frame with original complete arrival time/generation. Watchdog expires at 200 ms; tick targets 10 ms. Disconnect and loss of status subscription disarm. Configuration never refreshes drive age, and mutating configuration requires disarmed state and confirmed open real gate.

Control JSON: `{schema_version:1,request_id:u32,operation,payload}`. `capabilities`, `read_layout`, `reset_fault`, `reset_emergency_stop` use `{}`; `stage_layout` uses `{layout}` whose revision equals active base; `commit_layout` uses `{staged_revision,staged_request_id}` from exact stage. Replies: `{schema_version:1,request_id,result:"ok"|"error",active_revision,errors:[{actuator_id,code,message}],payload}`. Stage/commit IDs correlate within authenticated connection; reconnect clears pending stage/replay cache. Duplicate request bytes replay cached reply; changed bytes under reused ID reject. Read/capability responses can be historical: use fresh IDs.

No tests, builds, syntax checks, smoke scripts or hardware checks were run for this implementation. Radio behavior and hardware timing remain unverified. API references: [BlueZ GATT](https://bluez.readthedocs.io/en/latest/gatt-api/) and [dbus-next service API](https://python-dbus-next.readthedocs.io/en/latest/high-level-service/index.html).
