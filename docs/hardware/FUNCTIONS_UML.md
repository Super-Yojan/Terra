# Raspberry Pi hardware: function UML

This is a **historical implementation snapshot** from before the Pi motor relay simplification. Source links are pinned to that revision; the diagrams do not describe the current implementation. It includes every explicitly defined Python function and method under `hardware/raspberry-pi/terra_rover/`, including private helpers, nested callbacks, async functions, and property accessors. `rover.py` is the executable wrapper. Tests, build dependencies, third-party drivers, and implicit/generated methods are excluded. The phone app is outside this scope.

Each module has a Mermaid UML class diagram. `ModuleFunctions` groups free functions; `Callbacks` groups functions defined inside another function. These groups are documentation containers, not runtime classes. Signatures show parameter names without defaults or type annotations; `self`/`cls` are omitted, and variadic parameters use `args_`/`kwargs_` prefixes. `+` means a public name and `-` means a leading-underscore name; Python does not enforce this visibility. Getter/setter definitions may share a name.

Class arrows show inheritance. In the overview, dashed arrows show direct relative imports, not every possible runtime call. The sequence diagram is a representative drive iteration; configuration, error handling, and lifecycle branches are listed in the module diagrams.

## Module dependencies

```mermaid
flowchart TD
  __init__["__init__.py"]
  __main__["__main__.py"]
  backend["backend.py"]
  ble["ble.py"]
  bless_transport["bless_transport.py"]
  configuration["configuration.py"]
  layout["layout.py"]
  lifecycle["lifecycle.py"]
  log_forwarder["log_forwarder.py"]
  observability["observability.py"]
  onboarding["onboarding.py"]
  pairing["pairing.py"]
  protocol["protocol.py"]
  safety["safety.py"]
  usb_queue["usb_queue.py"]
  __main__ -. imports .-> backend
  __main__ -. imports .-> ble
  __main__ -. imports .-> bless_transport
  __main__ -. imports .-> onboarding
  __main__ -. imports .-> observability
  __main__ -. imports .-> lifecycle
  backend -. imports .-> layout
  backend -. imports .-> safety
  ble -. imports .-> bless_transport
  ble -. imports .-> configuration
  ble -. imports .-> safety
  ble -. imports .-> protocol
  configuration -. imports .-> backend
  configuration -. imports .-> layout
  configuration -. imports .-> protocol
  layout -. imports .-> protocol
  lifecycle -. imports .-> onboarding
  lifecycle -. imports .-> pairing
  log_forwarder -. imports .-> observability
  pairing -. imports .-> ble
  pairing -. imports .-> bless_transport
  pairing -. imports .-> onboarding
  safety -. imports .-> observability
  safety -. imports .-> layout
  safety -. imports .-> protocol
  usb_queue -. imports .-> observability
```

The package `__init__.py` has no explicit function definitions. `rover.py` imports and invokes `terra_rover.__main__.main()`.

## Drive command path

Snapshot timing: drive-fragment assembly expires at 100 ms, the output worker targets a 10 ms loop, the motion watchdog expires at 200 ms, and recurring status publication targets 100 ms. The watchdog and arming rules remain present in this diagram because they are present in the source.

```mermaid
sequenceDiagram
    participant Phone as iPhone
    participant GATT as PeerCharacteristic
    participant BLE as BlePeripheral
    participant Frames as FragmentAssembler / protocol
    participant Safety as SafetyController
    participant Backend as FusionHatBackend / _Adapter
    participant HAT as Fusion HAT driver
    Phone->>GATT: WriteValue(value, options)
    GATT->>BLE: authorize(options), handle_write(...)
    BLE->>BLE: _handle_write(characteristic, value, now)
    BLE->>Frames: push(value, now)
    Frames-->>BLE: assembled bytes, or incomplete
    opt Complete drive frame
        BLE->>Frames: decode_frame(data)
        Frames-->>BLE: CommandFrame
        BLE->>BLE: replace latest queued drive
    end
    loop _output_loop() worker iteration
        BLE->>Safety: accept(frame, received_at, now), if queued
        Safety->>Frames: encode_frame(), decode_frame(), validate_command()
        Safety->>Safety: _clock(), _expire(), admission checks
        BLE->>Backend: read_gate()
        BLE->>Safety: tick(now, hardware_gate)
        alt Armed, unpaused, accepted mailbox available
            Safety-->>BLE: OutputBatch with commanded values
        else Stopped, expired, gated, or faulted
            Safety-->>BLE: OutputBatch with configured safe outputs
        end
        BLE->>Backend: apply(batch), when layout exists
        Backend->>Backend: _mapped(), _write()
        Backend->>HAT: motor / PWM / servo output
        opt Status interval reached
            BLE->>BLE: store.status(), status_read_document(), _emit()
            BLE-->>Phone: status notification
        end
    end
```

## Function inventory

| Module | Explicit functions and methods |
| --- | ---: |
| [rover.py](https://github.com/Super-Yojan/Terra/blob/8695cf67992647c163142cf3f3b6d81aaaba4bd0/hardware/raspberry-pi/rover.py) | 0 |
| [terra_rover/__init__.py](https://github.com/Super-Yojan/Terra/blob/8695cf67992647c163142cf3f3b6d81aaaba4bd0/hardware/raspberry-pi/terra_rover/__init__.py) | 0 |
| [terra_rover/__main__.py](https://github.com/Super-Yojan/Terra/blob/8695cf67992647c163142cf3f3b6d81aaaba4bd0/hardware/raspberry-pi/terra_rover/__main__.py) | 17 |
| [terra_rover/backend.py](https://github.com/Super-Yojan/Terra/blob/8695cf67992647c163142cf3f3b6d81aaaba4bd0/hardware/raspberry-pi/terra_rover/backend.py) | 37 |
| [terra_rover/ble.py](https://github.com/Super-Yojan/Terra/blob/8695cf67992647c163142cf3f3b6d81aaaba4bd0/hardware/raspberry-pi/terra_rover/ble.py) | 27 |
| [terra_rover/bless_transport.py](https://github.com/Super-Yojan/Terra/blob/8695cf67992647c163142cf3f3b6d81aaaba4bd0/hardware/raspberry-pi/terra_rover/bless_transport.py) | 18 |
| [terra_rover/configuration.py](https://github.com/Super-Yojan/Terra/blob/8695cf67992647c163142cf3f3b6d81aaaba4bd0/hardware/raspberry-pi/terra_rover/configuration.py) | 24 |
| [terra_rover/layout.py](https://github.com/Super-Yojan/Terra/blob/8695cf67992647c163142cf3f3b6d81aaaba4bd0/hardware/raspberry-pi/terra_rover/layout.py) | 6 |
| [terra_rover/lifecycle.py](https://github.com/Super-Yojan/Terra/blob/8695cf67992647c163142cf3f3b6d81aaaba4bd0/hardware/raspberry-pi/terra_rover/lifecycle.py) | 4 |
| [terra_rover/log_forwarder.py](https://github.com/Super-Yojan/Terra/blob/8695cf67992647c163142cf3f3b6d81aaaba4bd0/hardware/raspberry-pi/terra_rover/log_forwarder.py) | 2 |
| [terra_rover/observability.py](https://github.com/Super-Yojan/Terra/blob/8695cf67992647c163142cf3f3b6d81aaaba4bd0/hardware/raspberry-pi/terra_rover/observability.py) | 18 |
| [terra_rover/onboarding.py](https://github.com/Super-Yojan/Terra/blob/8695cf67992647c163142cf3f3b6d81aaaba4bd0/hardware/raspberry-pi/terra_rover/onboarding.py) | 11 |
| [terra_rover/pairing.py](https://github.com/Super-Yojan/Terra/blob/8695cf67992647c163142cf3f3b6d81aaaba4bd0/hardware/raspberry-pi/terra_rover/pairing.py) | 23 |
| [terra_rover/protocol.py](https://github.com/Super-Yojan/Terra/blob/8695cf67992647c163142cf3f3b6d81aaaba4bd0/hardware/raspberry-pi/terra_rover/protocol.py) | 21 |
| [terra_rover/safety.py](https://github.com/Super-Yojan/Terra/blob/8695cf67992647c163142cf3f3b6d81aaaba4bd0/hardware/raspberry-pi/terra_rover/safety.py) | 19 |
| [terra_rover/usb_queue.py](https://github.com/Super-Yojan/Terra/blob/8695cf67992647c163142cf3f3b6d81aaaba4bd0/hardware/raspberry-pi/terra_rover/usb_queue.py) | 6 |
| **Total** | **233** |

## Entry points

- `python -m terra_rover` → `__main__.main()` → enrollment/customer lifecycle or `run_normal()`.
- `run_normal()` chooses a backend and awaits `BlePeripheral.run()`.
- `log_forwarder.main()` reads system/kernel journals and forwards observations independently.
- `usb_queue.main()` exposes the constrained observation-queue access used by the USB relay.

## terra_rover/__main__.py

[Source](https://github.com/Super-Yojan/Terra/blob/8695cf67992647c163142cf3f3b6d81aaaba4bd0/hardware/raspberry-pi/terra_rover/__main__.py) · 17 explicit function/method definitions.

```mermaid
classDiagram
  direction TB
  class C0["SetupAgent"] {
    -__init__(expected, deadline)
    +check(device)
    +RequestConfirmation(device, passkey)
    +RequestAuthorization(device)
    +AuthorizeService(device, uuid)
    +Cancel()
    +Release()
  }
  class C1["SetupAgent.RequestConfirmation Callbacks"] {
    <<callbacks>>
    +ready()
  }
  class C2["SetupStatus"] {
    -__init__(agent, gate_file)
    +UUID()
    +Service()
    +Flags()
    +ReadValue(options)
  }
  class C3["ModuleFunctions"] {
    <<module>>
    +setup(args)
    +main()
    +run_normal(args, name)
  }
  class C4["setup Callbacks"] {
    <<callbacks>>
    +read(options)
  }
```

## terra_rover/backend.py

[Source](https://github.com/Super-Yojan/Terra/blob/8695cf67992647c163142cf3f3b6d81aaaba4bd0/hardware/raspberry-pi/terra_rover/backend.py) · 37 explicit function/method definitions.

```mermaid
classDiagram
  direction TB
  class C0["BackendError"] {
    -__init__(port, operation, message)
  }
  class C1["BackendValidationError"] {
  }
  class C2["ModuleFunctions"] {
    <<module>>
    -_forget_exception_owners(exc)
    +file_gate_reader(path)
    -_capabilities(pwm_ports, library, version, occupied_resources)
    -_safe(a)
    -_mapped(a, value)
  }
  class C3["Backend"] {
    +capabilities()
    +configure(layout)
    +apply(batch)
    +read_gate()
    +require_gate_open()
    +close()
  }
  class C4["BenchGate"] {
    -__init__()
    -__call__()
    +reset()
    +set_enabled(enabled)
  }
  class C5["file_gate_reader Callbacks"] {
    <<callbacks>>
    +read()
  }
  class C6["_Adapter"] {
    -__init__(caps, gate_reader)
    +capabilities()
    +read_gate()
    +require_gate_open()
    +configure(layout)
    -_before_configure()
    -_release_for_configure()
    -_prepare()
    -_open(a)
    -_safe_all()
    +apply(batch)
    +close()
  }
  class C7["MockBackend"] {
    -__init__(pwm_ports, occupied_resources)
    -_write(a, value)
  }
  class C8["FusionHatBackend"] {
    -__init__(pwm_ports, gate_reader, supported_library_versions, motor_factory, pwm_factory, library_version, occupied_resources, external_power_cutoff)
    -_before_configure()
    -_release_for_configure()
    -_prepare()
    -_open(a)
    -_write(a, value)
  }
  C0 <|-- C1
  C6 <|-- C7
  C6 <|-- C8
```

`Backend` is the interface; `_Adapter` implements common mapping and output behavior. `MockBackend` and `FusionHatBackend` supply concrete writes. Hardware-specific objects are supplied by the external Fusion HAT library.

## terra_rover/ble.py

[Source](https://github.com/Super-Yojan/Terra/blob/8695cf67992647c163142cf3f3b6d81aaaba4bd0/hardware/raspberry-pi/terra_rover/ble.py) · 27 explicit function/method definitions.

```mermaid
classDiagram
  direction TB
  class C0["ModuleFunctions"] {
    <<module>>
    +denied(message)
    +status_read_document(payload)
  }
  class C1["BlePeripheral"] {
    -__init__(name, adapter)
    -_stalled_owner(path)
    +recover_stalled_connection(now)
    -_connection_watchdog()
    +clear()
    +require_single_owner()
    -_owner_device(peer)
    +authorize(options)
    +enqueue(kind, value)
    +disconnect(peer, now)
    +malformed_drive(peer)
    +handle_write(peer, characteristic, value, now)
    -_handle_write(characteristic, value, now)
    -_emit(characteristic, payload, generation)
    +join_worker(worker)
    +monitor_worker(worker)
    -_worker()
    -_output_loop()
    +run(backend, store_path, owner_store, manage_signals)
  }
  class C2["BlePeripheral.run Callbacks"] {
    <<callbacks>>
    +read(options, role)
    +write(value, options, role)
    +invalid_write(options, role)
    +notify(enabled, role)
    +changed(message)
    +added(path, interfaces)
  }
```

`_worker()` owns actuator output. `run()` registers GATT callbacks and lifecycle handlers; its nested functions appear in the callbacks group. `_connection_watchdog()` recovers stalled owner admission separately from the 200 ms motor-command watchdog in `safety.py`.

## terra_rover/bless_transport.py

[Source](https://github.com/Super-Yojan/Terra/blob/8695cf67992647c163142cf3f3b6d81aaaba4bd0/hardware/raspberry-pi/terra_rover/bless_transport.py) · 18 explicit function/method definitions.

```mermaid
classDiagram
  direction TB
  class C0["StableGattService"] {
    -__init__(original, handle)
    +Handle()
    +Handle(value)
    +get_obj()
  }
  class C1["PeerCharacteristic"] {
    -__init__(original, read, write, notify, invalid_write)
    +require_peer(options)
    +ReadValue(options)
    +WriteValue(value, options)
    +StartNotify()
    +StopNotify()
    +notifying()
    +notifying(enabled)
  }
  class C2["BlessTransport"] {
    -__init__(name, service_uuid, adapter)
    +prepare()
    +add(uuid, read, write, notify, invalid_write)
    +start()
    +close()
  }
  class C3["BlessTransport.add Callbacks"] {
    <<callbacks>>
    +publish(data)
  }
```

## terra_rover/configuration.py

[Source](https://github.com/Super-Yojan/Terra/blob/8695cf67992647c163142cf3f3b6d81aaaba4bd0/hardware/raspberry-pi/terra_rover/configuration.py) · 24 explicit function/method definitions.

```mermaid
classDiagram
  direction TB
  class C0["ModuleFunctions"] {
    <<module>>
    -_error(code, message)
  }
  class C1["ConfigurationStore"] {
    -__init__(path, safety, backend)
    +active_revision()
    +read()
    -_validate(layout)
    -_serialize(layout)
    -_read_file()
    -_gate_errors(now)
    +load(now)
    +reset_bench()
    +begin_connection(session, now)
    +end_connection(now)
    -_reply(request_id, errors, payload)
    +stage(request_id, layout, expected_revision, now)
    +commit(request_id, now, staged_request_id, staged_revision)
    +handle_bytes(data, now)
    +handle(envelope, now)
    -_dispatch(envelope, now)
    -_draft_payload()
    -_paginate(request_id, operation, payload, now)
    -_persist(candidate)
    -_commit_candidate(request_id, candidate, now)
    +status(now)
  }
  class C2["ConfigurationStore._paginate Callbacks"] {
    <<callbacks>>
    +reject(code, message)
  }
```

## terra_rover/layout.py

[Source](https://github.com/Super-Yojan/Terra/blob/8695cf67992647c163142cf3f3b6d81aaaba4bd0/hardware/raspberry-pi/terra_rover/layout.py) · 6 explicit function/method definitions.

```mermaid
classDiagram
  direction TB
  class C0["ModuleFunctions"] {
    <<module>>
    +normalize_layout(layout)
    +validate_structure(layout)
    +validate_layout(layout, capabilities)
  }
  class C1["normalize_layout Callbacks"] {
    <<callbacks>>
    +convert(container, key)
  }
  class C2["validate_structure Callbacks"] {
    <<callbacks>>
    +add(identifier, code, message)
  }
  class C3["validate_layout Callbacks"] {
    <<callbacks>>
    +add(a, code, message)
  }
```

## terra_rover/lifecycle.py

[Source](https://github.com/Super-Yojan/Terra/blob/8695cf67992647c163142cf3f3b6d81aaaba4bd0/hardware/raspberry-pi/terra_rover/lifecycle.py) · 4 explicit function/method definitions.

```mermaid
classDiagram
  direction TB
  class C0["ModuleFunctions"] {
    <<module>>
    +wait_for_hold(hat, detector)
    +serve_until_hold(args, name, normal, hat, detector)
    +flash_success(hat)
    +run_customer_service(args, normal, pair)
  }
```

## terra_rover/log_forwarder.py

[Source](https://github.com/Super-Yojan/Terra/blob/8695cf67992647c163142cf3f3b6d81aaaba4bd0/hardware/raspberry-pi/terra_rover/log_forwarder.py) · 2 explicit function/method definitions.

```mermaid
classDiagram
  direction TB
  class C0["ModuleFunctions"] {
    <<module>>
    +project_journal(entry)
    +main()
  }
```

## terra_rover/observability.py

[Source](https://github.com/Super-Yojan/Terra/blob/8695cf67992647c163142cf3f3b6d81aaaba4bd0/hardware/raspberry-pi/terra_rover/observability.py) · 18 explicit function/method definitions.

```mermaid
classDiagram
  direction TB
  class C0["ModuleFunctions"] {
    <<module>>
    +set_journal(journal)
    +record_event(component, kind, attributes)
    +configure_from_environment()
  }
  class C1["ObservationJournal"] {
    -__init__(path, producer_id, rover_id, capacity, max_bytes, emit_journal, retention_seconds)
    -_db()
    -_envelope(event)
    +record(event)
    +record_envelope(envelope)
    -_persist(db, payload)
    -_work()
    +pending(limit)
    +acknowledge(ids)
    +flush(timeout)
    +close()
  }
  class C2["ObservationUploader"] {
    -__init__(journal, endpoint, token)
    +once()
    -_work()
  }
  class C3["ObservationUploader.once.NoRedirect"] {
    +redirect_request(req, fp, code, msg, headers, newurl)
  }
```

The journal and uploader run independently of actuator control. Observation failures are caught rather than used as motion-admission decisions.

## terra_rover/onboarding.py

[Source](https://github.com/Super-Yojan/Terra/blob/8695cf67992647c163142cf3f3b6d81aaaba4bd0/hardware/raspberry-pi/terra_rover/onboarding.py) · 11 explicit function/method definitions.

```mermaid
classDiagram
  direction TB
  class C0["HoldDetector"] {
    -__init__()
    +update(pressed, now)
  }
  class C1["HatIndicators"] {
    -__init__(button_path, led_path)
    +pressed()
    +set_led(enabled)
  }
  class C2["ModuleFunctions"] {
    <<module>>
    +validate_owner(identity)
    +load_owner(path)
    -_exclusive_json(path, value)
    +save_owner(path, identity)
    +load_device_name(path)
    +pairing_name(name)
  }
```

## terra_rover/pairing.py

[Source](https://github.com/Super-Yojan/Terra/blob/8695cf67992647c163142cf3f3b6d81aaaba4bd0/hardware/raspberry-pi/terra_rover/pairing.py) · 23 explicit function/method definitions.

```mermaid
classDiagram
  direction TB
  class C0["PairingCleanupError"] {
  }
  class C1["PairingSession"] {
    -__init__(deadline, preexisting_bonds, clock)
    +check(device)
    +authorize(device)
    +read(device)
    +ready(fields)
    +connection_changed(device, connected)
    +cancel()
  }
  class C2["ButtonPairingAgent"] {
    -__init__(session)
    +RequestAuthorization(device)
    +RequestConfirmation(device, passkey)
    +AuthorizeService(device, uuid)
    +Cancel()
    +Release()
  }
  class C3["ButtonSetupStatus"] {
    -__init__(session, commit)
    +UUID()
    +Service()
    +Flags()
    +ReadValue(options)
  }
  class C4["ModuleFunctions"] {
    <<module>>
    +remove_owner_bond(args, owner)
    +pair_owner(args, indicators, name)
  }
  class C5["pair_owner Callbacks"] {
    <<callbacks>>
    +changed(message)
    +commit(device)
    +read(options)
  }
```

## terra_rover/protocol.py

[Source](https://github.com/Super-Yojan/Terra/blob/8695cf67992647c163142cf3f3b6d81aaaba4bd0/hardware/raspberry-pi/terra_rover/protocol.py) · 21 explicit function/method definitions.

```mermaid
classDiagram
  direction TB
  class C0["ProtocolError"] {
  }
  class C1["CommandKind"] {
  }
  class C2["ActuatorValue"] {
    +id
    +value
  }
  class C3["CommandFrame"] {
    +kind
    +session
    +revision
    +sequence
    +values
  }
  class C4["ModuleFunctions"] {
    <<module>>
    -_uint(value, bits)
    +encode_frame(frame)
    +decode_frame(data)
    +validate_command(frame, session, layout, last_sequence)
    +fragment_message(data, message_id, write_size, limit)
    +allocate_message_id(next_id, active)
    -_object(pairs)
    -_bad_constant(value)
    -_keys(value, expected)
    -_json(data)
    +decode_control(data)
    -_validate_layout_shape(layout)
    -_number(value)
    +encode_control(envelope)
    +encode_reply(reply)
  }
  class C5["FragmentAssembler"] {
    -__init__(limit, timeout)
    +clear()
    +push(data, now)
    -_push(data, now)
  }
  class C6["CommandAssemblers"] {
    -__init__()
    +push_priority(data, now)
  }
```

## terra_rover/safety.py

[Source](https://github.com/Super-Yojan/Terra/blob/8695cf67992647c163142cf3f3b6d81aaaba4bd0/hardware/raspberry-pi/terra_rover/safety.py) · 19 explicit function/method definitions.

```mermaid
classDiagram
  direction TB
  class C0["OutputCommand"] {
    +id
    +value
  }
  class C1["OutputBatch"] {
    +commands
    +armed
    +arming
    +reason
    +fault
    +emergency_stop
  }
  class C2["SafetyController"] {
    -__init__(capabilities)
    -_disarm(reason)
    -_finite_time(value)
    -_clock(now)
    -_expire(now)
    +configure(layout, now)
    +connect(session, now)
    +disarm(reason, now)
    +set_status_subscribed(subscribed, now)
    +accept(frame, received_at, now)
    -_accept(frame, received_at, now)
    -_safe_frame(frame)
    +tick(now, hardware_gate)
    -_safe_outputs()
    +disconnect(now)
    +latch_fault(reason, now)
    +reset_fault(now)
    +reset_emergency_stop(now)
    +status(now)
  }
```

## terra_rover/usb_queue.py

[Source](https://github.com/Super-Yojan/Terra/blob/8695cf67992647c163142cf3f3b6d81aaaba4bd0/hardware/raspberry-pi/terra_rover/usb_queue.py) · 6 explicit function/method definitions.

```mermaid
classDiagram
  direction TB
  class C0["ModuleFunctions"] {
    <<module>>
    +access_queue(path, request)
    +main()
  }
  class C1["SshJournal"] {
    -__init__(host, key, known_hosts)
    +request(request)
    +pending(limit)
    +acknowledge(ids)
  }
```
