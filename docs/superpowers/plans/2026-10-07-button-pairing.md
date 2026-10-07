# Headless Button Pairing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ship a rebuilt Pi executable that enrolls its first phone after one Fusion HAT button hold, with LED feedback and no SSH or known peer address.

**Architecture:** A background lifecycle waits for a physical hold on unowned devices, runs one bounded setup-only BLE session, then starts the existing owner-only peripheral. Hardware polling, ownership persistence and pairing authorization have independent modules and tests. The terminal numeric-confirmation path remains available for developers.

**Tech Stack:** Python asyncio, dbus-next 0.2.3, BlueZ, Fusion HAT sysfs, Swift/CoreBluetooth, systemd, Nuitka 4.2.2.

**Spec:** [Button pairing design](../specs/2026-10-07-button-pairing-design.md)

## Global Constraints

- One initial three-second hold; no second press and no Pi numeric confirmation.
- Debounce for 100 ms, poll approximately every 50 ms, use monotonic time.
- Require release after startup-held button and before each new attempt.
- A physically opened pairing window lasts at most 60 seconds.
- Name is `terra-` plus six random uppercase ASCII letters/digits, persisted across reboot/retries.
- Waiting LED off; setup 1 Hz; pending candidate 4 Hz; success three flashes then steady on; owned hardware fault repeated double flash.
- Only one approved candidate, one GATT application and one advertisement at a time.
- Setup exposes no motor controls and creates no actuator backend; motor gate is not needed for first-owner pairing.
- Existing owner admission, motor-gate requirements and explicit arming remain in force after enrollment.
- Invalid/insecure owner files fail closed; long holds never replace an existing owner.
- ARM64 Raspberry Pi OS Bookworm or newer; no new GPIO/evdev/audio dependencies.

## Review Focus

- A stuck button at boot must not authorize enrollment; test release-before-hold.
- A phone disconnecting between bond completion and enrollment must not become owner; test live-candidate/status-read requirements.
- An existing but malformed owner file must not reopen first-owner setup; test validation separately from missing-file handling.
- Interrupted persistence and BlueZ cleanup must not expose motion or overwrite ownership; test atomic save and partial-registration failures.
- iOS can retain a cached peripheral name; test advertised local-name preference and retain identifier-based reconnect behavior.

## Task 1: Hardware input and persistent identity

**Files:** Create `hardware/raspberry-pi/terra_rover/onboarding.py` and `hardware/raspberry-pi/tests/test_onboarding.py`.

**Interfaces:**
- `HoldDetector.update(pressed: bool, now: float) -> bool`: return true once per released/debounced three-second hold; initialize release-required.
- `HatIndicators(button_path: Path, led_path: Path)`: strict `read_button() -> bool` and `write_led(on: bool) -> None`.
- `load_owner(path: Path) -> dict | None`: validate identity keys, address/type and private file permissions; return None only when genuinely absent.
- `save_owner(path: Path, identity: dict) -> None`: exclusive enrollment, private atomic persistence and fsync.
- `load_device_name(path: Path) -> str`: validate/reuse saved name or persist a newly generated six-character identifier.

- [ ] Write tests for bounce, exact hold threshold, held-at-boot, one event per release, missing/invalid hardware values, retained names, invalid owners, and interrupted/existing-owner persistence.
- [ ] Run `PYTHONPATH=hardware/raspberry-pi hardware/raspberry-pi/.venv/bin/python -m unittest discover -s hardware/raspberry-pi/tests -p test_onboarding.py -v`; confirm failures are the missing implementation.
- [ ] Implement the interfaces using strict sysfs tokens, monotonic state and private temporary files; never create PWM objects here.
- [ ] Run the task tests and existing packaging tests; fix failures before moving on.

## Task 2: Bounded button-authorized BLE enrollment

**Files:** Create `hardware/raspberry-pi/terra_rover/pairing.py` and `hardware/raspberry-pi/tests/test_pairing.py`; consume existing BLE service/advertisement definitions.

**Interfaces:**
- `PairingSession(deadline: float, preexisting_bonds: set[str])`: claim one candidate, record agent approval/status read, validate deadline and connected/paired/bonded identity, and invalidate on cancellation.
- `ButtonPairingAgent(session: PairingSession)`: implement headless authorization and cancellation callbacks; all callbacks validate the same session/candidate.
- `ButtonSetupStatus(session: PairingSession)`: encrypted setup-only read; no writable/motion characteristics.
- `pair_owner(args, indicators: HatIndicators, name: str) -> None`: register the bounded setup application, persist an eligible owner using Task 1, and complete all cleanup before returning.

- [ ] Write tests with controlled D-Bus/device state for concurrent candidates, expired/stale callbacks, missing authorization callback, disconnect before completion, preexisting bond rejection, encrypted-read requirement, and cancellation.
- [ ] Test partial registration and bus loss: every cleanup is attempted, no preexisting bonds are removed, and no actuator objects are created.
- [ ] Run the task tests to establish expected failures.
- [ ] Implement `NoInputNoOutput` enrollment, pairability/discoverability deadlines and slow/fast LED feedback. Cleanup new failed candidate bonds only; disable pairing on every exit.
- [ ] Run all Pi tests. Preserve the original `DisplayYesNo` developer setup behavior.

## Task 3: Background lifecycle and customer phone flow

**Files:** Modify `hardware/raspberry-pi/terra_rover/__main__.py`, `hardware/raspberry-pi/terra_rover/onboarding.py`, `mobile/ios/TerraPhone/BluetoothLink.swift`, and `mobile/ios/TerraPhone/ActuatorLayoutView.swift`; create `hardware/raspberry-pi/tests/test_lifecycle.py`.

**Interfaces:**
- `run_customer_service(args) -> None`: owned-device normal service, or unowned release/hold waiting → Task 2 pairing → normal service. Each transition awaits cleanup before registering the next application.
- CLI `--button-pairing`, `--button-file`, `--led-file`, and `--device-name-file`; defaults are the pinned HAT sysfs paths and `/var/lib/terra-rover/device-name.json`.
- The normal backend factory runs only after valid ownership; missing owner produces waiting rather than a traceback in customer mode.
- TerraPhone prefers advertised local name, explains the hold/pairing flow, and offers explicit reconnect after automatic Pi service transition.

- [ ] Write lifecycle tests proving no backend during waiting/setup; no motion on enrollment/reconnect; existing ownership bypasses enrollment; invalid ownership fails closed; timeout/hardware failure requires a new hold; cancellation cleans up.
- [ ] Establish failing tests before integrating the lifecycle and signal handling.
- [ ] Implement lifecycle and graceful cancellation, preserving developer CLI and normal motor interlocks.
- [ ] Update phone setup guidance and discovery name handling without automatically arming or accepting stale connection callbacks.
- [ ] Run all Pi tests, `./scripts/check-swift.sh`, and the available iOS build check. Report platform blockers explicitly.

## Task 4: Installable boot service and rebuilt release

**Files:** Modify `packaging/terra-rover.service`, `packaging/rover/install.sh`, `packaging/rover/terra-rover.env`, `hardware/raspberry-pi/tests/test_packaging.py`, `docs/hardware/INSTALL.md`, `docs/hardware/BLUETOOTH.md`, and hardware evidence documentation.

**Interfaces:** Installed service invokes customer mode with HAT defaults; installer enables/starts first-owner background waiting after dependency checks, preserves owned-device state/configuration, and does not authorize pairing itself. Existing build recipe includes new modules/source automatically.

- [ ] Add installer tests for retained device identity/configuration and the new customer-service defaults; exercise live account/service-manager boundaries in an isolated Linux container.
- [ ] Establish expected failures, then update installer/service configuration and customer instructions.
- [ ] Run all Pi tests, `cargo test --workspace --locked --offline`, shell/Python syntax checks and `git diff --check`.
- [ ] Run `docker buildx build --platform linux/arm64 -f packaging/rover/Dockerfile --output type=local,dest=hardware/raspberry-pi/dist .`; require successful onefile compilation and clean no-Python runtime/staged-install checks.
- [ ] Verify the installed systemd unit, artifact checksums, and bundled source/configuration in a disposable ARM64 Linux container.
- [ ] Request an independent code review; fix actionable issues and rerun affected checks.
- [ ] Record the final archive path/hash and executed checks. Explicitly mark Pi/HAT button/LED, BlueZ/iPhone pairing and physical motor behavior unverified without those devices.

## Delivery

Provide the rebuilt archive and concise Pi upgrade steps. Keep implementation
changes separate from unrelated existing `Cargo.lock` and developer dependency
changes. Do not publish, merge or alter the currently installed customer Pi remotely.

Physical acceptance is the full button → blinking LED → app discovery → pairing →
automatic normal service → reboot/reconnect flow, including timeout and competing
phone rejection. It remains a separate device test when no Pi/iPhone is accessible.
