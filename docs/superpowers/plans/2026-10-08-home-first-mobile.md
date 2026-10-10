# Home-first Mobile Implementation Plan

**Goal:** Make rover and fleet connectivity the primary Home experience, with focused Settings and debugging destinations.
**Spec:** ../specs/2026-10-08-home-first-mobile-design.md
**Execution:** Inline; user requested “build”. Preserve existing uncommitted work and the current connection/lifecycle APIs.

1. Add Foundation-only typed connection presentation and tests: distinguish radio permissions/power, pairing, authentication, configuration readiness, user pauses, fleet setup/attempt/session, and overall summary without interpreting display strings.
2. Bridge real radio/coordinator facts into PhoneController. Persist confirmed rover display name, clear it on Forget, add fleet-only Retry, and apply validated saved fleet settings as one transaction. Do not reset Bluetooth for fleet Retry.
3. Replace the four-tab shell and timed splash with Home with lower-left Settings button. Build a stable Home showing separate rover/fleet sections, contextual actions, secondary details, and connected hardware Emergency Stop. Build dedicated fleet settings, rover setup, diagnostics, and Debug tools. Gate the legacy reusable debug sections by destination instead of scrolling a universal form.
4. Register new sources without regenerating the existing project. Run pure presentation/policy/joystick checks and unsigned iPhone/simulator builds. Inspect simulator Home, Settings, fleet editor, and Debug tools, including large text/dark appearance. Review lifecycle/settings/consent behavior independently.

Keep driving/maps/missions off Home. Do not imply fleet membership, tracking health, or armed outputs merely from transport sessions. Preserve sensor and connection state during navigation; leaving debug driving zeros/disarms. No rover redeployment required.

User refinement: retain the rover preview and actuator names; use Home and a lower-left Settings button rather than tabs. Further refinement: tracking ON for every ARGOS mode while connected, OFF when the fleet session ends. No manual tracking toggle in normal setup; show status and explicit failure Retry. Preserve router/controller/autonomy identity and disarmed output during sensor transitions.

Completed: typed states + tests, live controller bridge and display name persistence, focused navigation and sheets, transactional settings and fleet-only Retry, ARGOS-driven tracking lifecycle, simulator fixture screenshots and independent review. Device/simulator builds and connection/joystick checks run after final fixes. Physical camera/Bluetooth acceptance remains required.
