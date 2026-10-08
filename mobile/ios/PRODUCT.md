# Terra mobile product context

## Platform

Native iOS app built with SwiftUI, CoreBluetooth, ARKit/CoreMotion, and a shared Rust controller through UniFFI. Physical rover connections run on iPhone; Bevy controls are simulator-only.

## Confirmed primary job

Connect the phone to its rover and to the ARGOS fleet management dashboard. Make the two connection states and the next useful action clear on Home. Fleet operation belongs in ARGOS.

## Confirmed workflows

New rover pairing requires explicit consent. Reconnect to the remembered rover automatically, then connect to the saved fleet router. Configure router and hardware settings occasionally. Manual driving is rarely used and belongs in Debug tools. Home does not need maps or mission controls.

## Constraints

Preserve automatic connection/retry behavior, explicit arming, emergency-stop semantics, and disarmed reconnection. A router session is not proof of fleet membership or healthy tracking. Keep hardware and fleet transport status separate. Preserve existing settings, pairing preference, actuator layouts, simulator tools, and diagnostics.

## Audience

Rover operators connecting phone-mounted hardware to fleet management; this role is inferred from the established product workflow. No broader audience or marketing claims have been specified.

## Visual authority

Established Terra mark and name are real assets. Existing native iOS components and adaptive surfaces are the incumbent identity. The approved change concerns menu structure and operational clarity, not a requested rebrand.
