"""Generate docs/diagrams/vehicle-state-machine.excalidraw.

Run:
    python3 docs/diagrams/gen_vehicle_state_machine.py
    excalidraw render docs/diagrams/vehicle-state-machine.excalidraw \
        --out docs/diagrams/vehicle-state-machine.png --scale 2
"""
from pathlib import Path
from excal import (
    BLUE, BLUE_BG, GREEN, GREEN_BG, GREY, GREY_BG, INK, ORANGE, PURPLE,
    RED, RED_BG, YELLOW_BG, Scene,
)

PURPLE_BG = "#f3d9fa"
SOFT_WHITE = "#f8f9fa"


def build() -> Scene:
    s = Scene(seed=42)

    # ══════════════════════════════════════════════════════════════════════════
    # FRAME 1: Mission State Machine (Tasks)
    # ══════════════════════════════════════════════════════════════════════════
    f1 = s.frame(40, 40, 1400, 620, "1 · Mission State Machine (Tasks — crates/terra-av/src/tasks.rs)")

    s.text(
        70, 62,
        "Persistent autonomous mission lifecycle · Nothing on the rover clears a task (only a human replaces or clears it)",
        f1, size=16, color=GREY,
    )

    # State boxes
    # Idle (far left, vertically centered)
    s.box(85, 250, 175, 95, "Idle\n(no active task)\nTwist::STOP", f1,
          size=16, color=INK, backgroundColor=GREY_BG, strokeWidth=2)

    # Running (center-top)
    s.box(420, 175, 240, 115, "Running\nNavigator drives\n(Planner → Controller)", f1,
          size=17, color=GREEN, backgroundColor=GREEN_BG, strokeWidth=3)

    # Reached (right-top)
    s.box(820, 175, 215, 115, "Reached\nwithin tolerance\nTwist::STOP", f1,
          size=16, color=BLUE, backgroundColor=BLUE_BG, strokeWidth=2)

    # Paused (center-bottom-left)
    s.box(350, 425, 210, 100, "Paused\n(human paused)\nTwist::STOP · goal kept", f1,
          size=15, color=ORANGE, backgroundColor=YELLOW_BG, strokeWidth=2)

    # NeedsAttention (center-bottom-right)
    s.box(655, 425, 265, 100, "NeedsAttention(reason)\nTwist::STOP · goal kept\nnever auto-resumes", f1,
          size=15, color=RED, backgroundColor=RED_BG, strokeWidth=3)

    # Fault triggers callout (far right)
    s.rect(1085, 140, 325, 415, f1, strokeColor=RED, backgroundColor="#fff5f5",
           strokeWidth=2, strokeStyle="dashed")
    s.text(1108, 158, "Attention Reasons (Faults)", f1, size=18, color=RED)
    faults = [
        ("Estop", "phone or base e-stop engaged"),
        ("PoseLost", "untracked for > 2.0 s"),
        ("NoRoute", "global planner found no path"),
        ("Timeout", "exceeded controller time limit"),
        ("Stuck", "< 0.1 m & < 0.1 rad progress"),
        ("RemoteLost", "remote/cmd silent > 1.0 s\n(3.0 s startup grace)"),
    ]
    fy = 198
    for name, desc in faults:
        s.mono(1108, fy, f"• {name}", f1, size=14, color=RED)
        s.text(1124, fy + 20, desc, f1, size=13, color=INK)
        fy += 54 if "\n" not in desc else 68

    # ── Transitions ──────────────────────────────────────────────────────────
    # Idle -> Running (new Task)
    s.elbow([(260, 275), (340, 275), (340, 232), (420, 232)], f1, color=GREEN, width=2)
    s.text(278, 206, "new Task", f1, size=15, color=GREEN)

    # Running self-loop (same task.id hot-swap)
    s.elbow([(480, 175), (480, 118), (600, 118), (600, 175)], f1, color=PURPLE, width=2)
    s.text(445, 92, "same task.id → hot-swap planner / controller", f1, size=14, color=PURPLE)

    # Running -> Reached (top arrow)
    s.arrow((660, 210), (820, 210), f1, color=BLUE, width=2)
    s.text(692, 184, "goal reached", f1, size=14, color=BLUE)

    # Reached -> Running (bottom arrow)
    s.arrow((820, 258), (660, 258), f1, color=GREEN, width=2)
    s.text(672, 266, "pushed > 1.5× tol", f1, size=14, color=GREEN)

    # Running -> Paused (left branch down)
    s.arrow((475, 290), (435, 425), f1, color=ORANGE, width=2)
    s.text(392, 345, "Pause", f1, size=14, color=ORANGE)

    # Paused -> Running (up)
    s.arrow((495, 425), (520, 290), f1, color=GREEN, width=2)
    s.text(516, 355, "Resume", f1, size=14, color=GREEN)

    # Running -> NeedsAttention (down-right)
    s.arrow((595, 290), (745, 425), f1, color=RED, width=2)
    s.text(684, 336, "fault / e-stop", f1, size=14, color=RED)

    # NeedsAttention -> Running (up-left)
    s.arrow((705, 425), (560, 290), f1, color=GREEN, width=2)
    s.text(588, 382, "Resume\n(if no e-stop)", f1, size=13, color=GREEN)

    # Dashed pointer from NeedsAttention to Faults callout
    s.arrow((920, 475), (1085, 475), f1, color=RED, width=1.5, strokeStyle="dashed")

    # Clear rail back to Idle (from Paused, NeedsAttention, Reached)
    rail_y = 595
    s.line([(927, 290), (927, rail_y), (172, rail_y), (172, 345)],
           f1, arrow_end="arrow", strokeColor=GREY, strokeWidth=2)
    s.line([(455, 525), (455, rail_y)], f1, strokeColor=GREY, strokeWidth=2)
    s.line([(787, 525), (787, rail_y)], f1, strokeColor=GREY, strokeWidth=2)
    s.text(478, 568, "TaskCommand::Clear  (from any active state → Idle)", f1, size=14, color=GREY)

    # Persistence note above Idle (clear of the bottom Clear rail)
    s.rect(75, 118, 225, 78, f1, strokeColor=GREY, backgroundColor=SOFT_WHITE,
           strokeWidth=1.5, strokeStyle="dashed")
    s.text(90, 128, "Disk persistence\nActive task saved to file;\nreloaded after restart.",
           f1, size=13, color=GREY)

    # ══════════════════════════════════════════════════════════════════════════
    # FRAME 2: Per-Tick Arbiter & Fixed Safety Chain (Pipeline::tick)
    # ══════════════════════════════════════════════════════════════════════════
    f2 = s.frame(40, 710, 1400, 450, "2 · Per-Tick Arbiter & Fixed Safety Chain (Pipeline::tick — 20 Hz)")

    s.text(
        70, 732,
        "Every tick: pick ONE driver by priority, then ALWAYS run the fixed safety chain (nothing upstream can skip it)",
        f2, size=16, color=GREY,
    )

    # Left: 3 priority driver sources
    s.box(75, 785, 295, 78, "1 · Phone Goal  (highest)\nGoal::Velocity or Goal::Pose\n(human standing at rover)",
          f2, size=14, color=PURPLE, backgroundColor=PURPLE_BG, strokeWidth=2)

    s.box(75, 888, 295, 78, "2 · Base Teleop Stick\nremote.teleop twist\n(only when !tasks.is_driving())",
          f2, size=14, color=BLUE, backgroundColor=BLUE_BG, strokeWidth=2)

    s.box(75, 991, 295, 78, "3 · Tasks State Machine\ntask.twist from Frame 1\n(Twist::STOP unless Running)",
          f2, size=14, color=GREEN, backgroundColor=GREEN_BG, strokeWidth=2)

    # Arbiter diamond
    s.box(435, 872, 130, 110, "Who\ndrives?", f2, size=16, color=INK,
          backgroundColor=YELLOW_BG, shape="diamond", strokeWidth=2)

    # Arrows from 3 sources to arbiter
    s.elbow([(370, 824), (402, 824), (402, 912), (447, 912)], f2, color=PURPLE, width=2)
    s.arrow((370, 927), (435, 927), f2, color=BLUE, width=2)
    s.elbow([(370, 1030), (402, 1030), (402, 942), (447, 942)], f2, color=GREEN, width=2)

    # Fixed Safety Chain container
    s.rect(625, 790, 560, 265, f2, strokeColor=RED, backgroundColor="#fff9db",
           strokeWidth=2, strokeStyle="dashed")
    s.text(645, 804, "Fixed Safety Chain  (always runs on wanted twist)", f2, size=16, color=RED)

    # 3 safety stages
    s.box(648, 862, 155, 115, "vehicle.clamp\nwheel speed\nlimits\n(keeps radius)",
          f2, size=14, color=INK, backgroundColor="#ffffff", strokeWidth=2)

    s.box(830, 862, 155, 115, "limit_accel\nfree braking;\nsmooth ramp\n(dt ≤ 0.1 s)",
          f2, size=14, color=INK, backgroundColor="#ffffff", strokeWidth=2)

    s.box(1012, 862, 155, 115, "safety.check\nphone OR base\ne-stop →\nTwist::STOP",
          f2, size=14, color=RED, backgroundColor=RED_BG, strokeWidth=3)

    # Arbiter -> clamp -> limit_accel -> safety.check
    s.arrow((565, 927), (648, 927), f2, color=INK, width=2)
    s.text(572, 902, "wanted", f2, size=13, color=INK)
    s.arrow((803, 920), (830, 920), f2, color=INK, width=2)
    s.arrow((985, 920), (1012, 920), f2, color=INK, width=2)

    # Final output boxes (far right)
    s.box(1225, 835, 185, 88, "DriveCommand\ntarget + measured\n→ Pi motor loop",
          f2, size=14, color=GREEN, backgroundColor=GREEN_BG, strokeWidth=2)

    s.box(1225, 955, 185, 100, "link.publish\nPose · Depth · Map\nPlannedPath\nStatus · TaskStatus",
          f2, size=13, color=BLUE, backgroundColor=BLUE_BG, strokeWidth=2)

    s.elbow([(1167, 905), (1196, 905), (1196, 879), (1225, 879)], f2, color=GREEN, width=2)
    s.elbow([(1167, 940), (1196, 940), (1196, 1005), (1225, 1005)], f2, color=BLUE, width=2)

    # Bottom note in Frame 2
    s.text(
        75, 1105,
        "Note: Tasks::step() runs every tick even when Phone Goal or Teleop drives, so task status (reached, stuck, timeout) stays current.",
        f2, size=14, color=GREY,
    )

    return s


if __name__ == "__main__":
    out = Path(__file__).with_name("vehicle-state-machine.excalidraw")
    build().save(out)
    print(f"Wrote {out}")
