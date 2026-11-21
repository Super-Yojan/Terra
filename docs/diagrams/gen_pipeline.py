"""Generate pipeline.excalidraw: how the terra-av pipeline should run on TerraPhone,
with run-time controller selection from the base station (crates/terra-comms/API.md),
marked with what the code does today (LIVE / NOT WIRED / TODO).

    python3 docs/diagrams/gen_pipeline.py docs/diagrams/pipeline.excalidraw
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from excal import *  # noqa: E402,F403

S = Scene(seed=11)
box, text, mono, arrow, elbow, rect = S.box, S.text, S.mono, S.arrow, S.elbow, S.rect
PURPLE_BG = "#f3d9fa"

# ═════════════════════ A · runtime: loops, rates, hand-offs ═════════════════════
F = S.frame(0, 0, 1720, 620, "A · Runtime: loops, rates & the base station")
text(30, 24, "Runtime  ·  two loops on the phone, one lease on the rover, one link to the base", F, size=26)
text(30, 62, "The loops share only the latest DriveCommand. Every hop has its own timeout, so any stall ends in a stop.   Red dashed = what the code does today instead.",
     F, size=17, color=GREY)

box(30, 140, 220, 150, "Phone sensors\n\nARKit VIO pose\nLiDAR depth\nCore Motion gyro\nCore Location GPS", F,
    backgroundColor=GREY_BG)

rect(290, 105, 540, 225, F, strokeStyle="dashed", strokeColor=BLUE)
text(305, 112, "Planner loop · 20 Hz · plannerQueue", F, size=16, color=BLUE)
box(310, 165, 210, 90, "PhoneSensors\n.frame(estop)\n→ SensorFrame", F, backgroundColor=BLUE_BG, color=BLUE)
box(580, 155, 230, 110, "TerraAv.tick(frame, goal)\nPipeline::tick (Rust)\n→ DriveCommand\n(today: always 0)", F, backgroundColor=BLUE_BG,
    color=BLUE, size=17)
arrow((250, 215), (310, 210), F)
arrow((520, 210), (580, 210), F)

box(870, 165, 200, 90, "latest DriveCommand\n+ timestamp\n(NSLock)", F, backgroundColor=YELLOW_BG)
arrow((810, 210), (870, 210), F)
text(818, 180, "write", F, size=13, color=GREY)

rect(1110, 105, 300, 225, F, strokeStyle="dashed", strokeColor=GREEN)
text(1125, 112, "Velocity loop · 50 Hz · velocityQueue", F, size=16, color=GREEN)
box(1130, 150, 260, 140, "VelocityController\n\nfeedforward(target)\n+ kp·(target − measured)\nskid-steer mix → WheelCommand\nstop if target age > 0.3 s",
    F, backgroundColor=GREEN_BG, color=GREEN, size=15)
arrow((1070, 210), (1130, 210), F)
text(1078, 180, "read", F, size=13, color=GREY)

box(1450, 140, 240, 150, "rover-pi · DriveRelay\n\nholds efforts for 0.25 s\nthen neutral\n→ ESC / motor outputs", F,
    backgroundColor=GREY_BG, size=15)
arrow((1390, 215), (1450, 215), F, width=3)
text(1414, 228, "BLE", F, size=14, color=GREY)

# phone inputs
box(300, 400, 220, 80, "Phone e-stop\n(also stops motors at once)", F, backgroundColor=RED_BG, color=RED, size=15)
arrow((410, 400), (415, 255), F, color=RED)
text(423, 318, "frame.estop", F, size=13, color=RED)
box(545, 400, 225, 80, "Phone joystick (Test drive)\n→ Goal.velocity", F, backgroundColor="#fff", size=15)
arrow((645, 400), (680, 265), F)
text(668, 318, "goal", F, size=13, color=GREY)
# today: the stick skips terra-av and goes straight to BLE
elbow([(645, 480), (645, 575), (1705, 575), (1705, 260), (1690, 260)], F, color=RED, strokeStyle="dashed")
text(665, 583, "TODAY · velocityTick sees Goal.velocity → joystickToMotors() → BLE.\n"
     "Skips terra-av, VelocityController and the remote e-stop (the phone e-stop only disables the stick).", F, size=13, color=RED)
box(30, 400, 230, 80, "Send world map (button)\nARWorldMap + feature pts\n→ link.publish_world_map", F,
    backgroundColor=PURPLE_BG, color=PURPLE, size=14)

# link to the base station
box(800, 395, 270, 125, "RoverLink  (terra-comms) · LIVE\n\ncommands(): estop · teleop\ntasks + task/cmd → rejected\npublish pose · depth · status\n+ world_map", F,
    backgroundColor=PURPLE_BG, color=PURPLE, size=15)
arrow((870, 395), (790, 265), F, color=PURPLE, both=True)
text(845, 318, "once per tick", F, size=13, color=PURPLE)
box(1270, 385, 420, 140, "Base station UI  (BaseLink) · NOT IN REPO\nterra-comms examples/demo.rs stands in\n\nplanned: controller dropdown + form\nfrom Capabilities, click goal → send(Task)", F,
    backgroundColor="#fff", size=15, strokeStyle="dashed")
arrow((1270, 425), (1070, 425), F, color=PURPLE)
text(1090, 388, "estop · teleop (used)\ntask · cmd (rejected)", F, size=13, color=PURPLE)
arrow((1070, 480), (1270, 480), F, color=GREY)
text(1090, 488, "capabilities (once) · pose\ndepth · status · world_map\ntask/status (Idle + reject)", F, size=13, color=GREY)

# ═════════════════════ B · one Pipeline::tick ═════════════════════
OY = 670
BHT = 1640
F = S.frame(0, OY, 1060, BHT, "B · One Pipeline::tick (Rust, 20 Hz)")
text(30, OY + 24, "Pipeline::tick(frame, goal) → DriveCommand", F, size=26)
text(30, OY + 62, "purple = chosen per task from the base station   ·   red = fixed safety chain   ·   arrows: type + frame",
     F, size=16, color=GREY)
text(30, OY + 84, "today:", F, size=15, color=GREY)
text(90, OY + 84, "LIVE", F, size=15, color=GREEN)
text(135, OY + 84, "runs in tick", F, size=15, color=GREY)
text(260, OY + 84, "NOT WIRED", F, size=15, color=ORANGE)
text(365, OY + 84, "built, tick doesn't call it", F, size=15, color=GREY)
text(590, OY + 84, "TODO", F, size=15, color=RED)
text(640, OY + 84, "todo!() or missing", F, size=15, color=GREY)

X, W, BH = 240, 470, 80
CX = X + W / 2


def stage(y, label, impls=None, bg="#fff", color=INK, h=BH, size=16):
    box(X, y, W, h, label, F, backgroundColor=bg, color=color, size=size)
    if impls:
        text(X + W + 35, y + h / 2 - 11, impls, F, size=15, color=GREY)


STATUS = {"LIVE": GREEN, "NOT WIRED": ORANGE, "TODO": RED}


def tag(y, status, note="", h=BH, top=False):
    s = status + ("\n" + note if note else "")
    text(128, y + 4 if top else y + h / 2 - (11 if not note else 20), s, F, size=13, color=STATUS[status])


def down(y0, y1, label=None, color=INK):
    arrow((CX, y0), (CX, y1), F, color=color)
    if label:
        text(CX + 12, (y0 + y1) / 2 - 10, label, F, size=14, color=PURPLE)


y = OY + 120
stage(y, "SensorFrame + phone Goal, then link.commands()\nestop |= remote · no phone goal → base teleop", bg=GREY_BG)
tag(y, "LIVE")
down(y + BH, y + 130)

y += 130
task_y = y
stage(y, "0 · take_task / take_task_command → set_task\nvalidate, build, swap BETWEEN ticks (even moving)",
      bg=PURPLE_BG, color=PURPLE)
tag(y, "TODO", "tasks rejected")
box(X + W + 35, y - 20, 300, 120,
    "Controller registry · controllers()\n\nteleop · drive_pose · dwa · remote\neach: id · version · params[] · build()\nalso published as Capabilities",
    F, backgroundColor="#fff", color=PURPLE, size=14, strokeStyle="dashed")
arrow((X + W + 35, y + 40), (X + W, y + 40), F, color=PURPLE)
down(y + BH, y + 130)

y += 130
stage(y, "1 · StateEstimator.update(frame, vehicle)\nVIO pose: camera → body via Mount",
      "Vio  |  Localization (todo)   (toml only)", bg=BLUE_BG, color=BLUE)
tag(y, "LIVE", "Vio")
down(y + BH, y + 130, "Option<State>")

y += 130
S.box(CX - 110, y, 220, 90, "tracked\nstate?", F, shape="diamond", size=16, backgroundColor=YELLOW_BG)
none_y = y + 45
box(X + W + 35, y - 5, 320, 100, "TODAY tick stops here:\ncommands → 1 → 7, then returns\ntarget 0, measured None.\nStages 2–6 are never called.", F,
    backgroundColor=RED_BG, color=RED, size=14, strokeStyle="dashed")
down(y + 90, y + 140, "Some(state)")
box(25, none_y - 40, 180, 80, "None →\ntarget = 0\nmeasured = None", F, backgroundColor=RED_BG, color=RED, size=15)
arrow((CX - 110, none_y), (205, none_y), F, color=RED)
NONE_X = 115

y += 140
stage(y, "2 · Mapper.update(frame, state) → &Map\ndepth: camera → body (Mount) → world (pose)",
      "NoMap (todo)  |  OccupancyGrid", bg=GREEN_BG, color=GREEN)
tag(y, "NOT WIRED", "OccupancyGrid")
down(y + BH, y + 130, "Map   obstacles, world frame")

y += 130
plan_y = y
stage(y, "3 · PathPlanner.plan(state, map, task.goal, vehicle)\ntask.planner, else the toml one · re-run on new goal/map",
      "StraightLine  |  HybridAStar", bg=PURPLE_BG, color=PURPLE, size=15)
tag(y, "TODO", "both todo!()")
down(y + BH, y + 130, "Path   poses, world frame (cached)")

y += 130
ctrl_y = y
stage(y, "4 · active controller.follow(state, map, goal, path, vehicle)\nthe LocalPlanner named in the task, with its params",
      "Teleop | DrivePose | Dwa | Remote", bg=PURPLE_BG, color=PURPLE, size=15)
tag(y, "TODO", "all todo!(),\nno Remote")
# registry feeds both purple stages
elbow([(X + W + 185, task_y + 100), (X + W + 185, task_y + 118), (X + W + 18, task_y + 118),
       (X + W + 18, ctrl_y + 40), (X + W, ctrl_y + 40)], F, color=PURPLE, strokeStyle="dashed")
arrow((X + W + 18, plan_y + 40), (X + W, plan_y + 40), F, color=PURPLE, strokeStyle="dashed")
down(y + BH, y + 130, "Twist   body frame")

y += 130
chain_top = y
stage(y, "5 · vehicle.clamp(twist) + acceleration limits\nwheel ≤ max_wheel_speed · Δv, Δw per tick bounded",
      bg=RED_BG, color=RED)
tag(y, "NOT WIRED", "clamp only;\nno accel limit")
down(y + BH, y + 130, "Twist   reachable, smooth")

y += 130
safety_y = y
stage(y, "6 · SafetyGate.check(twist, frame)\nestop → zero twist.  Runs on every path.", bg=RED_BG, color=RED)
tag(y, "TODO", "todo!()", top=True)
elbow([(NONE_X, none_y + 40), (NONE_X, safety_y + 40), (X, safety_y + 40)], F, color=RED)
chain_bot = y + BH
# bracket for the fixed chain
S.line([(X + W + 20, chain_top), (X + W + 35, chain_top), (X + W + 35, chain_bot), (X + W + 20, chain_bot)], F,
       kind="line", strokeColor=RED, strokeWidth=2)
text(X + W + 50, chain_top + 45, "fixed chain\na controller can't skip it,\nwhatever it outputs", F, size=15, color=RED)
down(y + BH, y + 130)

y += 130
stage(y, "DriveCommand { target, measured: Some(state.twist) }\n→ latest, read by the 50 Hz velocity loop", bg=YELLOW_BG)
tag(y, "TODO", "always 0,\nmeasured None")
down(y + BH, y + 130)

y += 130
stage(y, "7 · link.publish(Pose, Depth, Status · TaskStatus if changed)\nqueue only, newest wins, never blocks the tick",
      bg=PURPLE_BG, color=PURPLE, size=15)
tag(y, "LIVE")

box(X + W + 35, OY + BHT - 150, 300, 110,
    "Shared, read-only every tick\nVehicleModel (DiffDrive): estimation,\nplanners, clamp\nMount: Vio + OccupancyGrid",
    F, backgroundColor=GREY_BG, size=14, strokeStyle="dashed")

# ═════════════════════ C · tasks, research workflow, fail-safes, gaps ═════════════════════
OX = 1100
F = S.frame(OX, OY, 620, BHT, "C · Tasks, research workflow, fail-safes")
text(OX + 30, OY + 24, "A task = where + how   (design, not built)", F, size=24)
mono(OX + 30, OY + 75,
     "Task {\n"
     "  id,\n"
     "  goal:       Stop | Pose { x, y, yaw? }   where\n"
     "              | Gps { lat, lon, heading? }\n"
     "              + tolerance (m)\n"
     "  controller: \"dwa\"                       how\n"
     "  params:     { horizon: 2.0, ... }\n"
     "  planner:    None | \"hybrid_astar\"\n"
     "}\n\n"
     "PERSISTENT: only a human replaces or\n"
     "clears a task. Same id = update: swap the\n"
     "controller while moving, goal kept.\n\n"
     "task/status: Idle · Running · Reached · Paused\n"
     "  NeedsAttention(timeout | stuck | estop |\n"
     "    pose lost | restarted ...) → hold still,\n"
     "    goal kept until a human acts\n"
     "  Reached + pushed out → drives back\n"
     "    (controller's returns_to_goal)\n"
     "task/cmd:    Resume · Pause · Clear\n\n"
     "Gps goal → converted to Pose with the\n"
     "estimator's GPS anchor, unless the\n"
     "controller lists goals: [Gps]", F, size=14)

text(OX + 30, OY + 555, "Adding a controller (research)", F, size=24, color=PURPLE)
mono(OX + 30, OY + 600,
     "1  planning/local/my_ctrl.rs\n"
     "     impl LocalPlanner for MyCtrl\n"
     "2  one ControllerSpec in controllers():\n"
     "     id, version, params[], build()\n"
     "3  rebuild the rover → it shows up in\n"
     "     Capabilities → UI dropdown + form\n\n"
     "No rebuild: pick \"remote\" and answer\n"
     "remote/state with remote/cmd from any\n"
     "language (1 s lease; prototyping only)", F, size=14, color=INK)

text(OX + 30, OY + 845, "Fail-safes", F, size=24)
mono(OX + 30, OY + 895,
     "1  E-stop     phone or remote (latched);\n"
     "              SafetyGate zeroes the twist\n"
     "2  No pose    target 0, measured None\n"
     "3  Bad task   rejected; current task untouched\n"
     "4  Attention  timeout / stuck / estop → hold\n"
     "              still; goal stays for a human\n"
     "5  Link lost  teleop / remote/cmd expire after\n"
     "              1 s → twist 0. Autonomous tasks\n"
     "              keep going until Reached\n"
     "6  Config     invalid toml → every tick stops\n"
     "7  Stale      target age > 0.3 s → stop\n"
     "8  BLE gap    no command for 0.25 s → neutral", F, size=14)

text(OX + 30, OY + 1190, "Gaps today (code ≠ this design)", F, size=24, color=RED)
mono(OX + 30, OY + 1240,
     "• tick runs commands → Vio → publish only;\n"
     "  stages 2–6 never called → target 0\n"
     "• todo!(): SafetyGate, Teleop, DrivePose,\n"
     "  Dwa, StraightLine, HybridAStar, NoMap,\n"
     "  Localization, DiffDrive step()/limits()\n"
     "• Built, unused: OccupancyGrid, clamp()\n"
     "• No registry / Task / set_task: tasks and\n"
     "  task cmds rejected, TaskStatus = Idle;\n"
     "  Capabilities = the toml's 1 controller\n"
     "• No acceleration limiter after clamp\n"
     "• Stick → joystickToMotors() → BLE, skips\n"
     "  terra-av, VelocityController, remote\n"
     "  e-stop (only the 0.25 s BLE lease holds)\n"
     "• No base station app (demo.rs only)\n"
     "• [mount] roll sign untested", F, size=14, color=RED)

text(OX + 30, OY + BHT - 80, "Design: crates/terra-comms/API.md\n"
                             "Code: crates/terra-av/src/pipeline.rs, comms.rs,\n"
                             "apps/ios/TerraPhone/App/RoverLoop.swift", F, size=13, color=GREY)

S.save(sys.argv[1] if len(sys.argv) > 1 else "pipeline.excalidraw")
