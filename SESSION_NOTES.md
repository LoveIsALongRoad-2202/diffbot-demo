# DiffBot Project — Status & Run Guide (as of this session)

## Current status: WORKING ✅
- ROS2 Humble + Gazebo simulation: fully working, keyboard-drivable
- Arduino L298N car: fully working, on/off (bang-bang) control, no PWM
- ROS2 <-> Arduino serial bridge: working, drives real car from same
  keyboard as Gazebo sim
- Known limitation: `teleop_twist_keyboard` cannot detect key-release,
  only key-press. So the robot (both sim and real) keeps moving until
  you press a different key (like `k` for stop) — it does NOT stop the
  instant you lift your finger off `i`. This is a limitation of terminal
  keyboard input, not a bug in our code.

## Next planned step (not done yet)
Build a browser-based teleop control (HTML page with arrow-key buttons)
using rosbridge, which DOES support proper key-down/key-up events, to
get true "moves only while held, auto-stops on release" behavior.

---

## HARDWARE — actual wiring (confirmed working)

```
L298N IN1 -> Arduino D7   (left side direction A)
L298N IN2 -> Arduino D8   (left side direction B)
L298N IN3 -> Arduino D9   (right side direction A)
L298N IN4 -> Arduino D10  (right side direction B)
L298N ENA -> L298N 5V pin (jumper wire, board-to-board — NOT to Arduino)
L298N ENB -> L298N 5V pin (jumper wire, board-to-board — NOT to Arduino)
L298N GND -> Arduino GND  (common ground)
L298N 12V -> battery pack + (2x 18650 in series, ~7.4-8.4V)
L298N GND (power terminal) -> battery pack -
```

Arduino connects to PC as **`/dev/ttyUSB0`** (not ttyACM0 — this board
uses a CH340-style USB-serial chip). Check this each session:
```bash
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

Arduino firmware file: `diffbot_final.ino` (already uploaded to the
board — you don't need to re-upload unless you change the code).
Serial protocol: `L:<val>,R:<val>\n`, sign-only (no PWM), auto-stops
after 500ms with no new command (safety watchdog).

---

## FULL STARTUP SEQUENCE — run every time after reboot

### Terminal 1 — Gazebo simulation
```bash
cd ~/diffbot_ws
ros2 launch diffbot_sim gazebo_sim.launch.py
```
Wait ~20-30 sec. Confirm Gazebo window opens with the 4-wheel robot
visible on the ground plane, and RViz2 opens alongside it.

### Terminal 2 — Real Arduino connection (serial bridge)
1. Plug in Arduino via USB. Connect the battery to the L298N.
2. **Make sure Arduino IDE's Serial Monitor is CLOSED** — if it's open,
   the port will be busy and the bridge node will fail to connect.
3. Check the port:
   ```bash
   ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
   ```
4. If you get "Device or resource busy" when starting the bridge below,
   run this first:
   ```bash
   sudo fuser -k /dev/ttyUSB0
   ```
5. Start the bridge node:
   ```bash
   cd ~/diffbot_ws
   python3 serial_bridge/serial_bridge_node_onoff.py --ros-args -p serial_port:=/dev/ttyUSB0
   ```
   Wait for this exact line before proceeding:
   ```
   [INFO] ... Connected to Arduino on /dev/ttyUSB0
   ```

### Terminal 3 — Keyboard teleop (drives BOTH sim and real car together)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p repeat_rate:=10.0
```
(We use `repeat_rate:=10.0` instead of the `teleop.launch.py` file
because it continuously republishes the last command at 10Hz — this
keeps the real Arduino's 500ms safety timeout from stopping the motors
mid-command. Without this flag, the real car stops after ~1 second
even while you're still driving.)

**Controls** (click into the terminal/xterm window first so it has
keyboard focus):
```
   u    i    o
   j    k    l
   m    ,    .
```
- `i` = forward, `,` = backward
- `j` = turn left in place, `l` = turn right in place
- `k` = STOP (you must press this explicitly — releasing `i` alone does
  nothing, see "known limitation" above)
- `q` / `z` = increase / decrease speed step (doesn't matter much for
  our on/off hardware, but affects Gazebo's simulated speed)

---

## Quick sanity checks (optional, if something seems off)

Check /cmd_vel is actually being published:
```bash
ros2 topic echo /cmd_vel
```

Check simulated odometry (Gazebo, ground truth):
```bash
ros2 topic echo /odom
```

Check real car's dead-reckoning ESTIMATE (not measured, no encoders,
will drift — see earlier explanation in project):
```bash
ros2 topic echo /odom_estimate
```

Check Arduino is alive and talking, bypass ROS entirely (only do this
when the bridge node in Terminal 2 is NOT running, since only one
program can hold the serial port at a time):
```bash
# close bridge node first (Ctrl+C in Terminal 2), then:
sudo fuser -k /dev/ttyUSB0   # just in case
```
Then open Arduino IDE Serial Monitor at 115200 baud and type:
```
L:100,R:100
```
Should reply `OK:100,100` and wheels should spin.

---

## Files in this project

```
diffbot_ws/
├── README.md                          <- original setup instructions
├── SESSION_NOTES.md                   <- this file
├── src/diffbot_sim/                   <- ROS2 Gazebo package
│   ├── urdf/diffbot.urdf.xacro
│   ├── launch/gazebo_sim.launch.py
│   ├── launch/teleop.launch.py        <- (superseded by repeat_rate cmd above)
│   ├── worlds/empty_arena.world
│   └── config/, rviz/, meshes/        <- placeholders, not yet used
├── arduino/
│   └── diffbot_l298n/
│       └── diffbot_final.ino          <- CURRENT working firmware
└── serial_bridge/
    └── serial_bridge_node_onoff.py    <- CURRENT working bridge node
```

---

## TOMORROW'S TODO: Browser-based hold-to-stop teleop

Plan: build an HTML page with on-screen buttons (or arrow key
listeners using proper `keydown`/`keyup` JS events), connect to ROS2
via `rosbridge_suite` (WebSocket bridge), publish `/cmd_vel` only while
a key/button is actively held, and publish zero-Twist immediately on
release. This will need:
```bash
sudo apt install ros-humble-rosbridge-suite
```
We'll wire this up next session — not done yet.
