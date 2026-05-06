# Fork Butler Bot

Autonomous butler robot built for MIT 2.120 (Spring 2026). The robot uses a Mecanum-wheel base, a 2-DOF Parallel linkage arm, and a Jetson Nano w/ two subordiante ESP32 ProS to autonomously navigate to AprilTag-marked targets, pick up dish racks, and deliver them — with person detection and Xbox controller manual override at any time.

---

## System Architecture

```
Xbox Controller (Bluetooth)
        │
        ▼
  Jetson Nano (Python 50 Hz)
  ├── simple_autonomy_controller.py   ← primary loop
  │     ├── AprilTag detection (camera)
  │     ├── Person detection (YOLOv8)
  │     ├── Mission sequencer
  │     └── GUI / debug overlay
  │
  ├── USB Serial /dev/tty*  ──►  Drive ESP32
  │     lx/ly/yaw commands            ├── Mecanum velocity PIDs (200 Hz)
  │     ◄── IMU + encoder telemetry   └── IMU / encoder streaming
  │
  └── USB Serial /dev/tty*  ──►  Arm ESP32
        lift/grip setpoints            ├── Lead-lag position control (2 kHz)
                                       └── Encoder feedback
```

---

## Running Butler bot to keep from becoming But Bot

This walkthrough describes what happens when a user turns the robot on and runs a mission end-to-end.

### 1. Startup
Connect both ESP32s to the Jetson via USB and pair the Xbox controller over Bluetooth. Launch the primary controller:
```bash
python -m Jetson.main.simple_autonomy_controller
```
The Jetson opens both serial ports, waits 2 seconds for the ESP32s to boot, then starts background reader threads for IMU and encoder telemetry. The camera opens, the AprilTag detector initializes, and the YOLOv8 person detection model loads. The OpenCV window appears showing the live camera feed with the mission sidebar on the right.

### 2. Calibrate Heading (if needed)
Point the robot in its intended forward direction and press `z` in the OpenCV window to zero the IMU yaw. This saves the offset to `yaw_offset.json` so it persists across sessions. Press `l` on subsequent runs to reload it. All heading control during missions uses this offset so that 0° always means the robot's calibrated forward.

### 3. Select and Launch a Mission
Use the **D-pad up/down** to scroll through the mission list in the sidebar. Each mission is a numbered list of steps (e.g. drive to tag, move arm, turn in place). Press **A or X** to start the highlighted mission from step 1, or use **D-pad left/right** (or type a step number) to begin from a specific step — useful for debugging individual steps without re-running the whole sequence. Double-clicking a mission name in the GUI window also launches it.

### 4. Navigation — Long Range (ZVU Dead Reckoning)
When a `drive_tag` or `drive_arm` step starts, the robot is likely too far from the target for reliable AprilTag pose estimation. The **Zero Velocity Update (ZVU)** system handles this:

- The robot **stops** and holds its heading with the IMU for 0.35 s so vibrations die. It then samples the tag pose — a stationary measurement is far less noisy than one taken while moving.
- The resulting drive direction `(lx, ly)` and the current IMU heading are **frozen**.
- The robot drives on those frozen commands for 2.5 s, holding heading precisely with a live IMU PD loop. No tag tracking occurs during this leg.
- After 2.5 s it stops again for another stationary fix and repeats. Each fix corrects any drift accumulated during the dead-reckoning leg.

Throughout this phase, **person detection** runs on the camera. If a person enters the frame the drive commands are capped to a slow speed; if they come too close all drive output is zeroed until they clear.

### 5. Navigation — Close Range (Live Tag Tracking)
Once the tag is within 1.75 m, the robot switches permanently to **live AprilTag tracking**. Each loop it computes:
- **Forward error** = current distance to tag − `stop_dist`
- **Lateral error** = tag pixel centroid offset from image center (normalized)

P-gain controllers on both axes drive `lx`, `ly`, and `yaw` commands each tick. The step completes once both errors stay within tolerance for 0.4 s continuously.

### 6. Arm Movements
Steps of type `drive_arm` move the arm simultaneously with driving. Steps of type `set_arm` move the arm while the robot holds position. The sequence runner slews lift and grip setpoints toward their targets at a fixed rate each tick, preventing sudden jerks. The arm step is considered done when both joints are within 0.05 rad of their targets.

### 7. Mission Completion
When the final step completes the sequencer prints a completion message and the controller returns automatically to **MANUAL** mode — full joystick authority is restored. Pressing **B** at any point during a mission immediately cancels it and returns to manual.

---

## Serial Communication Protocol

Both ESPs communicate with the Jetson over USB serial at **115,200 baud** using a simple `key:value;` format.

| Direction | Format | Example |
|-----------|--------|---------|
| Jetson → Drive ESP | `lx:X;ly:X;yaw:X;\n` | `lx:0.500;ly:0.000;yaw:-0.200;\n` |
| Jetson → Arm ESP | `lift:X;grip:X;\n` | `lift:2.930;grip:1.850;\n` |
| Drive ESP → Jetson | `IMU:roll:X;pitch:X;yaw:X;rollRate:X;pitchRate:X;yawRate:X;\n` | streamed at 50 Hz |
| Drive ESP → Jetson | `ENC:fl:X;bl:X;fr:X;br:X;\n` | wheel velocities at 50 Hz |

All `lx`, `ly`, `yaw` values are normalized to `[-1, 1]`. IMU angles are in radians; arm setpoints are in radians. Background reader threads on the Jetson parse every incoming line and store data in thread-safe dicts (`_imu_data`, `_enc_data`).

---

## Python Files (Jetson Nano)

### `robot_controller.py` — Manual Debug Controller

A stripped-down control loop used during hardware bring-up and manual testing. It bridges the Xbox controller inputs directly to the two ESP32 serial ports at 50 Hz with no autonomous logic.

**Primary functions:**

- **`compute_drive_command(lx, ly, yaw)`** — Applies a deadband (±0.1) and formats the normalized stick values into the `lx:X;ly:X;yaw:X;\n` serial string sent to the drive ESP.

- **`step_arm_setpoints(lift_sp, grip_sp, lt_raw, rt_raw, lb_held, rb_held, dt)`** — Integrates trigger depth (LT = lift down, RT = lift up) and bumper state (LB = grip close, RB = grip open) into absolute setpoints each loop, clamping to hardware limits defined in `Jetson/config.py`.

- **`_serial_reader(ser, label)`** — Daemon thread that continuously reads one line at a time from a serial port, dispatching `IMU:` lines into `_imu_data[label]` and `ENC:` lines into `_enc_data`.

- **`send_drive(lx, ly, yaw)` / `send_arm(lift_sp, grip_sp)`** — Direct serial write helpers (no deadband) used when external code needs to command the robot programmatically.

- **Main loop (50 Hz)** — Reads all joystick axes, applies slew rate limiting on drive outputs (see *Slew Rate* below), steps arm setpoints, and writes both serial commands each tick.

**Controller mapping:**

| Input | Action |
|-------|--------|
| Left stick X/Y | Strafe / forward-backward |
| Right stick X | Yaw (turn) |
| LT / RT | Lift down / up |
| LB / RB | Grip close / open |

---

### `simple_autonomy_controller.py` — Primary Loop

The production entry point. Runs at 50 Hz and combines the full GUI, sensor debug overlay, AprilTag navigation, person detection, mission planner, and controller input routing into one loop.

#### Controller Input & Mode Switching

All mode transitions are edge-triggered (rising edge only) to prevent repeated firing:

| Input | Action |
|-------|--------|
| A button | Start selected mission |
| B button | Cancel mission → return to manual |
| X button | Toggle fast arm speed mode |
| D-pad up/down | Navigate mission list |
| D-pad left/right | Set mission start step (for step-through debugging) |
| Number keys (0–9) + Enter | Type a specific start step index |

#### GUI & Debug Overlay

An OpenCV window renders the live camera feed at 640×360 with the following overlays:

- **AprilTag outlines** — colored polylines on detected tags with robot-frame `x/y` position labels
- **Mode banner** — `MANUAL` or `SEQ N/M: step_type` in the top-left corner
- **Drive command readout** — live `lx`, `ly`, `yaw` values being sent to the drive ESP
- **Arm state** — current `lift` and `grip` setpoints and whether the sequence or joystick owns the arm
- **IMU yaw** — current corrected heading with yaw offset annotation
- **Person threat level** — `CLEAR / SLOW / STOP` banner driven by the YOLO detector
- **Mission sidebar** — scrollable list of all missions with step count and active step indicator; clickable with mouse (single-click = select, double-click = run)

#### Primary Functions

- **`compute_drive_command(lx, ly, rx)`** — Applies deadband and returns the drive serial string. Called every manual-mode tick.

- **`step_arm_setpoints(...)`** — Same integrator as `robot_controller.py`; in autonomous mode the sequence runner bypasses this and slews directly to target positions.

- **`_serial_reader(ser, label)`** — Identical background thread parsing IMU and encoder telemetry into shared dicts.

- **`get_imu(label)`** / **`get_yaw_deg()`** — Thread-safe IMU snapshot. `get_yaw_deg()` applies the persisted yaw offset and wraps to `(-180, 180]` so 0° is always the calibrated forward direction.

- **`_camera_thread(...)`** — Daemon thread that captures frames, undistorts them, and runs the AprilTag detector on each frame, storing results in `_cam_frame` / `_cam_detections` for the main loop to consume.

- **Sequence runner** (inside main loop) — Steps through a mission list. Each iteration it reads `step["type"]` and dispatches to one of four handlers:
  - `drive_tag` / `drive_arm` — AprilTag-guided driving with ZVU dead reckoning (see below)
  - `set_arm` — Slews lift and grip to target positions
  - `turn_yaw` — IMU PD heading controller until within tolerance

#### Mission Planning

Missions are defined as Python dicts in the `MISSIONS` dictionary at the top of the file. Each mission is a named list of steps executed in order by the sequence runner. New missions can be added without touching any control logic — only the `MISSIONS` dict needs to be edited.

**Step types and their parameters:**

| Type | Required keys | Optional keys |
|------|--------------|---------------|
| `drive_tag` | `tag`, `stop_dist` | `lat_off`, `hold_s`, `yaw_deg`, `zvu_trust_dist`, `zvu_interval` |
| `drive_arm` | `tag`, `stop_dist`, `lift` or `grip` | same as `drive_tag` |
| `set_arm` | `lift` or `grip` (at least one) | — |
| `turn_yaw` | `yaw_deg` | `tol_deg`, `hold_s` |

**Parameter reference:**

- `tag` — AprilTag ID the robot navigates to. The detector searches every frame for this ID.
- `stop_dist` — Target distance from the tag in meters. Positive = stop in front; negative = allow the robot to pass the tag plane (e.g. driving into a rack). The step completes when the robot reaches this distance within tolerance.
- `lat_off` — Lateral offset from the tag center in normalized pixel units `[-1, 1]`. `0.0` centers on the tag; positive values offset right. Used to align to one side of a target.
- `lift` / `grip` — Absolute arm setpoints in radians. Omitting a key leaves that joint unchanged. The arm slews toward the target at `SEQ_ARM_LIFT_SPEED` / `SEQ_ARM_GRIP_SPEED` rad/s and the step blocks until within `SEQ_ARM_TOL_RAD` (0.05 rad) of both targets.
- `hold_s` — How long (seconds) the robot must hold within the position tolerance before the step is considered done. Prevents advancing on a momentary fluke reading.
- `yaw_deg` — Absolute IMU heading to hold during the step (degrees, 0 = calibrated forward). Overrides the default pixel-centering yaw controller. Used in `turn_yaw` as the target heading, and in drive steps to lock orientation while translating.
- `zvu_trust_dist` — Override the global `ZVU_TRUST_DIST_M` for this step. Set lower to force dead reckoning closer to the target; set to `0` to use live tag tracking the entire step.
- `zvu_interval` — Override `ZVU_INTERVAL_S` for this step. Shorter intervals give more frequent position fixes at the cost of more stop-and-go motion.

**`drive_tag`** — Drives the robot to a specific AprilTag. At long range it uses ZVU dead reckoning (stop → fix → dead-reckon → repeat); within `zvu_trust_dist` it switches to continuous live tag tracking. The drive axis controllers are:
```
ly  = clamp(K_FWD × (tag_distance − stop_dist))    # forward P-gain
lx  = clamp(K_LAT × lateral_pixel_error)            # strafe P-gain
yaw = clamp(K_YAW × lateral_pixel_error)            # heading P-gain (centering)
```
The step advances when forward error < 0.1 m and lateral error < 0.08 (normalized) hold simultaneously for `hold_s`.

**`drive_arm`** — Identical to `drive_tag` for driving, but simultaneously slews the arm to `lift` / `grip` targets. The step does not advance until both the drive tolerance and arm tolerance are satisfied. This allows the arm to pre-position during approach so no time is lost waiting for it after arrival.

**`set_arm`** — Holds the robot stationary and moves the arm only. Useful for gripping, releasing, or repositioning between drive steps.

**`turn_yaw`** — Spins in place to an absolute IMU heading. Uses a PD controller:
```
yaw_cmd = clamp(K_TURN_DEG × heading_error − K_D_TURN_DEG × yaw_rate)
```
Completes when heading error < `tol_deg` holds for `hold_s`.

**Example — DishRack Pickup mission:**
```python
"DishRack Pickup": [
    # 1. Approach to 1.73 m while raising arm and opening gripper
    {"type": "drive_arm", "tag": 6, "stop_dist": 1.73, "lat_off": 0.0,
     "lift": 3.5, "grip": 1.85, "zvu_trust_dist": 1.75},

    # 2. Close in to 1.34 m with live tag tracking
    {"type": "drive_tag", "tag": 6, "stop_dist": 1.34, "lat_off": 0.0},

    # 3. Lower arm to rack height
    {"type": "set_arm", "lift": 2.93, "grip": 1.85},

    # 4. Drive forward until under the rack (0.72 m)
    {"type": "drive_tag", "tag": 6, "stop_dist": 0.72, "lat_off": 0.0},

    # 5. Close gripper to grab rack
    {"type": "set_arm", "lift": 2.93, "grip": 0.38},

    # 6. Back away while raising arm to carry height
    {"type": "drive_arm", "tag": 6, "stop_dist": 1.5, "lat_off": 0.0, "lift": 3.6},

    # 7. Spin 180° to face the drop-off
    {"type": "turn_yaw", "yaw_deg": 180.0, "tol_deg": 2.0, "hold_s": 0.5},
]
```

Each step is independent — the sequence runner advances only after the current step's completion criteria are met, so adding, removing, or reordering steps does not require any changes to the control code.

#### Slew Rate Limiting

In manual mode, raw joystick values are not sent directly to the drive ESP. Instead, each axis is passed through `_slew(target, current, max_delta)` which limits the change per tick to `MAX_Drive_Slew × loop_period` (default: 4.0 normalized units/sec × 0.02 s = 0.08/tick). This prevents the Mecanum wheels from being commanded to large velocity steps instantaneously, reducing wheel slip and protecting the drivetrain.

---

## C++ Firmware (ESP32)

### Robot Drive — `esp_controller/src/robot/`

#### `robot_main.cpp` — Drive ESP Entry Point

Initializes the BNO08x IMU, serial link to Jetson, and motor/encoder hardware. The `loop()` runs at full speed and calls three timed tasks:

| Task | Rate | Function |
|------|------|----------|
| Parse Jetson commands | 50 Hz | `followTrajectory()` |
| Run wheel velocity PIDs | 200 Hz | `updatePIDs()` |
| Stream IMU + encoder telemetry | 50 Hz | Serial writes |

IMU data is streamed as `IMU:roll:X;...;yawRate:X;\n` using raw BNO08x rotation vector and gyro reports. Encoder velocities are streamed as `ENC:fl:X;bl:X;fr:X;br:X;\n`.

#### `robot_motion_control.cpp` — Mecanum Kinematics & Command Parsing

**`followTrajectory()`** — Reads one `lx:X;ly:X;yaw:X;\n` line from the Jetson serial buffer (or falls back to ESP-NOW joystick data if `JETSON_SERIAL` is not defined). Decodes the three normalized values and calls `updateSetpoints()`.

**`updateSetpoints(forward, strafe, turn)`** — Applies the standard Mecanum mixing matrix to convert a body-frame `(forward, strafe, turn)` command into four individual wheel velocity setpoints (in rad/s):

```
FL =  forward + strafe + turn
BL = -forward + strafe - turn   (note sign flip for rear wheels)
FR =  forward - strafe + turn
BR = -forward - strafe - turn
```

`MAX_FORWARD` and `MAX_TURN` constants scale the normalized `[-1, 1]` inputs to physical rad/s before the mix.

#### `robot_drive.cpp` — Wheel PID Velocity Control

Instantiates one `MotorDriver` and one `EncoderVelocity` object per wheel (FL, BL, FR, BR), each with its own PID controller.

**`updatePIDs()`** — Called at 200 Hz. For each wheel:
1. Reads encoder velocity (sign-corrected per wheel orientation via `pow(-1, i)`)
2. Computes PID error vs. current setpoint
3. Writes motor effort to the H-bridge driver

PID gains (`Kp`, `Ki`, `Kd`) and the derivative filter time constant (`pidTau`) are tuned per-wheel in this file.

---

### Arm — `esp_controller/src/arm/`

#### `arm_main.cpp` — Arm ESP Entry Point

Receives `lift:X;grip:X;\n` setpoints from the Jetson at up to 50 Hz and runs a 2 kHz position control loop. Streams back IMU data from the arm-mounted BNO08x at 10 Hz for diagnostic purposes.

#### `arm_drive.cpp` — Lead-Lag Position Control

Implements two independent position controllers (lift joint, grip joint).

**`updateSetpoints(lift_target, grip_target)`** — Accepts new absolute setpoint in radians. Clamps the input to hardware limits (`MIN/MAX_LIFT_RAD`, `MIN/MAX_GRIP_RAD`) before storing.

**`updateEffort()`** — Called at 2 kHz per joint:
1. Reads encoder position (with outlier rejection: ignores steps > 0.3 rad/tick)
2. Computes position error
3. Passes error through a lead-lag filter (tuned with `ALPHA`, `TD`, `TI` constants) to reduce steady-state error and improve step response
4. Scales by `Kp` to produce a motor effort and writes it to the H-bridge driver

---

## Key Algorithms

### Zero Velocity Update (ZVU) Dead Reckoning

Long-range AprilTag pose estimates are noisy. The ZVU system reduces this noise and handles temporary tag loss during `drive_tag` / `drive_arm` mission steps using three phases:

1. **`zvu_stop`** — Robot halts and holds heading via IMU PD for `ZVU_SETTLE_S` (0.35 s) to let vibrations die. After settle time, samples the tag pose with a stationary fix (lowest noise). Freezes the resulting `(lx, ly)` drive commands and captures the current IMU yaw as the heading reference. If the tag is absent after `ZVU_TIMEOUT_S`, resumes on the last known heading.

2. **`dead_reckon`** — Drives using the frozen `(lx, ly)` commands from the last fix for `ZVU_INTERVAL_S` seconds (default 2.5 s). Heading is actively held by a live IMU PD controller tracking the frozen heading reference — no tag, no encoders used. After the interval, transitions back to `zvu_stop` for a fresh fix. THIS IS UNDER DEVELOPMENT AND NOT CURRENTLY IMPLEMENTED. Planned to use for small scale adjustments when too close to april tag. 

3. **`tag_trust`** — Once the tag is within `ZVU_TRUST_DIST_M`, the robot switches to continuous live tag tracking with P-gain controllers on forward error (m) and lateral pixel error (normalized). This phase is permanent: the robot never returns to dead reckoning once it reaches close range.

Tuning knobs: `ZVU_INTERVAL_S`, `ZVU_SETTLE_S`, `ZVU_TRUST_DIST_M`, `ZVU_TIMEOUT_S`.

### AprilTag Navigation

The camera thread runs `pupil_apriltags` on every frame to detect tags and estimate their 6-DOF pose in camera frame. The main loop transforms each detection into robot frame using the fixed camera-to-robot extrinsic `T_CAM_ROBOT`mounted at the base of the butle bot:

```python
p_robot = R_cr @ p_cam + t_cr
```

`p_robot[1]` is the forward distance to the tag. Lateral error is computed from the tag's pixel centroid normalized to `[-1, 1]`. The `drive_tag` controller runs:

```
ly_cmd = clamp(K_FWD  × (dist - stop_dist))   # forward P-gain
lx_cmd = clamp(K_LAT  × lateral_pixel_error)   # lateral P-gain
yaw_cmd = clamp(K_YAW × lateral_pixel_error)   # yaw P-gain (centering)
```

A step advances when both forward error < `SEQ_REACH_FWD_M` (0.1 m) and lateral error < `SEQ_REACH_PIX_X` (0.08 normalized) hold continuously for `SEQ_DRIVE_HOLD_S` (0.4 s).

### Person Detection (YOLOv8)

A `PersonDetector` instance (wrapping `yolov8n.pt`) runs on every `PERSON_DETECT_EVERY_N` frame to limit CPU load. It returns one of three threat levels:

| Level | Action |
|-------|--------|
| `CLEAR` | No person in frame — full drive authority |
| `SLOW` | Person detected at distance — drive commands clamped to `PERSON_SLOW_SPEED` |
| `STOP` | Person too close — drive command overridden to `lx:0;ly:0;yaw:0` |

Person detection can be toggled at runtime with the `p` key in the OpenCV window. When disabled, threat level is forced to `CLEAR`.

### IMU Yaw Calibration

Raw BNO08x yaw is accumulated drift-sensitive. A yaw offset (`_yaw_offset_deg`) is subtracted from the raw reading so that 0° always means the robot's calibrated forward direction. Press `z` in the OpenCV window to zero the current heading and persist it to `yaw_offset.json`; press `l` to reload a previously saved offset. This is primarily used in debugging and for mission initialization. 
