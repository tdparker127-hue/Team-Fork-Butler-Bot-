# Butler Bot — Tuning & Configuration Checklist

Work through this document top-to-bottom before the first autonomous run.
Each section lists what to measure, what to set, and where the constant lives.

---

## 1. Physical Measurements

### 1.1 Robot Geometry
These must match the actual robot before odometry will be accurate.

| What to measure | Current value | Where to set |
|---|---|---|
| Wheel radius (centre of tread to axle) | `0.06 m` | `WHEEL_R` in `include/robot_drive.h` AND `WHEEL_R` in `Jetson/localization/ekf_localizer.py` |
| Half wheelbase front-to-back (axle-to-axle centre / 2) | `0.1675 m` | `L_X` in `ekf_localizer.py` |
| Half wheelbase left-to-right (axle-to-axle centre / 2) | `0.21 m` | `L_Y` in `ekf_localizer.py` |

> **How to measure:** Place robot on a flat surface.  Measure centre-to-centre between the two front wheels and divide by 2 → `L_Y`.  Measure centre-to-centre between front and rear axles and divide by 2 → `L_X`.

---

### 1.2 Camera Mounting
The extrinsic `T_CAM_ROBOT` in `Jetson/vision/apriltag_pose.py` describes where the camera sits relative to the robot base origin.

| What to measure | Current value | Where to set |
|---|---|---|
| Forward offset of camera from robot centre (+ = forward) | `0.00 m` | `T_CAM_ROBOT[1,3]` in `apriltag_pose.py` |
| Lateral offset of camera from robot centre (+ = right) | `0.00 m` | `T_CAM_ROBOT[0,3]` in `apriltag_pose.py` |
| Height of camera lens above robot base origin | `0.20 m` | `T_CAM_ROBOT[2,3]` in `apriltag_pose.py` |
| Camera yaw rotation relative to robot forward axis (deg) | `0°` | Rotation columns of `T_CAM_ROBOT` — call `wall_tag_rotation()` or build manually |

> **Note:** If the camera is not aligned with the robot's forward direction, the heading extracted from `robot_pose_from_camera()` will be wrong.  Add the appropriate yaw rotation to the first two columns of `T_CAM_ROBOT`.

---

### 1.3 AprilTag Layout
Tag 1 is the **world origin** (floor-projected position = `(0, 0)`).  All distances are in metres from tag 1.

**First, measure and fill in `TAG_HEIGHT_M`:**

| What to measure | Current value | Where to set |
|---|---|---|
| Height of all tag centres above the floor | `1.20 m` | `TAG_HEIGHT_M` in `Jetson/vision/apriltag_pose.py` |

**Then, for each tag, fill in `TAG_WORLD_POSES` in `apriltag_pose.py`:**

| Tag ID | dx from tag 1 (m) | dy from tag 1 (m) | Facing direction | Current placeholder |
|---|---|---|---|---|
| **1** | `0.0` | `0.0` | EAST (0°) | world origin |
| 2 | `0.0` | `1.5` | EAST (0°) | placeholder |
| 3 | `1.0` | `3.0` | SOUTH (270°) | placeholder |
| 4 | `3.0` | `3.0` | SOUTH (270°) | placeholder |
| 5 | `4.0` | `1.5` | WEST (180°) | placeholder |
| 6 | `2.0` | `0.0` | NORTH (90°) | placeholder |

`facing_angle_deg` convention:
- `0°`   → tag face points +X (EAST, tag is on the west wall)
- `90°`  → tag face points +Y (NORTH, tag is on the south wall)
- `180°` → tag face points -X (WEST, tag is on the east wall)
- `270°` → tag face points -Y (SOUTH, tag is on the north wall)

> **How to measure:** Stand at tag 1.  Measure along the wall (+Y into the arena, +X along the wall) to each other tag.

**AprilTag physical size:**

| What to measure | Current value | Where to set |
|---|---|---|
| Printed tag side length (outer black border edge to edge) | `0.10 m` | `TAG_SIZE_M` in `apriltag_pose.py` AND `TAG_SIZE_M` in `mission_controller.py` |

---

### 1.4 Arm Travel Limits
Run the arm to each physical end-stop, read the encoder position, and update the limits.

| What to measure | Current value | Where to set |
|---|---|---|
| Lift encoder position when fully lowered (picking height) | `0.0 rad` ✅ | `MIN_LIFT_RAD` in `include/arm_drive.h` AND `MIN_LIFT_RAD` in `robot_controller.py` |
| Lift encoder position when fully raised | `2.2 rad` ✅ | `MAX_LIFT_RAD` in `arm_drive.h` AND `MAX_LIFT_RAD` in `robot_controller.py` |
| Gripper encoder position when fully closed | `-2.0 rad` ⚠️ TODO | `MIN_GRIP_RAD` in `arm_drive.h` AND `robot_controller.py` |
| Gripper encoder position when fully open | `2.0 rad` ⚠️ TODO | `MAX_GRIP_RAD` in `arm_drive.h` AND `robot_controller.py` |

> Run `robot_controller.py` in teleop, hold LT/RT to open/close the gripper until it stalls.  Print the encoder value to Serial to read it.

---

## 2. Arm Pickup Sequence Setpoints
These are the positions the arm moves to during autonomous pickup/place.  Tune after the physical limits above are confirmed.

| Setpoint | Description | Current value | File |
|---|---|---|---|
| `ARM_LOWER_LIFT` | Lift height for reaching the tray | `-2.5 rad` | `mission_controller.py` |
| `ARM_GRIP_CLOSE` | Gripper position to clamp the tray | `-1.8 rad` | `mission_controller.py` |
| `ARM_CARRY_LIFT` | Lift height while carrying | `1.5 rad` | `mission_controller.py` |
| `ARM_GRIP_OPEN` | Gripper position to release | `1.5 rad` | `mission_controller.py` |
| `ARM_LOWER_TIME_S` | Wait after sending lower command | `2.0 s` | `mission_controller.py` |
| `ARM_GRIP_TIME_S` | Wait after sending grip command | `1.5 s` | `mission_controller.py` |
| `ARM_LIFT_TIME_S` | Wait after lifting | `2.0 s` | `mission_controller.py` |

---

## 3. Drive Controller (ESP32)
Located in `include/robot_drive.h`.

| Parameter | Current value | What it does |
|---|---|---|
| `Kp` | `0.2` | Wheel velocity PID — proportional gain |
| `Ki` | `0.0` | Wheel velocity PID — integral gain |
| `Kd` | `0.01` | Wheel velocity PID — derivative gain |
| `pidTau` | `0.5` | Derivative low-pass filter time constant |
| `MAX_FORWARD` | `8 rad/s` | Maximum wheel setpoint from Jetson `ly=1` |
| `MAX_TURN` | `5 rad/s` | Maximum wheel setpoint from Jetson `yaw=1` |

> **Tuning order:** Set `Ki=0, Kd=0`, raise `Kp` until the robot tracks a step command without oscillating.  Add `Kd` to damp overshoot.  Add `Ki` only if steady-state error matters.

---

## 4. Arm Controller (ESP32)
Lead-lag compensator, parameters in `include/arm_drive.h`.

| Parameter | Current value | What it does |
|---|---|---|
| `ARM_KP` | `0.936` | Proportional gain |
| `ARM_ALPHA` | `10.0` | Lead filter zero/pole ratio |
| `ARM_TD` | `0.0021` | Derivative time constant |
| `ARM_TI` | `0.0183` | Integral time constant |

---

## 5. Waypoint Navigation Gains
Located at the top of `Jetson/main/mission_controller.py`.

| Parameter | Current value | What it does |
|---|---|---|
| `KP_LIN` | `0.35` | Forward speed proportional to distance |
| `KP_STRAFE` | `0.25` | Strafe speed proportional to lateral error |
| `KP_ANG` | `0.70` | Yaw rate proportional to heading error |
| `WAYPOINT_REACHED_M` | `0.10 m` | Distance threshold to mark waypoint reached |
| `HEADING_REACHED_RAD` | `0.08 rad (~5°)` | Heading tolerance for final orientation |

> **Tuning order:** With a known good EKF pose, command a single waypoint 1 m ahead.  Raise `KP_LIN` until the robot moves quickly without overshooting.  If the robot weaves, reduce `KP_ANG` or raise `WAYPOINT_REACHED_M`.

---

## 6. EKF Localizer Noise Tuning
Located at the top of `Jetson/localization/ekf_localizer.py`.

### 6.1 Process noise (how much to trust the wheels)
| Parameter | Current value | What it does |
|---|---|---|
| `SIGMA_XY` | `0.05 m/s` | Translational uncertainty injected per second |
| `SIGMA_THETA` | `0.02 rad/s` | Rotational uncertainty injected per second |

Higher → filter trusts vision more; lower → filter trusts odometry more.

### 6.2 IMU measurement noise
| Parameter | Current value | What it does |
|---|---|---|
| `SIGMA_IMU_YAW` | `0.035 rad (~2°)` | BNO08x heading noise.  Raise if you see jumpy corrections. |

### 6.3 AprilTag measurement noise
| Parameter | Current value | What it does |
|---|---|---|
| `SIGMA_TAG_XY` | `0.03 m` | Per-axis position noise for a single-tag fix |
| `SIGMA_TAG_YAW` | `0.04 rad` | Heading noise for a single-tag fix |

Both scale by `1/√n_tags` when multiple tags are visible.

### 6.4 Gating thresholds
| Parameter | Current value | What it does |
|---|---|---|
| `MAHAL_THRESH_IMU` | `3.841` (χ²(1, 95%)) | Rejects IMU updates that are statistical outliers |
| `MAHAL_THRESH_TAG` | `7.815` (χ²(3, 95%)) | Rejects AprilTag updates that are outliers |
| `EUCLID_THRESH_IMU_RAD` | `0.50 rad` | Euclidean gate on raw yaw innovation |
| `EUCLID_THRESH_TAG_POS_M` | `0.50 m` | Euclidean gate on position innovation norm |
| `EUCLID_THRESH_TAG_YAW_RAD` | `0.40 rad` | Euclidean gate on heading innovation |

Switch between gating modes by passing `gating=GatingMethod.EUCLIDEAN` to `EKFLocalizer()` in `mission_controller.py`.

### 6.5 Motor sign convention
If odometry drifts in a consistent direction while driving straight:

| Parameter | Current value | What it does |
|---|---|---|
| `MOTOR_SIGNS` | `[1, 1, 1, 1]` | Per-motor sign; flip individual elements to correct FK |

Motor order: `[FrLft, BkLft, FrRgt, BkRgt]`.

---

## 7. Person Detection Thresholds
Located at the top of `Jetson/vision/person_detection.py`.

| Parameter | Current value | What it does |
|---|---|---|
| `PERSON_CONF` | `0.50` | YOLO minimum confidence.  Lower → more sensitive but more false positives |
| `SLOW_DIST_M` | `2.0 m` | Enter SLOW state if person is within this distance |
| `STOP_DIST_M` | `0.8 m` | Enter STOP state if person is within this distance |
| `SLOW_BBOX_FRAC` | `0.10` | Fallback: SLOW if person bbox > 10% of frame area |
| `STOP_BBOX_FRAC` | `0.25` | Fallback: STOP if person bbox > 25% of frame area |
| `DEPTH_PATCH_HALF` | `3` | Half-size of depth sample patch (pixels) |
| `DEPTH_MAX_MM` | `6000 mm` | Depth readings above this are treated as invalid |
| `YOLO_DEVICE` | `"cpu"` | Change to `"cuda:0"` if Jetson GPU is available |

---

## 8. Tray Detection Thresholds
Located at the top of `Jetson/vision/tray_detection.py`.

| Parameter | Current value | What it does |
|---|---|---|
| `BRIGHT_TRAY_LOWER_HSV` | `[0, 150, 80]` | HSV lower bound for generic bright-tray mask |
| `BRIGHT_TRAY_UPPER_HSV` | `[180, 255, 255]` | HSV upper bound for generic bright-tray mask |
| `TRAY_MIN_AREA` | `1500 px²` | Minimum blob area to count as a tray |
| `TARGET_PICKUP_DIST_MM` | `400 mm` | Target camera-to-tray distance for pickup |
| `CENTERING_DEAD_ZONE_PX` | `20 px` | Pixel tolerance for tray centering |
| `KP_APPROACH_LATERAL` | `0.0015` | Strafe gain during approach |
| `KP_APPROACH_YAW` | `0.0008` | Yaw gain during approach |
| `KP_APPROACH_FORWARD` | `0.0008` | Forward gain during approach |
| `MAX_APPROACH_LX/LY/YAW` | `0.40 / 0.40 / 0.30` | Clamp on approach command magnitude |

> **Tuning:** Run `test_localization.py` (or a tray-specific test) and adjust `BRIGHT_TRAY_LOWER_HSV` saturation threshold until only the physical tray blobs are detected.  Then adjust `KP_APPROACH_*` gains to make the approach smooth.

---

## 9. Teleop / Joystick
Located in `Jetson/main/robot_controller.py`.

| Parameter | Current value | What it does |
|---|---|---|
| `DEADBAND` | `0.10` | Joystick dead-zone (fraction of full stick) |
| `MAX_LIFT_SPEED` | `0.5 rad/s` | Lift rate when bumper held |
| `MAX_GRIP_SPEED` | `1.5 rad/s` | Max gripper rate at full trigger |
| `BTN_LB` / `BTN_RB` | `6` / `7` | Bumper button indices.  Run joystick debug script to verify. |

---

## 10. Mission Waypoints
Edit `EXAMPLE_MISSION` in `Jetson/main/mission_controller.py`.  All positions are in metres relative to the **tag 1 floor-projected origin**.

```python
EXAMPLE_MISSION = [
    {"x": 1.0, "y": 1.0, "label": "approach_table"},
    {"x": 2.0, "y": 1.0, "label": "tray_pickup_zone"},
    {"x": 2.0, "y": 2.5, "theta": math.pi / 2, "label": "drop_zone"},
]
```

`theta` (optional) sets the desired final heading at that waypoint in radians (0 = facing tag-1 wall, π/2 = facing north wall, etc.).

---

## 11. Quick-Reference: Files and Their Parameters

| File | What lives there |
|---|---|
| `include/robot_drive.h` | Motor PID gains, max wheel speed |
| `include/arm_drive.h` | Arm lead-lag gains, soft travel limits |
| `Jetson/vision/apriltag_pose.py` | `TAG_HEIGHT_M`, `TAG_WORLD_POSES`, `T_CAM_ROBOT`, `TAG_SIZE_M` |
| `Jetson/localization/ekf_localizer.py` | `WHEEL_R`, `L_X`, `L_Y`, all σ noise values, gating thresholds |
| `Jetson/vision/person_detection.py` | YOLO confidence, distance/bbox thresholds |
| `Jetson/vision/tray_detection.py` | HSV tray color, pickup distance, approach gains |
| `Jetson/main/mission_controller.py` | Nav gains, arm sequence setpoints, waypoints, camera device |
| `Jetson/main/robot_controller.py` | Serial ports, deadband, arm speed limits, joystick indices |

---

## 12. Validation Steps (in order)

1. **Arm limits** — teleop only, verify lift/grip travel without hitting hard stops
2. **Drive PID** — teleop, verify robot tracks velocity commands smoothly
3. **Camera calibration** — run `apriltag_camera_calibration.py`, save `.npz`
4. **AprilTag geometry** — run `python3 Jetson/test_localization.py`, physically move the robot and verify X/Y/yaw match expectations
5. **EKF fusion** — drive in a straight line and a square, check that pose stays reasonable
6. **Person detection** — walk in front of the robot, verify SLOW/STOP triggers at the right distances
7. **Tray detection** — place a tray, verify it blobs correctly and `is_centered_and_close()` returns True when aligned
8. **Waypoint nav** — command a single 1 m forward waypoint, tune nav gains
9. **Full mission** — run `mission_controller.py`, press Start
