"""
simple_autonomy_controller.py
Jetson Nano -- manual drive + AprilTag-following autonomy

Controls:
  A button (0) -> enter autonomous mode (drive toward TARGET_TAG_ID)
  B button (1) -> instantly return to manual control  (default on start)

Manual drive mapping:
  Left stick  -> forward / strafe
  Right stick X -> yaw
  LT / RT     -> arm lift down / up
  LB / RB     -> gripper close / open

Autonomous mode:
  Camera detects TARGET_TAG_ID and a P-controller drives the robot toward
  it, stopping STOP_DIST_M in front of the tag face.  If the tag leaves
  view the robot stops and waits until it reappears.

OpenCV window:
  - Live camera feed with tag outlines and robot-frame x/y labels
  - Mode banner (MANUAL / AUTO)
  - Tag distance info when in auto mode
  - "A=auto  B=manual" hint at bottom
"""

import enum
import math
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import pygame
import serial

try:
    import pupil_apriltags as apriltag
    _APRILTAG_OK = True
except ImportError:
    print("[WARN] pupil_apriltags not installed -- autonomous mode unavailable.")
    _APRILTAG_OK = False

from Jetson.config import (
    MIN_LIFT_RAD, MAX_LIFT_RAD, MIN_GRIP_RAD, MAX_GRIP_RAD,
    MAX_LIFT_SPEED, MAX_GRIP_SPEED, MAX_LIFT_SPEED_FAST, MAX_GRIP_SPEED_FAST,
    TAG_FAMILY, TAG_SIZE_M, T_CAM_ROBOT, COLOR_CAMERA_DEVICE,
)

# -- Serial ports --------------------------------------------------------------
DRIVE_PORT = "/dev/ttyACM0"
ARM_PORT   = "/dev/ttyACM1"
BAUD_RATE  = 115200

# -- Manual drive constants ----------------------------------------------------
DEADBAND       = 0.1
MAX_Drive_Slew = 4.0   # max normalized change per second (smoothing)

# -- Arm speeds ----------------------------------------------------------------
# Imported from Jetson/config.py — edit there.
# MAX_LIFT_SPEED, MAX_GRIP_SPEED, MAX_LIFT_SPEED_FAST, MAX_GRIP_SPEED_FAST

# -- Xbox One BT axis / button indices -----------------------------------------
AXIS_LX = 0  # Left stick X  -> strafe
AXIS_LY = 1  # Left stick Y  -> forward
AXIS_RX = 2  # Right stick X -> yaw
AXIS_RY = 3  # unused
AXIS_LT = 4  # Left trigger  -> lift down
AXIS_RT = 5  # Right trigger -> lift up
BTN_LB     = 6   # grip close
BTN_RB     = 7   # grip open
BTN_AUTO        = 0   # A -- enter single-tag autonomous mode
BTN_MANUAL      = 1   # B -- return to manual (instant override, cancels sequence)
BTN_SEQ         = 2   # X -- start full autonomous sequence
BTN_SPEED_BOOST = 3   # Y -- toggle high-speed arm mode

# -- Camera / AprilTag ---------------------------------------------------------
CAMERA_DEVICE = COLOR_CAMERA_DEVICE
CALIB_FILE    = Path(__file__).parent.parent / "vision" / "camera_calibration_live.npz"
FRAME_W, FRAME_H, FRAME_FPS = 640, 360, 15

# -- Autonomous control --------------------------------------------------------
TARGET_TAG_ID  = 6      # AprilTag ID to drive toward
STOP_DIST_M    = 0.5    # desired forward distance from tag face [m]
K_FWD          = 0.6    # forward P-gain  (normalized speed / m error)
K_LAT          = 1.5    # lateral P-gain  (normalized speed / normalized pixel error [-1,1])
K_YAW          = 0.8    # yaw P-gain from normalized pixel error
# Goal offset relative to tag center.
# GOAL_OFFSET_X: normalized pixel units [-1, 1]. 0 = aim at pixel center.
#   positive = aim to the right of tag center, negative = aim to the left.
# GOAL_OFFSET_Y: meters. Adds to STOP_DIST_M (positive = stop further away).
GOAL_OFFSET_X  = 0.0    # lateral pixel offset (right = +, left = -)
GOAL_OFFSET_Y  = 0.0    # forward offset [m]  (further = +, closer = -)


# ─── Sequence runner constants ─────────────────────────────────────────────────
SEQ_ARM_LIFT_SPEED = MAX_LIFT_SPEED   # max lift slew rate during sequence [rad/s]
SEQ_ARM_GRIP_SPEED = MAX_GRIP_SPEED   # max grip slew rate during sequence [rad/s]
SEQ_ARM_TOL_RAD    = 0.05             # arm arrival tolerance [rad]
SEQ_DRIVE_HOLD_S   = 0.4             # hold within drive tolerance this long before advancing [s]
SEQ_REACH_FWD_M    = 0.08            # forward error threshold [m]
SEQ_REACH_PIX_X    = 0.06            # lateral pixel tolerance (normalized [-1,1])
SEQ_YAW_TOL_DEG    = 3.0             # heading tolerance for turn_yaw steps [deg]
SEQ_YAW_HOLD_S     = 0.3             # hold within yaw tolerance before advancing [s]
K_TURN_DEG         = 0.025           # P-gain for turn_yaw:  yaw_cmd = clamp(K_TURN_DEG * err_deg)
                                      #   40 deg error → 1.0 (full speed)


# ─── AUTONOMOUS SEQUENCE ───────────────────────────────────────────────────────
# Edit this list to define the X-button autonomous sequence.
# Steps execute in order; B button cancels immediately.
#
# Step types:
#
#   "drive_tag"  — drive toward an AprilTag until centered and at stop_dist
#       tag        : int    AprilTag ID
#       stop_dist  : float  desired forward stop distance [m]
#       lat_off    : float  lateral pixel goal offset [-1..1], 0 = tag center
#       hold_s     : float  (optional) seconds to hold within tolerance
#
#   "set_arm"    — rate-limited slew of lift and/or grip; waits for arrival
#       lift       : float  target lift position [rad]  (omit to leave unchanged)
#       grip       : float  target grip position [rad]  (omit to leave unchanged)
#
#   "drive_arm"  — drive_tag AND set_arm simultaneously;
#                  advances only when BOTH are done
#       (all drive_tag params + all set_arm params)
#
#   "turn_yaw"   — rotate in place to a given IMU heading
#       yaw_deg    : float  target heading [degrees, IMU frame]
#       tol_deg    : float  (optional) tolerance [deg]
#       hold_s     : float  (optional) hold time [s]
#
SEQUENCE = [
    # 1. Raise arm to carry height, open gripper
    {"type": "set_arm",   "lift": 3.0,  "grip": 1.85},
    # 2. Drive toward tag 6, stop 0.5 m away
    # {"type": "drive_tag", "tag": 6,     "stop_dist": 0.5, "lat_off": 0.0},
    # 3. Simultaneously close in and lower lift
    # {"type": "drive_arm", "tag": 6,     "stop_dist": 0.3, "lat_off": 0.0,
    #                       "lift": 1.5,  "grip": 1.85},
    # # 4. Release gripper
    # {"type": "set_arm",   "grip": 0.0},
]


# -- Helper functions ----------------------------------------------------------

def _clamp_range(value: float, lower: float, upper: float) -> float:
    """Clamp a value between two bounds, regardless of numeric ordering."""
    if lower <= upper:
        return max(lower, min(upper, value))
    return min(lower, max(upper, value))


def _trigger_depth(raw: float) -> float:
    """Convert trigger axis (rest=-1, full=+1) to depth in [0, 1]."""
    return max(0.0, (raw + 1.0) / 2.0)


def step_arm_setpoints(
    lift_sp: float, grip_sp: float,
    lt_raw: float, rt_raw: float,
    lb_held: bool, rb_held: bool,
    dt: float,
    fast: bool = False,
) -> tuple:
    lift_speed = MAX_LIFT_SPEED_FAST if fast else MAX_LIFT_SPEED
    grip_speed = MAX_GRIP_SPEED_FAST if fast else MAX_GRIP_SPEED
    lift_sp = _clamp_range(
        lift_sp + (_trigger_depth(lt_raw) - _trigger_depth(rt_raw)) * lift_speed * dt,
        MIN_LIFT_RAD, MAX_LIFT_RAD,
    )
    grip_sp = _clamp_range(
        grip_sp + (int(rb_held) - int(lb_held)) * grip_speed * dt,
        MIN_GRIP_RAD, MAX_GRIP_RAD,
    )
    return lift_sp, grip_sp


def compute_drive_command(lx: float, ly: float, rx: float) -> str:
    lx_out  = _scale(lx)
    ly_out  = -_scale(ly)
    yaw_out = _scale(rx)
    return f"lx:{lx_out:.3f};ly:{ly_out:.3f};yaw:{yaw_out:.3f};\n"


def _scale(value: float, deadband: float = DEADBAND) -> float:
    return 0.0 if abs(value) < deadband else value


def _slew(target: float, current: float, max_delta: float) -> float:
    return current + max(-max_delta, min(max_delta, target - current))


# -- Serial / IMU / encoder state ----------------------------------------------

_imu_lock = threading.Lock()
_imu_data = {
    "drive": {"roll": 0., "pitch": 0., "yaw": 0.,
               "rollRate": 0., "pitchRate": 0., "yawRate": 0.},
    "arm":   {"roll": 0., "pitch": 0., "yaw": 0.,
               "rollRate": 0., "pitchRate": 0., "yawRate": 0.},
}
_enc_lock = threading.Lock()
_enc_data = {"fl": 0., "bl": 0., "fr": 0., "br": 0., "timestamp": 0.}
_drive_ser = None
_arm_ser   = None


def _parse_kv_line(line: str, prefix: str) -> dict | None:
    if not line.startswith(prefix):
        return None
    result = {}
    try:
        for token in line[len(prefix):].split(";"):
            if not token:
                continue
            k, _, v = token.partition(":")
            result[k] = float(v)
    except (ValueError, AttributeError):
        return None
    return result or None


def _serial_reader(ser: serial.Serial, label: str) -> None:
    while True:
        try:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            imu = _parse_kv_line(line, "IMU:")
            if imu:
                with _imu_lock:
                    _imu_data[label].update(imu)
                continue
            enc = _parse_kv_line(line, "ENC:") if label == "drive" else None
            if enc and len(enc) == 4:
                with _enc_lock:
                    _enc_data.update(enc)
                    _enc_data["timestamp"] = time.monotonic()
                continue
            if line.startswith("DBG:") or line in ("Failed", "Sent"):
                pass
            else:
                print(f"[{label}] {line}")
        except serial.SerialException:
            print(f"[{label}] Serial read error -- reconnect and restart.")
            break
        except Exception as e:
            print(f"[{label}] Reader error: {e}")


def get_imu(label: str) -> dict:
    with _imu_lock:
        return dict(_imu_data[label])


def get_enc() -> dict:
    with _enc_lock:
        return dict(_enc_data)


# -- Drive / arm serial handles exposed for external callers ------------------
def send_drive(lx: float, ly: float, yaw: float) -> None:
    if _drive_ser and _drive_ser.is_open:
        _drive_ser.write(f"lx:{lx:.3f};ly:{ly:.3f};yaw:{yaw:.3f};\n".encode())


def send_arm(lift_sp: float, grip_sp: float) -> None:
    if _arm_ser and _arm_ser.is_open:
        _arm_ser.write(f"lift:{lift_sp:.3f};grip:{grip_sp:.3f};\n".encode())


# -- Camera thread -------------------------------------------------------------

_cam_lock       = threading.Lock()
_cam_frame      = None   # latest undistorted BGR frame (numpy array)
_cam_detections = []     # latest pupil_apriltags detection list


def _camera_thread(cap, detector, K_cam, dist_cam, camera_params) -> None:
    global _cam_frame, _cam_detections
    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.01)
            continue
        frame = cv2.undistort(frame, K_cam, dist_cam)
        frame = cv2.flip(frame, -1)
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        dets  = detector.detect(
            gray, estimate_tag_pose=True,
            camera_params=list(camera_params), tag_size=TAG_SIZE_M,
        )
        with _cam_lock:
            _cam_frame      = frame
            _cam_detections = dets


# -- Main ----------------------------------------------------------------------

def main() -> None:
    global _drive_ser, _arm_ser

    # Joystick
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No joystick detected. Connect Xbox controller and retry.")
        return
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Controller: {joystick.get_name()}")
    print("  A = autonomous mode    B = manual mode  (starting in MANUAL)")

    # Serial ports
    try:
        drive_ser = serial.Serial(DRIVE_PORT, BAUD_RATE, timeout=1)
    except serial.SerialException as e:
        print(f"Could not open drive port {DRIVE_PORT}: {e}")
        return
    try:
        arm_ser = serial.Serial(ARM_PORT, BAUD_RATE, timeout=1)
    except serial.SerialException as e:
        print(f"Could not open arm port {ARM_PORT}: {e}")
        drive_ser.close()
        return

    time.sleep(2)
    drive_ser.reset_input_buffer()
    arm_ser.reset_input_buffer()
    _drive_ser = drive_ser
    _arm_ser   = arm_ser
    threading.Thread(target=_serial_reader, args=(drive_ser, "drive"), daemon=True).start()
    threading.Thread(target=_serial_reader, args=(arm_ser,   "arm"),   daemon=True).start()

    # Camera + AprilTag detector
    R_cr = T_CAM_ROBOT[:3, :3]
    t_cr = T_CAM_ROBOT[:3, 3]
    cap  = None

    if _APRILTAG_OK and CALIB_FILE.exists():
        cal  = np.load(CALIB_FILE)
        K_cam, dist_cam = cal["camera_matrix"], cal["dist_coeffs"]
        camera_params   = (K_cam[0, 0], K_cam[1, 1], K_cam[0, 2], K_cam[1, 2])
        cap = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
        cap.set(cv2.CAP_PROP_FPS, FRAME_FPS)
        detector = apriltag.Detector(
            families=TAG_FAMILY, nthreads=2, quad_decimate=1.0,
            quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25,
        )
        threading.Thread(
            target=_camera_thread,
            args=(cap, detector, K_cam, dist_cam, camera_params),
            daemon=True,
        ).start()
        print(f"Camera opened: {CAMERA_DEVICE}  ->  target tag: {TARGET_TAG_ID}")
    else:
        if not _APRILTAG_OK:
            print("[WARN] pupil_apriltags missing -- camera/auto mode unavailable.")
        else:
            print(f"[WARN] Calibration file not found: {CALIB_FILE}")

    # Control state
    loop_period = 1.0 / 50.0
    lift_sp = grip_sp = 0.0
    lx_cmd = ly_cmd = yaw_cmd = 0.0
    auto_mode      = False   # start in manual
    fast_mode      = False   # start in normal speed
    seq_idx        = -1      # -1 = not running; >= 0 = current SEQUENCE index
    seq_hold_start = 0.0     # monotonic time when we entered the tolerance zone
    _a_prev = _b_prev = _x_prev = _y_prev = False

    print("Running at 50 Hz.  Ctrl-C or q/Esc in window to stop.")

    try:
        while True:
            t0 = time.monotonic()
            pygame.event.pump()

            # Read controller
            lx = joystick.get_axis(AXIS_LX)
            ly = joystick.get_axis(AXIS_LY)
            rx = joystick.get_axis(AXIS_RX)
            lt = joystick.get_axis(AXIS_LT) if joystick.get_numaxes() > AXIS_LT else -1.0
            rt = joystick.get_axis(AXIS_RT) if joystick.get_numaxes() > AXIS_RT else -1.0
            lb = bool(joystick.get_button(BTN_LB)) if joystick.get_numbuttons() > BTN_LB else False
            rb = bool(joystick.get_button(BTN_RB)) if joystick.get_numbuttons() > BTN_RB else False
            btn_a = bool(joystick.get_button(BTN_AUTO))
            btn_b = bool(joystick.get_button(BTN_MANUAL))
            btn_x = bool(joystick.get_button(BTN_SEQ))   if joystick.get_numbuttons() > BTN_SEQ         else False
            btn_y = bool(joystick.get_button(BTN_SPEED_BOOST)) if joystick.get_numbuttons() > BTN_SPEED_BOOST else False

            # Mode switching (edge-triggered)
            if btn_b and not _b_prev:               # B always cancels everything
                auto_mode = False
                seq_idx   = -1
                print("\n[MODE] MANUAL")
            elif btn_x and not _x_prev:             # X starts sequence
                seq_idx        = 0
                auto_mode      = False
                seq_hold_start = 0.0
                print(f"\n[SEQ] Start -- Step 1/{len(SEQUENCE)}: {SEQUENCE[0]['type']}")
            elif btn_a and not _a_prev and seq_idx < 0:   # A only when not in sequence
                auto_mode = True
                print(f"\n[MODE] AUTONOMOUS -- driving toward tag {TARGET_TAG_ID}")
            if btn_y and not _y_prev:
                fast_mode = not fast_mode
                label = "FAST" if fast_mode else "NORMAL"
                print(f"\n[SPEED] Arm speed: {label}")
            _a_prev, _b_prev, _x_prev, _y_prev = btn_a, btn_b, btn_x, btn_y

            # Get latest camera frame + detections
            with _cam_lock:
                frame      = _cam_frame.copy() if _cam_frame is not None else None
                detections = list(_cam_detections)

            # ── Locate TARGET_TAG_ID for A-button auto mode ───────────────────
            tag_pos_rob  = None
            tag_pixel_cx = None
            for det in detections:
                if det.tag_id == TARGET_TAG_ID and det.pose_t is not None:
                    p_cam        = np.array(det.pose_t, dtype=float).ravel()
                    tag_pos_rob  = R_cr @ p_cam + t_cr
                    raw_cx       = float(det.corners[:, 0].mean())
                    tag_pixel_cx = (raw_cx - FRAME_W / 2.0) / (FRAME_W / 2.0)
                    break

            # A-button autonomous effort (always computed for overlay)
            if tag_pos_rob is not None and tag_pixel_cx is not None:
                _el  = tag_pixel_cx - GOAL_OFFSET_X
                auto_lx  = max(-1.0, min(1.0, K_LAT * _el))
                auto_ly  = max(-1.0, min(1.0, K_FWD * (tag_pos_rob[1] - (STOP_DIST_M + GOAL_OFFSET_Y))))
                auto_yaw = max(-1.0, min(1.0, K_YAW * _el))
            else:
                auto_lx = auto_ly = auto_yaw = 0.0

            # ── Sequence runner ───────────────────────────────────────────────
            seq_drive_lx = seq_drive_ly = seq_drive_yaw = 0.0
            seq_arm_owns = False   # True = sequence owns arm (block joystick)
            if seq_idx >= 0:
                step  = SEQUENCE[seq_idx]
                stype = step["type"]
                now   = time.monotonic()

                # Arm slewing (set_arm and drive_arm) -- fixed rate for safety
                if stype in ("set_arm", "drive_arm"):
                    seq_arm_owns = True
                    lift_tgt = step.get("lift", lift_sp)
                    grip_tgt = step.get("grip", grip_sp)
                    lift_sp  = _slew(lift_tgt, lift_sp, SEQ_ARM_LIFT_SPEED * loop_period)
                    grip_sp  = _slew(grip_tgt, grip_sp, SEQ_ARM_GRIP_SPEED * loop_period)
                    arm_done = (abs(lift_sp - lift_tgt) < SEQ_ARM_TOL_RAD and
                                abs(grip_sp - grip_tgt) < SEQ_ARM_TOL_RAD)
                else:
                    arm_done = True

                # Drive control (drive_tag and drive_arm)
                if stype in ("drive_tag", "drive_arm"):
                    _t_pos = None
                    _t_pix = None
                    for det in detections:
                        if det.tag_id == step["tag"] and det.pose_t is not None:
                            pc     = np.array(det.pose_t, dtype=float).ravel()
                            _t_pos = R_cr @ pc + t_cr
                            _t_pix = (float(det.corners[:, 0].mean()) - FRAME_W / 2) / (FRAME_W / 2)
                            break
                    if _t_pos is not None:
                        _ef = _t_pos[1] - step.get("stop_dist", 0.5)
                        _el = _t_pix  - step.get("lat_off", 0.0)
                        seq_drive_lx  = max(-1., min(1., K_LAT * _el))
                        seq_drive_ly  = max(-1., min(1., K_FWD * _ef))
                        seq_drive_yaw = max(-1., min(1., K_YAW * _el))
                        _in_tol = abs(_ef) < SEQ_REACH_FWD_M and abs(_el) < SEQ_REACH_PIX_X
                    else:
                        _in_tol = False
                    if _in_tol:
                        if seq_hold_start == 0.0:
                            seq_hold_start = now
                        drive_done = (now - seq_hold_start) >= step.get("hold_s", SEQ_DRIVE_HOLD_S)
                    else:
                        seq_hold_start = 0.0
                        drive_done = False

                elif stype == "turn_yaw":
                    _imu_yaw = get_imu("drive").get("yaw", 0.0)
                    _err_deg = ((step["yaw_deg"] - _imu_yaw + 180) % 360) - 180
                    seq_drive_yaw = max(-1., min(1., K_TURN_DEG * _err_deg))
                    _in_tol = abs(_err_deg) < step.get("tol_deg", SEQ_YAW_TOL_DEG)
                    if _in_tol:
                        if seq_hold_start == 0.0:
                            seq_hold_start = now
                        drive_done = (now - seq_hold_start) >= step.get("hold_s", SEQ_YAW_HOLD_S)
                    else:
                        seq_hold_start = 0.0
                        drive_done = False

                else:   # "set_arm" only -- no drive
                    drive_done = True

                if drive_done and arm_done:
                    seq_idx += 1
                    seq_hold_start = 0.0
                    if seq_idx >= len(SEQUENCE):
                        seq_idx = -1
                        print("\n[SEQ] Sequence complete -- returning to MANUAL")
                    else:
                        print(f"\n[SEQ] Step {seq_idx + 1}/{len(SEQUENCE)}: {SEQUENCE[seq_idx]['type']}")

            # ── Drive command ─────────────────────────────────────────────────
            if seq_idx >= 0:
                _stype = SEQUENCE[seq_idx]["type"]
                if _stype in ("drive_tag", "drive_arm", "turn_yaw"):
                    drive_cmd = f"lx:{seq_drive_lx:.3f};ly:{seq_drive_ly:.3f};yaw:{seq_drive_yaw:.3f};\n"
                else:
                    drive_cmd = "lx:0.000;ly:0.000;yaw:0.000;\n"
            elif auto_mode and tag_pos_rob is not None:
                drive_cmd = f"lx:{auto_lx:.3f};ly:{auto_ly:.3f};yaw:{auto_yaw:.3f};\n"
            elif auto_mode:
                drive_cmd = "lx:0.000;ly:0.000;yaw:0.000;\n"
            else:
                max_step = MAX_Drive_Slew * loop_period
                lx_cmd  = _slew(lx,  lx_cmd,  max_step)
                ly_cmd  = _slew(ly,  ly_cmd,  max_step)
                yaw_cmd = _slew(rx, yaw_cmd, max_step)
                drive_cmd = compute_drive_command(lx_cmd, ly_cmd, yaw_cmd)

            # ── Arm command ───────────────────────────────────────────────────
            # Sequence owns arm during set_arm / drive_arm; joystick otherwise
            if not seq_arm_owns:
                lift_sp, grip_sp = step_arm_setpoints(
                    lift_sp, grip_sp, lt, rt, lb, rb, loop_period, fast=fast_mode,
                )

            drive_ser.write(drive_cmd.encode())
            arm_ser.write(f"lift:{lift_sp:.3f};grip:{grip_sp:.3f};\n".encode())

            # OpenCV window
            if frame is not None:
                font = cv2.FONT_HERSHEY_SIMPLEX

                # Tag outlines + robot-frame position labels
                for det in detections:
                    pts = det.corners.astype(int)
                    col = (0, 255, 60) if det.tag_id == TARGET_TAG_ID else (0, 200, 200)
                    cv2.polylines(frame, [pts.reshape(-1, 1, 2)], True, col, 2)
                    cx_t = int(pts[:, 0].mean())
                    cy_t = int(pts[:, 1].mean())
                    cv2.putText(frame, f"id={det.tag_id}", (cx_t - 20, cy_t - 10),
                                font, 0.6, col, 2, cv2.LINE_AA)
                    if det.pose_t is not None:
                        p_cam = np.array(det.pose_t, dtype=float).ravel()
                        p     = R_cr @ p_cam + t_cr
                        # Robot-frame (after transform)
                        cv2.putText(frame, f"rob x={p[0]:+.2f} y={p[1]:+.2f} m",
                                    (cx_t - 70, cy_t + 18),
                                    font, 0.45, (0, 0, 255), 1, cv2.LINE_AA)
                        # Raw camera-frame (before transform): cam +X=right, +Y=down, +Z=forward
                        cv2.putText(frame, f"cam x={p_cam[0]:+.2f} z={p_cam[2]:+.2f} m",
                                    (cx_t - 70, cy_t + 34),
                                    font, 0.45, (255, 180, 0), 1, cv2.LINE_AA)

                # Mode banner (top-left)
                if seq_idx >= 0:
                    _sb = SEQUENCE[seq_idx]
                    mode_str = f"SEQ {seq_idx + 1}/{len(SEQUENCE)}: {_sb['type']}  (B=cancel)"
                    mode_col = (0, 180, 255)
                elif auto_mode:
                    mode_str = f"AUTO  (tag {TARGET_TAG_ID})"
                    mode_col = (0, 220, 60)
                else:
                    mode_str = "MANUAL"
                    mode_col = (40, 80, 255)
                cv2.putText(frame, mode_str, (8, 30), font, 0.9, (0, 0, 0),    4, cv2.LINE_AA)
                cv2.putText(frame, mode_str, (8, 30), font, 0.9, mode_col, 2, cv2.LINE_AA)

                # Speed mode indicator (top-right)
                spd_str = "FAST ARM" if fast_mode else "normal arm"
                spd_col = (0, 80, 255) if fast_mode else (160, 160, 160)
                (spd_w, _), _ = cv2.getTextSize(spd_str, font, 0.7, 2)
                cv2.putText(frame, spd_str, (FRAME_W - spd_w - 8, 30), font, 0.7, (0, 0, 0),  4, cv2.LINE_AA)
                cv2.putText(frame, spd_str, (FRAME_W - spd_w - 8, 30), font, 0.7, spd_col,    2, cv2.LINE_AA)

                # Auto status line
                if auto_mode and tag_pos_rob is not None:
                    cv2.putText(frame,
                                f"fwd={tag_pos_rob[1]:.2f}m  pix_x={tag_pixel_cx:+.3f}",
                                (8, 58), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                elif auto_mode:
                    cv2.putText(frame, "Tag not visible -- stopped",
                                (8, 58), font, 0.5, (80, 80, 255), 1, cv2.LINE_AA)

                # Actual control output (what is being sent)
                _parts = {t.split(':')[0]: float(t.split(':')[1])
                          for t in drive_cmd.strip().rstrip(';').split(';') if ':' in t}
                cv2.putText(frame,
                            f"cmd  lx={_parts.get('lx',0):+.3f}  ly={_parts.get('ly',0):+.3f}  yaw={_parts.get('yaw',0):+.3f}",
                            (8, 78), font, 0.5, (200, 255, 200), 1, cv2.LINE_AA)
                # Autonomous planned effort -- always shown
                auto_col = (200, 200, 255) if tag_pos_rob is not None else (100, 100, 180)
                cv2.putText(frame,
                            f"auto lx={auto_lx:+.3f}  ly={auto_ly:+.3f}  yaw={auto_yaw:+.3f}",
                            (8, 98), font, 0.5, auto_col, 1, cv2.LINE_AA)

                speed_str = "ARM: FAST (Y)" if fast_mode else "ARM: normal (Y)"
                speed_col = (0, 80, 255) if fast_mode else (180, 180, 180)
                cv2.putText(frame, speed_str,
                            (FRAME_W - 160, FRAME_H - 10), font, 0.45, speed_col, 1, cv2.LINE_AA)
                cv2.putText(frame, "A=auto  X=sequence  B=manual",
                            (8, FRAME_H - 10), font, 0.45, (180, 180, 180), 1, cv2.LINE_AA)

                cv2.imshow("Autonomy Controller  (q / Esc = quit)", frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), 27):
                    break

            elapsed = time.monotonic() - t0
            sleep_for = loop_period - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)

    except KeyboardInterrupt:
        print("\nShutting down.")
    finally:
        drive_ser.write(b"lx:0.000;ly:0.000;yaw:0.000;\n")
        arm_ser.write(f"lift:{lift_sp:.3f};grip:{grip_sp:.3f};\n".encode())
        time.sleep(0.1)
        drive_ser.close()
        arm_ser.close()
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()
        pygame.quit()


if __name__ == "__main__":
    main()
