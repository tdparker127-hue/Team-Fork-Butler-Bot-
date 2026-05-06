"""
simple_autonomy_controller.py
Jetson Nano -- manual drive + composable mission sequencer

Controls:
  A button (0) -> start selected mission
  B button (1) -> cancel mission / return to manual  (default on start)
  X button (2) -> start selected mission  (same as A)
  Y button (3) -> toggle fast arm mode
  D-pad up/down -> navigate mission list

Manual drive mapping:
  Left stick  -> forward / strafe
  Right stick X -> yaw
  LT / RT     -> arm lift down / up
  LB / RB     -> gripper close / open

OpenCV window:
  - Live camera feed with tag outlines and robot-frame x/y labels
  - Mode banner (MANUAL / SEQ step N)
  - Drive cmd and arm setpoint debug lines
  - Mission sidebar (click = select, double-click = run)
"""

import enum
import json
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
    PERSON_SLOW_SPEED,
    PERSON_DETECT_EVERY_N,
)
from Jetson.vision.person_detection import PersonDetector, PersonThreat

# -- Yaw offset persistence ---------------------------------------------------
YAW_OFFSET_FILE = Path(__file__).parent.parent / "yaw_offset.json"

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
SIDEBAR_W  = 200                    # pixel width of mission panel overlay
OVERLAY_X  = FRAME_W - SIDEBAR_W   # left edge of sidebar overlay on the frame
WIN_NAME   = "Autonomy Controller  (q / Esc = quit)"

# -- Autonomous control --------------------------------------------------------
TARGET_TAG_ID  = 6      # AprilTag ID highlighted in the camera overlay
K_FWD          = 2.2    # forward P-gain  (normalized speed / m error)
K_LAT          = 0.5    # lateral P-gain  (normalized speed / normalized pixel error [-1,1])
K_YAW          = 0.7    # yaw P-gain from normalized pixel error


# ─── Sequence runner constants ─────────────────────────────────────────────────
SEQ_ARM_LIFT_SPEED = MAX_LIFT_SPEED   # max lift slew rate during sequence [rad/s]
SEQ_ARM_GRIP_SPEED = MAX_GRIP_SPEED   # max grip slew rate during sequence [rad/s]
SEQ_ARM_TOL_RAD    = 0.05             # arm arrival tolerance [rad]
SEQ_DRIVE_HOLD_S   = 0.4             # hold within drive tolerance this long before advancing [s]
SEQ_REACH_FWD_M    = 0.1            # forward error threshold [m]
SEQ_REACH_PIX_X    = 0.08           # lateral pixel tolerance (normalized [-1,1])
SEQ_YAW_TOL_DEG    = 4.0             # heading tolerance for turn_yaw steps [deg]
SEQ_YAW_HOLD_S     = 0.3             # hold within yaw tolerance before advancing [s]
K_TURN_DEG         = 0.1           # P-gain for turn_yaw:  yaw_cmd = clamp(K_TURN_DEG * err_deg)
K_D_TURN_DEG       = 0.0002           # D-gain for turn_yaw: damps using IMU yawRate [1/(deg/s)]

# ── ZVU (zero-velocity update) IMU dead-reckoning assist ──────────────────────
# April tag pose is noisy at long range. Instead of steering on that jitter:
#   1. Stop briefly (ZVU_SETTLE_S) so vibrations die — the "zero velocity update":
#      pose noise drops dramatically when the platform is stationary.
#   2. Sample the tag. Freeze the drive direction (lx, ly) and capture the current
#      IMU heading as the reference to hold during dead-reckoning.
#   3. Drive for ZVU_INTERVAL_S seco
#  nds using frozen (lx, ly) for translation and
#      live IMU PD for heading — no April tag used during this leg.
#   4. Repeat until the tag is within ZVU_TRUST_DIST_M, then track it live.
ZVU_TRUST_DIST_M = 1.75    # switch to live tag tracking within this range [m]
ZVU_INTERVAL_S   = 2.5    # seconds to dead-reckon between ZVU tag checks  ← primary tuning knob
ZVU_SETTLE_S     = 0.35   # seconds to hold still before sampling the tag at a ZVU stop
ZVU_TIMEOUT_S    = 2.0    # if tag still absent after settle+timeout, resume on last heading
K_TURN_PRECISE     = 0.0005
#   40 deg error → 1.0 (full speed)


# ─── MISSIONS ─────────────────────────────────────────────────────────────────
# Add or edit named missions here.  Each mission is a list of steps.
# Select at runtime with D-pad up/down and press X to run.
#
# Step types:
#   "drive_tag"  tag, stop_dist, lat_off, [hold_s]
#   "set_arm"    [lift], [grip]              (omit a key to leave unchanged)
#   "drive_arm"  all drive_tag + set_arm params
#   "turn_yaw"   yaw_deg, [tol_deg], [hold_s]
#   "zvu_trust_d_ist":0 goes to 100% april tag trust
MISSIONS = {
    "DishRack Pickup": [
        {
            "type": "drive_arm",
            "tag": 6,
            "stop_dist": 1.73,
            "lat_off": 0.0,
            "lift": 3.5,
            "grip": 1.85,
            "zvu_trust_dist": 1.75,
        },
        # {"type": "turn_yaw", "yaw_deg": 0.0, "tol_deg": 2.0, "hold_s": 0.5},
        {
            "type": "drive_tag",
            "tag": 6,
            "stop_dist": 1.34,
            "lat_off": 0.0,
            # "zvu_trust_dist": 10.0,
        },
        {"type": "set_arm", "lift": 2.93, "grip": 1.85},
        {
            "type": "drive_tag",
            "tag": 6,
            "stop_dist": 0.72,
            "lat_off": 0.0,
        },  # limit 0.62
        {"type": "set_arm", "lift": 2.93, "grip": 0.38},
        {
            "type": "drive_arm",
            "tag": 6,
            "stop_dist": 1.5,
            "lat_off": 0.0,
            "lift": 3.6,
        },
        {"type": "turn_yaw", "yaw_deg": 180.0, "tol_deg": 2.0, "hold_s": 0.5},
        # {"type": "set_arm", "grip": 0.0},
    ],
    "DishRack Pickup&Dropoff": [
        {
            "type": "drive_arm",
            "tag": 6,
            "stop_dist": 1.7,
            "lat_off": 0.0,
            "lift": 3.5,
            "grip": 1.85,
            "zvu_trust_dist": 2.0,
        },
        # {"type": "turn_yaw", "yaw_deg": 0.0, "tol_deg": 2.0, "hold_s": 0.5},
        {"type": "drive_tag", "tag": 6, "stop_dist": 1.34, "lat_off": 0.0},
        {"type": "set_arm", "lift": 1.84, "grip": 1.85},
        {
            "type": "drive_tag",
            "tag": 6,
            "stop_dist": 0.72,
            "lat_off": 0.0,
        },  # limit 0.62
        {"type": "set_arm", "lift": 3.6, "grip": 0.38},
        {
            "type": "drive_arm",
            "tag": 6,
            "stop_dist": 1.5,
            "lat_off": 0.0,
            "lift": 3.6,
        },
        {"type": "turn_yaw", "yaw_deg": 180.0, "tol_deg": 2.0, "hold_s": 0.5},
        {
            "type": "drive_arm",
            "tag": 7,
            "stop_dist": 1.0,
            "lat_off": -0.3,
            "lift": 1.0,
            "zvu_trust_dist": 2.0,
        },
        # {"type": "drive_tag", "tag": 7, "stop_dist": 1.34, "lat_off": 0.0},
        {"type": "turn_yaw", "yaw_deg": -90.0, "tol_deg": 2.0, "hold_s": 0.5},
        {
            "type": "set_arm",
            "lift": 3.5,
            "tag": 8,
        },
        {"type": "drive_tag", "tag": 8, "stop_dist": -0.384, "lat_off": 0.6},
        {"type": "set_arm", "lift": 3.0, "grip": 1.85},
        {
            "type": "drive_arm",
            "tag": 8,
            "stop_dist": 1.5,
            "lat_off": 0.5,
            "lift": 1.0,
            "grip": 0.38,
        },
        # {"type": "set_arm", "grip": 0.0},
    ],
    "Temp Dish Drop Off": [
        {
            "type": "drive_arm",
            "tag": 5,
            "stop_dist": 1.87,
            "lat_off": 0.0,
            "lift": 3.5,
            "grip": 0.0,
            "zvu_trust_dist": 2.0,
        },
        {
            "type": "drive_arm",
            "tag": 5,
            "stop_dist": 1.34,
            "lift": 3.5,
            "grip": 0.0,
            "lat_off": 0.0,
        },
        {"type": "set_arm", "lift": 2.55, "grip": 0.0},
        {
            "type": "drive_tag",
            "tag": 5,
            "stop_dist": 0.98,
            "lat_off": 0.0,
        },
        {"type": "set_arm", "lift": 2.375, "grip": 1.85},
        {
            "type": "drive_tag",
            "tag": 5,
            "stop_dist": 1.75,
            "lat_off": 0.0,
        },
    ],
    "Approach Only": [
        {"type": "set_arm", "lift": 3.0, "grip": 1.85},
        {"type": "drive_tag", "tag": 6, "stop_dist": 0.5, "lat_off": 0.0},
    ],
    "Arm Home": [
        {"type": "set_arm", "lift": 0.0, "grip": 0.0},
    ],
    "April Tag Only": [
        {"type": "drive_tag", "tag": 6, "stop_dist": 0.5, "lat_off": 0.0},
    ],
    "Turn in Place": [
        {"type": "turn_yaw", "yaw_deg": 90.0, "tol_deg": 2.0, "hold_s": 0.5},
    ],
}


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


# -- Yaw offset helpers --------------------------------------------------------

_yaw_offset_deg: float = 0.0   # degrees; subtracted from raw IMU yaw (CW-positive convention)


def get_yaw_deg() -> float:
    """Offset-corrected drive yaw in degrees, wrapped to (-180, 180]. CW=positive."""
    raw = -math.degrees(get_imu("drive").get("yaw", 0.0))
    return ((raw - _yaw_offset_deg + 180) % 360) - 180


def _load_yaw_offset() -> float:
    """Load saved yaw offset from disk; returns 0.0 if file missing or invalid."""
    try:
        return float(json.loads(YAW_OFFSET_FILE.read_text()).get("offset_deg", 0.0))
    except Exception:
        return 0.0


def _save_yaw_offset(offset_deg: float) -> None:
    """Persist yaw offset to disk."""
    YAW_OFFSET_FILE.write_text(json.dumps({"offset_deg": round(offset_deg, 6)}))


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


# -- Sidebar (overlay on right strip of the camera frame) --------------------

# Shared state written by the mouse callback, consumed in the main loop.
_mouse_event = {"row": -1, "dbl": False}


def _mouse_cb(event, x, y, flags, param) -> None:  # noqa: ANN001
    """OpenCV mouse callback -- runs inside cv2.waitKey on the main thread."""
    if x < OVERLAY_X:
        return
    row_h   = 40
    y_start = 38
    for i in range(len(MISSIONS)):
        yt = y_start + i * row_h
        if yt - 2 <= y <= yt + row_h - 6:
            if event == cv2.EVENT_LBUTTONDBLCLK:
                _mouse_event["row"] = i
                _mouse_event["dbl"] = True
            elif event == cv2.EVENT_LBUTTONDOWN:
                _mouse_event["row"] = i
                _mouse_event["dbl"] = False
            return


def _draw_sidebar(canvas: np.ndarray, sel_mission: int, seq_idx: int,
                  active_name: str, start_steps: list, step_buf: str = "") -> None:
    """Draw the mission panel as a semi-transparent overlay on canvas."""
    font    = cv2.FONT_HERSHEY_SIMPLEX
    names   = list(MISSIONS.keys())
    row_h   = 40
    y_start = 38
    x0      = OVERLAY_X

    # Dim the right strip so text is readable over any camera content
    strip = canvas[:, x0:].astype(np.float32)
    canvas[:, x0:] = (strip * 0.22).clip(0, 255).astype(np.uint8)

    # Title + divider
    cv2.putText(canvas, "MISSIONS", (x0 + 8, 22),
                font, 0.52, (180, 180, 180), 1, cv2.LINE_AA)
    cv2.line(canvas, (x0, 28), (FRAME_W - 6, 28), (70, 70, 70), 1)

    for i, name in enumerate(names):
        y      = y_start + i * row_h
        is_sel = (i == sel_mission)
        is_run = (seq_idx >= 0 and name == active_name)

        if is_run:
            bg, fg = (0, 80, 30),   (60, 255, 120)
        elif is_sel:
            bg, fg = (50, 50, 110), (160, 160, 255)
        else:
            bg, fg = (35, 35, 35),  (100, 100, 100)

        cv2.rectangle(canvas,
                      (x0 + 2,        y - 2),
                      (FRAME_W - 4,   y + row_h - 6),
                      bg, -1)

        pfx, pc = (">", (255, 200, 60) if is_sel else (60, 255, 120)) \
                  if (is_run or is_sel) else (f"{i + 1}.", (70, 70, 70))
        cv2.putText(canvas, pfx,  (x0 + 6,  y + 15), font, 0.42, pc, 1, cv2.LINE_AA)

        display = name if len(name) <= 17 else name[:16] + "~"
        cv2.putText(canvas, display, (x0 + 24, y + 15), font, 0.42, fg, 1, cv2.LINE_AA)

        steps = MISSIONS[name]
        if is_run:
            sub, sc = f"step {seq_idx + 1}/{len(steps)}", (100, 220, 100)
        else:
            n = len(steps)
            sub, sc = f"{n} step{'s' if n != 1 else ''}", (70, 70, 70)
        cv2.putText(canvas, sub, (x0 + 24, y + 28), font, 0.37, sc, 1, cv2.LINE_AA)

        # Start-step badge on the right side of the row
        _ss = start_steps[i] + 1 if i < len(start_steps) else 1
        if i == sel_mission and step_buf:
            _badge = step_buf + "_"   # show live input with cursor
            _ss_col = (0, 255, 200)
        else:
            _badge = str(_ss)
            _ss_col = (0, 180, 255) if start_steps[i] > 0 else (55, 55, 55)
        cv2.putText(canvas, f">{_badge}", (FRAME_W - 36, y + 15),
                    font, 0.42, _ss_col, 1, cv2.LINE_AA)

    # Hint strip at bottom
    hy = FRAME_H - 28
    cv2.line(canvas, (x0, hy - 8), (FRAME_W - 6, hy - 8), (60, 60, 60), 1)
    cv2.putText(canvas, "U/D=mission  L/R=step  dbl=run",
                (x0 + 4, hy),      font, 0.35, (90, 90, 90), 1, cv2.LINE_AA)
    cv2.putText(canvas, "X=run  B=cancel",
                (x0 + 4, hy + 14), font, 0.35, (90, 90, 90), 1, cv2.LINE_AA)


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

    global _yaw_offset_deg

    # Control state
    loop_period = 1.0 / 50.0
    lift_sp = grip_sp = 0.0
    lx_cmd = ly_cmd = yaw_cmd = 0.0
    fast_mode      = False   # start in normal speed
    seq_idx        = -1      # -1 = not running; >= 0 = current step index
    seq_hold_start = 0.0     # monotonic time when we entered the tolerance zone

    # ── ZVU state — resets on every new drive_tag / drive_arm step ────────────
    # _seq_step_prev: remembers which step index was active last iteration so we
    #                 can detect the moment seq_idx advances to a new step.
    _seq_step_prev   = -1
    # Phase starts nominally as "tag_trust" but is immediately overwritten on the
    # first step (because _seq_step_prev=-1 != seq_idx) with "zvu_stop".
    _zvu_phase       = "tag_trust"  # "tag_trust" | "zvu_stop" | "dead_reckon"
    _zvu_settle_t    = 0.0          # time.monotonic() when robot stopped for ZVU
    _zvu_leg_start   = 0.0          # time.monotonic() when current dead-reckon leg began
    _zvu_drive_lx    = 0.0          # lateral command frozen from last ZVU fix
    _zvu_drive_ly    = 0.0          # forward command frozen from last ZVU fix
    # _zvu_heading_deg: the IMU yaw captured at the last ZVU fix. The dead-reckon
    # leg runs live IMU PD to hold this heading — no pixel centering, no encoders.
    _zvu_heading_deg = 0.0
    _mission_keys  = list(MISSIONS.keys())
    sel_mission    = 0                            # highlighted mission in sidebar
    _start_steps   = [0] * len(_mission_keys)     # 0-indexed start step per mission
    _step_buf      = ""                           # digit buffer for step entry
    active_name    = _mission_keys[0]             # name of running/last mission
    active_seq     = MISSIONS[active_name]        # step list currently running
    _a_prev = _b_prev = _x_prev = _y_prev = False
    _hat_prev = (0, 0)

    print("Running at 50 Hz.  Ctrl-C or q/Esc in window to stop.")

    # Person detector — instantiated once so the model loads before the loop
    print("[PERSON] Loading YOLO person detector...")
    person_detector       = PersonDetector()
    person_threat         = PersonThreat.CLEAR
    person_detect_enabled = True    # toggled with 'p' key in OpenCV window
    _person_detect_ctr    = 0       # counts loop iterations; detect every PERSON_DETECT_EVERY_N
    _person_annotated     = None    # cached annotated frame from last detection
    print("[PERSON] Ready.")

    cv2.namedWindow(WIN_NAME)
    cv2.setMouseCallback(WIN_NAME, _mouse_cb)

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
            btn_x = bool(joystick.get_button(BTN_SEQ))         if joystick.get_numbuttons() > BTN_SEQ         else False
            btn_y  = bool(joystick.get_button(BTN_SPEED_BOOST)) if joystick.get_numbuttons() > BTN_SPEED_BOOST else False
            hat    = joystick.get_hat(0) if joystick.get_numhats() > 0 else (0, 0)

            _mission_keys = list(MISSIONS.keys())

            # D-pad navigation (only when not running a sequence)
            if seq_idx < 0:
                _n_steps_sel = len(MISSIONS[_mission_keys[sel_mission]])
                if hat[1] == 1 and _hat_prev[1] != 1:        # up   -> prev mission
                    sel_mission = (sel_mission - 1) % len(_mission_keys)
                    _step_buf = ""
                elif hat[1] == -1 and _hat_prev[1] != -1:    # down -> next mission
                    sel_mission = (sel_mission + 1) % len(_mission_keys)
                    _step_buf = ""
                elif hat[0] == 1 and _hat_prev[0] != 1:      # right -> increment start step
                    _step_buf = ""
                    _start_steps[sel_mission] = min(_start_steps[sel_mission] + 1, _n_steps_sel - 1)
                    print(f"\n[SEQ] '{_mission_keys[sel_mission]}' start step -> {_start_steps[sel_mission] + 1}")
                elif hat[0] == -1 and _hat_prev[0] != -1:    # left  -> decrement start step
                    _step_buf = ""
                    _start_steps[sel_mission] = max(_start_steps[sel_mission] - 1, 0)
                    print(f"\n[SEQ] '{_mission_keys[sel_mission]}' start step -> {_start_steps[sel_mission] + 1}")

            # Mode switching (edge-triggered)
            # Helper: flush any pending step-buffer digit before launching
            def _flush_step_buf():
                if _step_buf:
                    digit = int(_step_buf)
                    n_steps = len(MISSIONS[_mission_keys[sel_mission]])
                    _start_steps[sel_mission] = min(max(digit - 1, 0), n_steps - 1)
                    print(f"\n[SEQ] '{_mission_keys[sel_mission]}' start step (auto-commit) -> {_start_steps[sel_mission] + 1}")
                return _start_steps[sel_mission]

            if btn_b and not _b_prev:               # B always cancels
                seq_idx   = -1
                _step_buf = ""
                print("\n[MODE] MANUAL")
            elif btn_x and not _x_prev:             # X starts selected mission
                active_name    = _mission_keys[sel_mission]
                active_seq     = MISSIONS[active_name]
                seq_idx        = _flush_step_buf()
                _step_buf      = ""
                seq_hold_start = 0.0
                print(f"\n[SEQ] '{active_name}' -- Step {seq_idx + 1}/{len(active_seq)}: {active_seq[seq_idx]['type']}")
            elif btn_a and not _a_prev and seq_idx < 0:   # A starts selected mission
                active_name    = _mission_keys[sel_mission]
                active_seq     = MISSIONS[active_name]
                seq_idx        = _flush_step_buf()
                _step_buf      = ""
                seq_hold_start = 0.0
                print(f"\n[SEQ] '{active_name}' -- Step {seq_idx + 1}/{len(active_seq)}: {active_seq[seq_idx]['type']}")
            if btn_y and not _y_prev:
                fast_mode = not fast_mode
                print(f"\n[SPEED] Arm: {'FAST' if fast_mode else 'NORMAL'}")
            _a_prev, _b_prev, _x_prev, _y_prev = btn_a, btn_b, btn_x, btn_y
            _hat_prev = hat

            # Mouse click events (set by _mouse_cb inside cv2.waitKey)
            _mev_row = _mouse_event["row"]
            _mev_dbl = _mouse_event["dbl"]
            _mouse_event["row"] = -1
            _mouse_event["dbl"] = False
            if _mev_row >= 0 and seq_idx < 0:
                if _mev_dbl:
                    sel_mission    = _mev_row
                    active_name    = _mission_keys[sel_mission]
                    active_seq     = MISSIONS[active_name]
                    seq_idx        = _flush_step_buf()
                    _step_buf      = ""
                    seq_hold_start = 0.0
                    print(f"\n[SEQ] '{active_name}' -- Step {seq_idx + 1}/{len(active_seq)}: {active_seq[seq_idx]['type']}")
                else:
                    sel_mission = _mev_row

            # Get latest camera frame + detections
            with _cam_lock:
                frame      = _cam_frame.copy() if _cam_frame is not None else None
                detections = list(_cam_detections)

            # ── Sequence runner ───────────────────────────────────────────────
            seq_drive_lx = seq_drive_ly = seq_drive_yaw = 0.0
            seq_arm_owns = False   # True = sequence owns arm (block joystick)
            if seq_idx >= 0:
                step  = active_seq[seq_idx]
                stype = step["type"]
                now   = time.monotonic()

                # ── ZVU step-transition reset ─────────────────────────────────
                # Fires once each time seq_idx advances to a new step.
                # We always enter the new step in "zvu_stop" so the robot
                # pauses briefly and gets a fresh tag fix before moving.
                if seq_idx != _seq_step_prev:
                    _seq_step_prev   = seq_idx
                    _zvu_phase       = "zvu_stop"  # stop first, read tag, then decide
                    _zvu_settle_t    = now          # start the settle timer immediately
                    _zvu_leg_start   = 0.0          # no active dead-reckon leg yet
                    _zvu_drive_lx    = 0.0          # no frozen translation yet
                    _zvu_drive_ly    = 0.0
                    _zvu_heading_deg = 0.0          # no frozen heading yet

                # Arm slewing (set_arm and drive_arm) -- fixed rate for safety
                if stype in ("set_arm", "drive_arm"):
                    seq_arm_owns = True
                    lift_tgt = step.get("lift", lift_sp)
                    grip_tgt = step.get("grip", grip_sp)
                    lift_sp  = _slew(lift_tgt, lift_sp, SEQ_ARM_LIFT_SPEED * loop_period)
                    grip_sp  = _slew(grip_tgt, grip_sp, SEQ_ARM_GRIP_SPEED * loop_period)
                    arm_done = (abs(lift_sp - lift_tgt) < SEQ_ARM_TOL_RAD and
                                abs(grip_sp - grip_tgt) < SEQ_ARM_TOL_RAD)
                    print(f"\r[ARM] step={seq_idx} tgt=({lift_tgt:.2f},{grip_tgt:.2f}) "
                          f"sp=({lift_sp:.2f},{grip_sp:.2f}) done={arm_done}    ", end="", flush=True)
                else:
                    arm_done = True

                # Drive control (drive_tag and drive_arm)
                if stype in ("drive_tag", "drive_arm"):
                    if "tag" not in step:
                        raise KeyError(
                            f"Mission '{active_name}' step {seq_idx + 1} (type='{stype}') "
                            f"is missing the required 'tag' key. "
                            f"Every drive_tag and drive_arm step must include e.g. \"tag\": 6. "
                            f"Step contents: {step}"
                        )
                    # --- Detect the target tag in the current frame -----------
                    # pose_t is the camera-frame translation to the tag center.
                    # We rotate it into robot frame with R_cr / t_cr so that
                    # _t_pos[1] is the forward distance to the tag.
                    _t_pos = None
                    _t_pix = None
                    for det in detections:
                        if det.tag_id == step["tag"] and det.pose_t is not None:
                            pc     = np.array(det.pose_t, dtype=float).ravel()
                            _t_pos = R_cr @ pc + t_cr   # [x, forward, z] in robot frame
                            # Normalize tag center x to [-1, +1]: 0 = image center,
                            # +1 = right edge, -1 = left edge.
                            _t_pix = (float(det.corners[:, 0].mean()) - FRAME_W / 2) / (FRAME_W / 2)
                            break

                    # Optional explicit heading and per-step ZVU overrides
                    _step_yaw_deg = step.get("yaw_deg", None)
                    _step_yaw_tol = step.get("tol_deg", SEQ_YAW_TOL_DEG)
                    _trust_dist   = step.get("zvu_trust_dist", ZVU_TRUST_DIST_M)
                    _zvu_interval = step.get("zvu_interval",   ZVU_INTERVAL_S)

                    # --- Automatic promotion to tag_trust --------------------
                    # If the tag is visible and within reliable range, skip all
                    # ZVU machinery and fall straight through to live tracking.
                    # This is an upgrade-only transition: we never go back to
                    # dead_reckon once we're close enough.
                    if _t_pos is not None and _t_pos[1] <= _trust_dist:
                        _zvu_phase = "tag_trust"

                    # ── Phase: tag_trust ──────────────────────────────────────
                    # Standard behavior: recompute drive commands from the live
                    # tag pose every loop iteration. At close range the pose
                    # estimate is stable enough to use directly.
                    if _zvu_phase == "tag_trust":
                        if _t_pos is not None:
                            _ef = _t_pos[1] - step.get("stop_dist", 0.5)   # forward error [m]
                            _el = _t_pix - step.get("lat_off", 0.0)        # lateral error [norm px]
                            seq_drive_lx = max(-1., min(1., K_LAT * _el))
                            seq_drive_ly = max(-1., min(1., K_FWD * _ef))
                            _in_pos_tol  = abs(_ef) < SEQ_REACH_FWD_M and abs(_el) < SEQ_REACH_PIX_X

                            # Yaw: IMU PD if yaw_deg specified, otherwise pixel-centering
                            if _step_yaw_deg is not None:
                                _imu      = get_imu("drive")
                                _imu_yaw  = ((- math.degrees(_imu.get("yaw", 0.0)) - _yaw_offset_deg + 180) % 360) - 180
                                _yaw_rate = -math.degrees(_imu.get("yawRate", 0.0))
                                _yaw_err  = ((_step_yaw_deg - _imu_yaw + 180) % 360) - 180
                                seq_drive_yaw = max(-1., min(1.,
                                    K_TURN_DEG * _yaw_err - K_D_TURN_DEG * _yaw_rate))
                                _yaw_settled = abs(_yaw_err) < _step_yaw_tol
                            else:
                                seq_drive_yaw = max(-1., min(1., K_YAW * _el))
                                _yaw_settled  = True   # no explicit heading requirement

                            _in_tol = _in_pos_tol and _yaw_settled
                        else:
                            # Tag lost at close range — hold position and wait for it
                            seq_drive_lx = seq_drive_ly = seq_drive_yaw = 0.0
                            _in_tol = False

                    # ── Phase: zvu_stop ───────────────────────────────────────
                    # Robot is stationary waiting for vibrations to decay.
                    # We only read the tag after ZVU_SETTLE_S seconds — that's
                    # the "zero velocity" moment when pose noise is lowest.
                    elif _zvu_phase == "zvu_stop":
                        seq_drive_lx = seq_drive_ly = 0.0

                        # Keep heading tight with IMU while stopped. If we have a
                        # previously frozen heading use it; otherwise hold zero.
                        # This prevents yaw drift from corrupting the fix we're
                        # about to take.
                        _hold_yaw = _step_yaw_deg if _step_yaw_deg is not None else (
                            _zvu_heading_deg if _zvu_drive_ly != 0.0 else None)
                        if _hold_yaw is not None:
                            _imu_s      = get_imu("drive")
                            _imu_yaw_s  = ((- math.degrees(_imu_s.get("yaw", 0.0)) - _yaw_offset_deg + 180) % 360) - 180
                            _yaw_rate_s = -math.degrees(_imu_s.get("yawRate", 0.0))
                            _yaw_err_s  = ((_hold_yaw - _imu_yaw_s + 180) % 360) - 180
                            seq_drive_yaw = max(-1., min(1.,
                                K_TURN_DEG * _yaw_err_s - K_D_TURN_DEG * _yaw_rate_s))
                        else:
                            seq_drive_yaw = 0.0

                        _elapsed_stop = now - _zvu_settle_t
                        if _elapsed_stop >= ZVU_SETTLE_S:
                            if _t_pos is not None:
                                # Good stationary fix: compute translation commands
                                # and freeze them for the upcoming dead-reckon leg.
                                _ef = _t_pos[1] - step.get("stop_dist", 0.5)
                                _el = _t_pix - step.get("lat_off", 0.0)
                                _zvu_drive_lx = max(-1., min(1., K_LAT * _el))
                                _zvu_drive_ly = max(-1., min(1., K_FWD * _ef))
                                # Capture current IMU heading as the reference to
                                # maintain during dead-reckoning. This is the core of
                                # the IMU-based ZVU: we know the exact heading at the
                                # moment of the stationary fix, so we can hold it
                                # precisely with IMU PD — no encoders, no pixel jitter.
                                _imu_fix         = get_imu("drive")
                                _zvu_heading_deg = ((- math.degrees(_imu_fix.get("yaw", 0.0)) - _yaw_offset_deg + 180) % 360) - 180
                                print(f"\r[ZVU] fix  dist={_t_pos[1]:.2f}m  hdg={_zvu_heading_deg:.1f}°  ly={_zvu_drive_ly:.3f}  lx={_zvu_drive_lx:.3f}   ",
                                      end="", flush=True)
                                if _t_pos[1] <= _trust_dist:
                                    _zvu_phase = "tag_trust"
                                else:
                                    _zvu_phase     = "dead_reckon"
                                    _zvu_leg_start = now   # start the timed leg
                            elif _elapsed_stop >= ZVU_SETTLE_S + ZVU_TIMEOUT_S:
                                # Tag never appeared. If we have a saved heading,
                                # resume on it; otherwise keep waiting.
                                if _zvu_drive_ly != 0.0:
                                    print(f"\r[ZVU] timeout — resuming dead-reckon           ",
                                          flush=True)
                                    _zvu_phase     = "dead_reckon"
                                    _zvu_leg_start = now

                        _in_tol = False   # never declare arrival while stopped for ZVU

                    # ── Phase: dead_reckon ────────────────────────────────────
                    # Drive on the translation commands frozen at the last fix.
                    # Heading is held by live IMU PD on _zvu_heading_deg — the
                    # yaw captured at the stationary fix moment. No April tag or
                    # encoder is used here; this is pure IMU dead-reckoning.
                    # After ZVU_INTERVAL_S seconds, stop for another tag fix.
                    elif _zvu_phase == "dead_reckon":
                        _elapsed_leg = now - _zvu_leg_start

                        if _elapsed_leg >= _zvu_interval:
                            # Timed interval complete — stop for a ZVU tag check
                            print(f"\r[ZVU] {_elapsed_leg:.1f}s leg done — stopping for fix   ",
                                  flush=True)
                            _zvu_phase    = "zvu_stop"
                            _zvu_settle_t = now
                            seq_drive_lx = seq_drive_ly = seq_drive_yaw = 0.0
                        else:
                            # Mid-interval: frozen translation + live IMU heading.
                            # If yaw_deg is set in the step, prefer that target;
                            # otherwise use the heading captured at the last fix.
                            seq_drive_lx = _zvu_drive_lx
                            seq_drive_ly = _zvu_drive_ly
                            _heading_target = _step_yaw_deg if _step_yaw_deg is not None else _zvu_heading_deg
                            _imu_dr      = get_imu("drive")
                            _imu_yaw_dr  = ((- math.degrees(_imu_dr.get("yaw", 0.0)) - _yaw_offset_deg + 180) % 360) - 180
                            _yaw_rate_dr = -math.degrees(_imu_dr.get("yawRate", 0.0))
                            _yaw_err_dr  = ((_heading_target - _imu_yaw_dr + 180) % 360) - 180
                            seq_drive_yaw = max(-1., min(1.,
                                K_TURN_DEG * _yaw_err_dr - K_D_TURN_DEG * _yaw_rate_dr))

                        _in_tol = False   # never declare arrival during dead-reckoning

                    # --- Hold timer (same logic as before) -------------------
                    # seq_hold_start counts consecutive time inside tolerance.
                    # In zvu_stop and dead_reckon _in_tol is always False, so
                    # seq_hold_start is continuously reset — arrival is only
                    # possible once we're in tag_trust and on target.
                    if _in_tol:
                        if seq_hold_start == 0.0:
                            seq_hold_start = now
                        drive_done = (now - seq_hold_start) >= step.get("hold_s", SEQ_DRIVE_HOLD_S)
                    else:
                        seq_hold_start = 0.0
                        drive_done = False

                elif stype == "turn_yaw":
                    _imu = get_imu("drive")
                    # Negate: IMU CCW=positive, drive command CW=positive; apply yaw offset
                    _imu_yaw      = ((-math.degrees(_imu.get("yaw", 0.0)) - _yaw_offset_deg + 180) % 360) - 180
                    _yaw_rate_dps = -math.degrees(_imu.get("yawRate", 0.0))  # rad/s → deg/s, sign-flipped
                    _err_deg = ((step["yaw_deg"] - _imu_yaw + 180) % 360) - 180
                    # PD controller: P on heading error, D damps via live yaw rate
                    _pd_out = K_TURN_DEG * _err_deg - K_D_TURN_DEG * _yaw_rate_dps
                    print(f"\r[YAW] _pd_out={_pd_out:.3f} err={_err_deg:.2f}° imu_yaw={_imu_yaw:.2f}° yaw_rate={_yaw_rate_dps:.2f}°/s    ", end="", flush=True)
                    seq_drive_yaw = max(-1., min(1., _pd_out))
                    _in_tol = abs(_err_deg) < step.get("tol_deg", SEQ_YAW_TOL_DEG)
                    if _in_tol:
                        # seq_drive_yaw = max(-1.0, min(1.0, K_TURN_PRECISE * _err_deg))
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
                    if seq_idx >= len(active_seq):
                        seq_idx = -1
                        print(f"\n[SEQ] '{active_name}' complete -- returning to MANUAL")
                    else:
                        print(f"\n[SEQ] Step {seq_idx + 1}/{len(active_seq)}: {active_seq[seq_idx]['type']}")

            # ── Drive command ─────────────────────────────────────────────────
            if seq_idx >= 0:
                _stype = active_seq[seq_idx]["type"]
                if _stype in ("drive_tag", "drive_arm", "turn_yaw"):
                    drive_cmd = f"lx:{seq_drive_lx:.3f};ly:{seq_drive_ly:.3f};yaw:{seq_drive_yaw:.3f};\n"
                else:
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

            # ── Person detection (cap drive_cmd before writing) ───────────────
            _person_detect_ctr += 1
            if person_detect_enabled and frame is not None and (_person_detect_ctr % PERSON_DETECT_EVERY_N == 0):
                person_threat, _person_dets, _person_annotated = person_detector.detect(frame)
            elif not person_detect_enabled:
                person_threat     = PersonThreat.CLEAR
                _person_annotated = None

            if person_threat == PersonThreat.STOP:
                drive_cmd = "lx:0.000;ly:0.000;yaw:0.000;\n"
            elif person_threat == PersonThreat.SLOW:
                _parts_raw = {t.split(':')[0]: float(t.split(':')[1])
                              for t in drive_cmd.strip().rstrip(';').split(';') if ':' in t}
                _cap = PERSON_SLOW_SPEED
                drive_cmd = (f"lx:{max(-_cap, min(_cap, _parts_raw.get('lx', 0))):.3f};"
                             f"ly:{max(-_cap, min(_cap, _parts_raw.get('ly', 0))):.3f};"
                             f"yaw:{max(-_cap, min(_cap, _parts_raw.get('yaw', 0))):.3f};\n")

            drive_ser.write(drive_cmd.encode())
            arm_ser.write(f"lift:{lift_sp:.3f};grip:{grip_sp:.3f};\n".encode())

            # OpenCV window
            canvas = np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)
            font   = cv2.FONT_HERSHEY_SIMPLEX

            if frame is not None:
                if frame.shape[0] != FRAME_H or frame.shape[1] != FRAME_W:
                    frame = cv2.resize(frame, (FRAME_W, FRAME_H))
                canvas[:] = frame

                # Person detection bounding boxes (drawn first, behind other overlays)
                if _person_annotated is not None:
                    _pa = _person_annotated
                    if _pa.shape[0] != FRAME_H or _pa.shape[1] != FRAME_W:
                        _pa = cv2.resize(_pa, (FRAME_W, FRAME_H))
                    canvas[:] = _pa

                # Tag outlines + robot-frame position labels
                for det in detections:
                    pts = det.corners.astype(int)
                    col = (0, 255, 60) if det.tag_id == TARGET_TAG_ID else (0, 200, 200)
                    cv2.polylines(canvas, [pts.reshape(-1, 1, 2)], True, col, 2)
                    cx_t = int(pts[:, 0].mean())
                    cy_t = int(pts[:, 1].mean())
                    cv2.putText(canvas, f"id={det.tag_id}", (cx_t - 20, cy_t - 10),
                                font, 0.6, col, 2, cv2.LINE_AA)
                    if det.pose_t is not None:
                        p_cam = np.array(det.pose_t, dtype=float).ravel()
                        p     = R_cr @ p_cam + t_cr
                        cv2.putText(canvas, f"rob x={p[0]:+.2f} y={p[1]:+.2f} m",
                                    (cx_t - 70, cy_t + 18),
                                    font, 0.45, (0, 0, 255), 1, cv2.LINE_AA)
                        cv2.putText(canvas, f"cam x={p_cam[0]:+.2f} z={p_cam[2]:+.2f} m",
                                    (cx_t - 70, cy_t + 34),
                                    font, 0.45, (255, 180, 0), 1, cv2.LINE_AA)

                # Mode banner (top-left)
                if seq_idx >= 0:
                    mode_str = f"SEQ {seq_idx + 1}/{len(active_seq)}: {active_seq[seq_idx]['type']}  (B=cancel)"
                    mode_col = (0, 180, 255)
                else:
                    mode_str = "MANUAL"
                    mode_col = (40, 80, 255)
                cv2.putText(canvas, mode_str, (8, 30), font, 0.9, (0, 0, 0),    4, cv2.LINE_AA)
                cv2.putText(canvas, mode_str, (8, 30), font, 0.9, mode_col, 2, cv2.LINE_AA)

                # Person threat banner (top-right, before sidebar)
                _threat_labels = {
                    PersonThreat.CLEAR: ("PERSON: CLEAR", (0, 200, 0)),
                    PersonThreat.SLOW:  ("PERSON: SLOW",  (0, 165, 255)),
                    PersonThreat.STOP:  ("PERSON: STOP",  (0, 0, 220)),
                }
                if not person_detect_enabled:
                    _thr_str, _thr_col = "PERSON: OFF (p)", (120, 120, 120)
                else:
                    _thr_str, _thr_col = _threat_labels[person_threat]
                (_thr_w, _thr_h), _ = cv2.getTextSize(_thr_str, font, 0.75, 2)
                _thr_x = OVERLAY_X - _thr_w - 8
                cv2.putText(canvas, _thr_str, (_thr_x, 56), font, 0.75, (0, 0, 0),  4, cv2.LINE_AA)
                cv2.putText(canvas, _thr_str, (_thr_x, 56), font, 0.75, _thr_col, 2, cv2.LINE_AA)

                # Speed mode indicator -- keep left of the sidebar overlay
                spd_str = "FAST ARM" if fast_mode else "normal arm"
                spd_col = (0, 80, 255) if fast_mode else (160, 160, 160)
                (spd_w, _), _ = cv2.getTextSize(spd_str, font, 0.7, 2)
                spd_x = OVERLAY_X - spd_w - 12
                cv2.putText(canvas, spd_str, (spd_x, 30), font, 0.7, (0, 0, 0),  4, cv2.LINE_AA)
                cv2.putText(canvas, spd_str, (spd_x, 30), font, 0.7, spd_col,    2, cv2.LINE_AA)

                # Drive output overlay
                _parts = {t.split(':')[0]: float(t.split(':')[1])
                          for t in drive_cmd.strip().rstrip(';').split(';') if ':' in t}
                cv2.putText(canvas,
                            f"cmd  lx={_parts.get('lx',0):+.3f}  ly={_parts.get('ly',0):+.3f}  yaw={_parts.get('yaw',0):+.3f}",
                            (8, 58), font, 0.5, (200, 255, 200), 1, cv2.LINE_AA)
                arm_owns_str = "seq" if seq_arm_owns else "joy"
                cv2.putText(canvas,
                            f"arm [{arm_owns_str}][step {seq_idx}]  lift={lift_sp:.3f}  grip={grip_sp:.3f}",
                            (8, 78), font, 0.5, (255, 200, 100), 1, cv2.LINE_AA)
                _cur_yaw = get_yaw_deg()
                _off_str = f" [off {_yaw_offset_deg:+.1f}°]" if abs(_yaw_offset_deg) > 0.05 else ""
                _disp_tag_id = (step["tag"] if seq_idx >= 0 and stype in ("drive_tag", "drive_arm")
                                else TARGET_TAG_ID)
                _disp_det = next((d for d in detections
                                  if d.tag_id == _disp_tag_id and d.pose_t is not None), None)
                if _disp_det is not None:
                    _dc = np.array(_disp_det.pose_t, dtype=float).ravel()
                    _tag_dbg = (f"tag{_disp_tag_id}  cam_x={_dc[0]:+.3f}m"
                                f"  cam_z={_dc[2]:.3f}m  yaw={_cur_yaw:+.1f}°{_off_str}")
                else:
                    _tag_dbg = f"tag{_disp_tag_id}: no detection  yaw={_cur_yaw:+.1f}°{_off_str}"
                cv2.putText(canvas, _tag_dbg, (8, 98), font, 0.5, (180, 220, 255), 1, cv2.LINE_AA)

                speed_str = "ARM: FAST (Y)" if fast_mode else "ARM: normal (Y)"
                speed_col = (0, 80, 255) if fast_mode else (180, 180, 180)
                cv2.putText(canvas, speed_str,
                            (FRAME_W - 160, FRAME_H - 10), font, 0.45, speed_col, 1, cv2.LINE_AA)
                cv2.putText(canvas, "A=run  X=run  B=cancel  p=person  z=zero yaw  l=load yaw",
                            (8, FRAME_H - 10), font, 0.45, (180, 180, 180), 1, cv2.LINE_AA)

            # Sidebar overlay (always drawn, even when camera unavailable)
            _draw_sidebar(canvas, sel_mission, seq_idx, active_name, _start_steps, _step_buf)

            cv2.imshow(WIN_NAME, canvas)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            elif key == ord('a'):   # print current AprilTag readings
                with _cam_lock:
                    _snap = list(_cam_detections)
                if _snap:
                    print(f"\n[TAGS] {len(_snap)} tag(s) detected:")
                    for d in _snap:
                        t = d.pose_t.flatten()
                        print(f"  tag_id={d.tag_id}  x={t[0]:.3f}m  y={t[1]:.3f}m  z={t[2]:.3f}m  margin={d.decision_margin:.1f}")
                else:
                    print("\n[TAGS] No tags detected in current frame.")
            elif key == ord('p'):
                person_detect_enabled = not person_detect_enabled
                state_str = "ON" if person_detect_enabled else "OFF"
                print(f"\n[PERSON] Detection {state_str}")
            elif key == ord('z'):   # zero yaw at current position and save
                _raw_yaw = -math.degrees(get_imu("drive").get("yaw", 0.0))
                _yaw_offset_deg = _raw_yaw
                _save_yaw_offset(_yaw_offset_deg)
                print(f"\n[YAW] Zeroed. New offset = {_yaw_offset_deg:+.3f}°  (saved to {YAW_OFFSET_FILE})")
            elif key == ord('l'):   # load saved yaw offset from disk
                _yaw_offset_deg = _load_yaw_offset()
                print(f"\n[YAW] Loaded offset = {_yaw_offset_deg:+.3f}°  from {YAW_OFFSET_FILE}")
            elif ord('0') <= key <= ord('9') and seq_idx < 0:
                _step_buf += chr(key)
                if len(_step_buf) == 2:   # auto-commit after 2 digits
                    digit = int(_step_buf)
                    n_steps = len(MISSIONS[_mission_keys[sel_mission]])
                    _start_steps[sel_mission] = min(max(digit - 1, 0), n_steps - 1)
                    print(f"\n[SEQ] '{_mission_keys[sel_mission]}' start step -> {_start_steps[sel_mission] + 1}")
                    _step_buf = ""
            elif key in (13, 10) and seq_idx < 0 and _step_buf:  # Enter commits 1-digit entry (CR or LF)
                digit = int(_step_buf)
                n_steps = len(MISSIONS[_mission_keys[sel_mission]])
                _start_steps[sel_mission] = min(max(digit - 1, 0), n_steps - 1)
                print(f"\n[SEQ] '{_mission_keys[sel_mission]}' start step -> {_start_steps[sel_mission] + 1}")
                _step_buf = ""
            elif key == 8 and seq_idx < 0:   # Backspace clears buffer or resets to step 1
                if _step_buf:
                    _step_buf = _step_buf[:-1]
                else:
                    _start_steps[sel_mission] = 0
                    print(f"\n[SEQ] '{_mission_keys[sel_mission]}' start step -> 1")

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
