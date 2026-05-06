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
K_FWD          = 0.6    # forward P-gain  (normalized speed / m error)
K_LAT          = 1.5    # lateral P-gain  (normalized speed / normalized pixel error [-1,1])
K_YAW          = 0.8    # yaw P-gain from normalized pixel error


# ─── Sequence runner constants ─────────────────────────────────────────────────
SEQ_ARM_LIFT_SPEED = MAX_LIFT_SPEED   # max lift slew rate during sequence [rad/s]
SEQ_ARM_GRIP_SPEED = MAX_GRIP_SPEED   # max grip slew rate during sequence [rad/s]
SEQ_ARM_TOL_RAD    = 0.05             # arm arrival tolerance [rad]
SEQ_DRIVE_HOLD_S   = 0.4             # hold within drive tolerance this long before advancing [s]
SEQ_REACH_FWD_M    = 0.08            # forward error threshold [m]
SEQ_REACH_PIX_X    = 0.06            # lateral pixel tolerance (normalized [-1,1])
SEQ_YAW_TOL_DEG    = 3.0             # heading tolerance for turn_yaw steps [deg]
SEQ_YAW_HOLD_S     = 0.3             # hold within yaw tolerance before advancing [s]
K_TURN_DEG         = 0.1           # P-gain for turn_yaw:  yaw_cmd = clamp(K_TURN_DEG * err_deg)
K_D_TURN_DEG       = 0.0002           # D-gain for turn_yaw: damps using IMU yawRate [1/(deg/s)]
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
#
MISSIONS = {
    "Full Pickup": [
        {
            "type": "drive_arm",
            "tag": 6,
            "stop_dist": 1.7,
            "lat_off": 0.0,
            "lift": 3.5,
            "grip": 1.85,
        },
        # {"type": "turn_yaw", "yaw_deg": 0.0, "tol_deg": 2.0, "hold_s": 0.5},
        {"type": "drive_tag", "tag": 6, "stop_dist": 1.34, "lat_off": 0.0},
        {"type": "set_arm", "lift": 3.0, "grip": 1.85},
        {"type": "drive_tag", "tag": 6, "stop_dist": 0.77, "lat_off": 0.0}, # limit 0.62
        {"type": "set_arm", "lift": 3.0, "grip": 0.38},
        # {
        #     "type": "drive_arm",
        #     "tag": 6,
        #     "stop_dist": 0.25,
        #     "lat_off": 0.0,
        #     "lift": 1.5,
        #     "grip": 1.85,
        # },
        # {"type": "set_arm", "grip": 0.0},
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
                  active_name: str) -> None:
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

    # Hint strip at bottom
    hy = FRAME_H - 28
    cv2.line(canvas, (x0, hy - 8), (FRAME_W - 6, hy - 8), (60, 60, 60), 1)
    cv2.putText(canvas, "click=select  dbl=run",
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

    # Control state
    loop_period = 1.0 / 50.0
    lift_sp = grip_sp = 0.0
    lx_cmd = ly_cmd = yaw_cmd = 0.0
    fast_mode      = False   # start in normal speed
    seq_idx        = -1      # -1 = not running; >= 0 = current step index
    seq_hold_start = 0.0     # monotonic time when we entered the tolerance zone
    _mission_keys  = list(MISSIONS.keys())
    sel_mission    = 0                            # highlighted mission in sidebar
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
            btn_y = bool(joystick.get_button(BTN_SPEED_BOOST)) if joystick.get_numbuttons() > BTN_SPEED_BOOST else False
            hat   = joystick.get_hat(0) if joystick.get_numhats() > 0 else (0, 0)

            _mission_keys = list(MISSIONS.keys())

            # D-pad up/down navigates mission list (only when not running)
            if seq_idx < 0:
                if hat[1] == 1 and _hat_prev[1] != 1:
                    sel_mission = (sel_mission - 1) % len(_mission_keys)
                elif hat[1] == -1 and _hat_prev[1] != -1:
                    sel_mission = (sel_mission + 1) % len(_mission_keys)

            # Mode switching (edge-triggered)
            if btn_b and not _b_prev:               # B always cancels
                seq_idx   = -1
                print("\n[MODE] MANUAL")
            elif btn_x and not _x_prev:             # X starts selected mission
                active_name    = _mission_keys[sel_mission]
                active_seq     = MISSIONS[active_name]
                seq_idx        = 0
                seq_hold_start = 0.0
                print(f"\n[SEQ] '{active_name}' -- Step 1/{len(active_seq)}: {active_seq[0]['type']}")
            elif btn_a and not _a_prev and seq_idx < 0:   # A starts selected mission
                active_name    = _mission_keys[sel_mission]
                active_seq     = MISSIONS[active_name]
                seq_idx        = 0
                seq_hold_start = 0.0
                print(f"\n[SEQ] '{active_name}' -- Step 1/{len(active_seq)}: {active_seq[0]['type']}")
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
                    seq_idx        = 0
                    seq_hold_start = 0.0
                    print(f"\n[SEQ] '{active_name}' -- Step 1/{len(active_seq)}: {active_seq[0]['type']}")
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
                    _t_pos = None
                    _t_pix = None
                    for det in detections:
                        if det.tag_id == step["tag"] and det.pose_t is not None:
                            pc     = np.array(det.pose_t, dtype=float).ravel()
                            _t_pos = R_cr @ pc + t_cr
                            _t_pix = (float(det.corners[:, 0].mean()) - FRAME_W / 2) / (FRAME_W / 2)
                            break

                    # Optional final heading: present in step dict as "yaw_deg"
                    _step_yaw_deg  = step.get("yaw_deg", None)
                    _step_yaw_tol  = step.get("tol_deg", SEQ_YAW_TOL_DEG)

                    if _t_pos is not None:
                        _ef = _t_pos[1] - step.get("stop_dist", 0.5)
                        _el = _t_pix  - step.get("lat_off", 0.0)
                        seq_drive_lx  = max(-1., min(1., K_LAT * _el))
                        seq_drive_ly  = max(-1., min(1., K_FWD * _ef))
                        _in_pos_tol = abs(_ef) < SEQ_REACH_FWD_M and abs(_el) < SEQ_REACH_PIX_X

                        # Yaw: IMU PD runs throughout if yaw_deg specified; else pixel centering
                        if _step_yaw_deg is not None:
                            _imu      = get_imu("drive")
                            _imu_yaw  = -math.degrees(_imu.get("yaw", 0.0))
                            _yaw_rate = -math.degrees(_imu.get("yawRate", 0.0))
                            _yaw_err  = ((_step_yaw_deg - _imu_yaw + 180) % 360) - 180
                            seq_drive_yaw = max(-1., min(1.,
                                K_TURN_DEG * _yaw_err - K_D_TURN_DEG * _yaw_rate))
                            _yaw_settled = abs(_yaw_err) < _step_yaw_tol
                        else:
                            seq_drive_yaw = max(-1., min(1., K_YAW * _el))
                            _yaw_settled  = True   # no yaw requirement → always satisfied

                        _in_tol = _in_pos_tol and _yaw_settled
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
                    _imu = get_imu("drive")
                    # Negate: IMU CCW=positive, drive command CW=positive
                    _imu_yaw      = -math.degrees(_imu.get("yaw", 0.0))      # rad → deg, sign-flipped
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
                _cur_yaw = -math.degrees(get_imu("drive").get("yaw", 0.0))  # negated: CW=positive matches drive convention
                cv2.putText(canvas,
                            f"IMU yaw={_cur_yaw:+.1f} deg",
                            (8, 98), font, 0.5, (180, 220, 255), 1, cv2.LINE_AA)

                speed_str = "ARM: FAST (Y)" if fast_mode else "ARM: normal (Y)"
                speed_col = (0, 80, 255) if fast_mode else (180, 180, 180)
                cv2.putText(canvas, speed_str,
                            (FRAME_W - 160, FRAME_H - 10), font, 0.45, speed_col, 1, cv2.LINE_AA)
                cv2.putText(canvas, "A=run  X=run  B=cancel  p=person det",
                            (8, FRAME_H - 10), font, 0.45, (180, 180, 180), 1, cv2.LINE_AA)

            # Sidebar overlay (always drawn, even when camera unavailable)
            _draw_sidebar(canvas, sel_mission, seq_idx, active_name)

            cv2.imshow(WIN_NAME, canvas)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            elif key == ord('p'):
                person_detect_enabled = not person_detect_enabled
                state_str = "ON" if person_detect_enabled else "OFF"
                print(f"\n[PERSON] Detection {state_str}")

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
