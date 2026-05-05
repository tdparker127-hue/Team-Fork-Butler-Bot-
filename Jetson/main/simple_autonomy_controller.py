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
MAX_LIFT_SPEED = 1.0   # rad/s
MAX_GRIP_SPEED = 1.5   # rad/s

# -- Xbox One BT axis / button indices -----------------------------------------
AXIS_LX = 0  # Left stick X  -> strafe
AXIS_LY = 1  # Left stick Y  -> forward
AXIS_RX = 2  # Right stick X -> yaw
AXIS_RY = 3  # unused
AXIS_LT = 4  # Left trigger  -> lift down
AXIS_RT = 5  # Right trigger -> lift up
BTN_LB     = 6   # grip close
BTN_RB     = 7   # grip open
BTN_AUTO   = 0   # A -- enter autonomous mode
BTN_MANUAL = 1   # B -- return to manual (instant override)

# -- Camera / AprilTag ---------------------------------------------------------
CAMERA_DEVICE = COLOR_CAMERA_DEVICE
CALIB_FILE    = Path(__file__).parent.parent / "vision" / "camera_calibration_live.npz"
FRAME_W, FRAME_H, FRAME_FPS = 640, 360, 15

# -- Autonomous control --------------------------------------------------------
TARGET_TAG_ID = 1      # AprilTag ID to drive toward
STOP_DIST_M   = 0.5    # desired forward distance from tag face [m]
K_FWD         = 0.6    # forward P-gain  (normalized speed / m error)
K_LAT         = 1.0    # lateral P-gain  (normalized speed / m)
K_YAW         = 0.5    # yaw P-gain from lateral error


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
) -> tuple:
    lift_sp = _clamp_range(
        lift_sp + (_trigger_depth(lt_raw) - _trigger_depth(rt_raw)) * MAX_LIFT_SPEED * dt,
        MIN_LIFT_RAD, MAX_LIFT_RAD,
    )
    grip_sp = _clamp_range(
        grip_sp + (int(rb_held) - int(lb_held)) * MAX_GRIP_SPEED * dt,
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
    auto_mode = False          # start in manual
    _a_prev = _b_prev = False

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

            # Mode switching (edge-triggered)
            if btn_a and not _a_prev:
                auto_mode = True
                print(f"\n[MODE] AUTONOMOUS -- driving toward tag {TARGET_TAG_ID}")
            if btn_b and not _b_prev:
                auto_mode = False
                print("\n[MODE] MANUAL")
            _a_prev, _b_prev = btn_a, btn_b

            # Get latest camera frame + detections
            with _cam_lock:
                frame      = _cam_frame.copy() if _cam_frame is not None else None
                detections = list(_cam_detections)

            # Locate target tag in robot frame [right, forward, up]
            tag_pos_rob = None
            for det in detections:
                if det.tag_id == TARGET_TAG_ID and det.pose_t is not None:
                    p_cam       = np.array(det.pose_t, dtype=float).ravel()
                    tag_pos_rob = R_cr @ p_cam + t_cr
                    break

            # Drive command
            if auto_mode and tag_pos_rob is not None:
                err_fwd = tag_pos_rob[1] - STOP_DIST_M
                err_lat = tag_pos_rob[0]
                lx_out  = max(-1.0, min(1.0, K_LAT * err_lat))
                ly_out  = max(-1.0, min(1.0, K_FWD * err_fwd))
                yaw_out = max(-1.0, min(1.0, K_YAW * err_lat))
                drive_cmd = f"lx:{lx_out:.3f};ly:{ly_out:.3f};yaw:{yaw_out:.3f};\n"
            elif auto_mode:
                drive_cmd = "lx:0.000;ly:0.000;yaw:0.000;\n"   # tag lost -- stop
            else:
                max_step = MAX_Drive_Slew * loop_period
                lx_cmd  = _slew(lx,  lx_cmd,  max_step)
                ly_cmd  = _slew(ly,  ly_cmd,  max_step)
                yaw_cmd = _slew(rx, yaw_cmd, max_step)
                drive_cmd = compute_drive_command(lx_cmd, ly_cmd, yaw_cmd)

            # Arm command
            lift_sp, grip_sp = step_arm_setpoints(
                lift_sp, grip_sp, lt, rt, lb, rb, loop_period,
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
                        p = R_cr @ np.array(det.pose_t, dtype=float).ravel() + t_cr
                        cv2.putText(frame, f"x={p[0]:+.2f} y={p[1]:+.2f} m",
                                    (cx_t - 55, cy_t + 18),
                                    font, 0.45, (0, 255, 255), 1, cv2.LINE_AA)

                # Mode banner
                mode_str = f"AUTO  (tag {TARGET_TAG_ID})" if auto_mode else "MANUAL"
                mode_col = (0, 220, 60) if auto_mode else (40, 80, 255)
                cv2.putText(frame, mode_str, (8, 30), font, 0.9, (0, 0, 0),    4, cv2.LINE_AA)
                cv2.putText(frame, mode_str, (8, 30), font, 0.9, mode_col, 2, cv2.LINE_AA)

                # Auto status line
                if auto_mode and tag_pos_rob is not None:
                    dist = math.hypot(tag_pos_rob[0], tag_pos_rob[1])
                    cv2.putText(frame,
                                f"dist={dist:.2f}m  lat={tag_pos_rob[0]:+.2f}m  fwd={tag_pos_rob[1]:.2f}m",
                                (8, 58), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                elif auto_mode:
                    cv2.putText(frame, "Tag not visible -- stopped",
                                (8, 58), font, 0.5, (80, 80, 255), 1, cv2.LINE_AA)

                cv2.putText(frame, "A=auto  B=manual",
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
