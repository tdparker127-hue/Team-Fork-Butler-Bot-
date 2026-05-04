"""
test_waypoint_apriltag.py — Waypoint controller integration test.

Drives the robot straight onto a target AprilTag along its outward normal,
stops at a configurable distance, then rotates to face the spatially nearest
neighbouring tag defined in mission_config.yaml.

Phases
------
1. NAVIGATE_TO_STAGING  — WaypointController drives freely to a staging point
                          on the approach axis (STAGING_DIST_M from the tag).
                          Completes when position AND heading (tag + 180°) are
                          within tolerance.
2. STRAIGHT_APPROACH    — heading locked at (tag_heading + π); only lx/ly are
                          driven.  Lateral strafe corrects any axis offset
                          accumulated during staging; forward drive closes the
                          remaining distance to the stop point.
3. FACE_NEXT_TAG        — pure yaw rotation to face the spatially nearest
                          landmark (Euclidean distance) from mission_config.yaml.
4. DONE                 — motors stopped, final pose printed.

Usage
-----
    conda activate 2.120
    python3 Jetson/test_waypoint_apriltag.py

Controls (OpenCV window)
------------------------
    q / Esc  — quit at any time (motors stopped cleanly)
    s        — print current EKF state to terminal
"""

import enum
import math
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import serial
import yaml

# ---------------------------------------------------------------------------
# Path setup — works regardless of CWD
# ---------------------------------------------------------------------------
_here = Path(__file__).resolve().parent
_repo_root = next(
    (p for p in [_here, _here.parent, _here.parent.parent]
     if (p / "Jetson" / "__init__.py").exists()),
    _here.parent,
)
if str(_repo_root) not in sys.path:
    sys.path.insert(0, str(_repo_root))

from Jetson.vision.apriltag_pose import localize_camera, robot_pose_from_camera
from Jetson.config import (
    TAG_WORLD_POSES, TAG_FAMILY, TAG_SIZE_M,
    KP_LIN, KP_STRAFE, KP_ANG,
    WAYPOINT_REACHED_M, HEADING_REACHED_RAD,
    COLOR_CAMERA_DEVICE,
)
from Jetson.localization.ekf_localizer import EKFLocalizer, GatingMethod
from Jetson.main.mission_controller import WaypointController
from Jetson.main import robot_controller as rc

try:
    import pupil_apriltags as apriltag
except ImportError:
    print("ERROR: pupil_apriltags not installed.  Run: pip install pupil-apriltags")
    sys.exit(1)


# ===========================================================================
# Parameters — edit these to configure the test
# ===========================================================================

TARGET_TAG_ID    = 5      # landmark ID to approach
STOP_DISTANCE_M  = 0.40   # final stop distance along the outward normal [m]
STAGING_DIST_M   = 1.0    # staging waypoint distance along the normal [m]
HEADING_HOLD_MAX = 0.25   # max yaw magnitude used to hold heading during approach

MISSION_CONFIG   = Path(__file__).parent / "mission_config.yaml"
CALIB_FILE       = Path(__file__).parent / "vision/camera_calibration_live.npz"
CONTROL_HZ       = 40.0   # main loop rate [Hz]

FRAME_W   = 640
FRAME_H   = 360
FRAME_FPS = 15

# Tag 1 (table_tray) position in the mission_config.yaml absolute arena frame.
# Used to convert initial_state coords into the TAG_WORLD_POSES frame.
_TAG1_MC_X = 3.57
_TAG1_MC_Y = 2.565


# ===========================================================================
# FSM states
# ===========================================================================

class State(enum.Enum):
    NAVIGATE_TO_STAGING = "NAVIGATE_TO_STAGING"
    STRAIGHT_APPROACH   = "STRAIGHT_APPROACH"
    FACE_NEXT_TAG       = "FACE_NEXT_TAG"
    DONE                = "DONE"


# ===========================================================================
# Helpers
# ===========================================================================

def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _load_mission_config(path: Path) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def _find_nearest_landmark(target_id: int, landmarks: list) -> dict:
    """Return the landmark nearest to target_id by Euclidean world distance."""
    target = next(lm for lm in landmarks if lm["id"] == target_id)
    tx = target["position"]["x"]
    ty = target["position"]["y"]
    return min(
        (lm for lm in landmarks if lm["id"] != target_id),
        key=lambda lm: math.hypot(
            lm["position"]["x"] - tx,
            lm["position"]["y"] - ty,
        ),
    )


def _tag_geometry(tag_id: int, stop_dist: float, staging_dist: float):
    """
    Compute approach-axis geometry for a given tag in the TAG_WORLD_POSES frame.

    The outward normal (alpha) is extracted from column 2 of the tag rotation
    matrix, which wall_tag_rotation() sets to the tag +Z (facing) direction.

    Returns
    -------
    tag_x, tag_y  : tag world position [m]
    alpha         : outward normal angle [rad]
    stage_x/y     : staging waypoint
    stop_x/y      : stop waypoint
    """
    pose  = TAG_WORLD_POSES[tag_id]
    tag_x = float(pose["position"][0])
    tag_y = float(pose["position"][1])

    R     = pose["rotation"]
    alpha = math.atan2(float(R[1, 2]), float(R[0, 2]))  # atan2(sin φ, cos φ)

    stage_x = tag_x + staging_dist * math.cos(alpha)
    stage_y = tag_y + staging_dist * math.sin(alpha)
    stop_x  = tag_x + stop_dist    * math.cos(alpha)
    stop_y  = tag_y + stop_dist    * math.sin(alpha)

    return tag_x, tag_y, alpha, stage_x, stage_y, stop_x, stop_y


def _tangential_offset(rx: float, ry: float,
                       tag_x: float, tag_y: float,
                       alpha: float) -> float:
    """Signed lateral distance from robot to the approach axis [m]."""
    tx = -math.sin(alpha)
    ty =  math.cos(alpha)
    return (rx - tag_x) * tx + (ry - tag_y) * ty


# ===========================================================================
# AprilTag worker (daemon thread)
# ===========================================================================

def _apriltag_worker(
    cap: cv2.VideoCapture,
    detector,
    K: np.ndarray,
    dist_coeffs: np.ndarray,
    shared: dict,
    lock: threading.Lock,
) -> None:
    """
    Continuously captures frames, detects AprilTags, and updates shared state.

    shared["result"] = (robot_x, robot_y, robot_yaw, n_tags) | None
    shared["frame"]  = latest annotated BGR frame | None
    """
    camera_params = (K[0, 0], K[1, 1], K[0, 2], K[1, 2])

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.01)
            continue

        frame = cv2.undistort(frame, K, dist_coeffs)
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        dets  = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=list(camera_params),
            tag_size=TAG_SIZE_M,
        )

        for d in dets:
            pts = d.corners.astype(int)
            cv2.polylines(frame, [pts.reshape(-1, 1, 2)], True, (0, 220, 0), 2)
            cx = int(pts[:, 0].mean())
            cy = int(pts[:, 1].mean())
            cv2.putText(frame, f"id={d.tag_id}", (cx - 20, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 0), 2, cv2.LINE_AA)

        cam_pos, R_wc, n_used = localize_camera(dets, TAG_WORLD_POSES)
        result = None
        if cam_pos is not None:
            rx, ry, ryaw = robot_pose_from_camera(cam_pos, R_wc)
            result = (rx, ry, ryaw, n_used)

        with lock:
            shared["result"] = result
            shared["frame"]  = frame


# ===========================================================================
# Display helpers
# ===========================================================================

def _draw_hud(frame: np.ndarray, state: State,
              x: float, y: float, theta: float,
              dist_to_tag: float, next_name: str, next_id: int) -> None:
    font  = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.55
    thick = 1
    lh    = 22
    pad   = 8

    def put(text: str, row: int, col: int = 8,
            color=(255, 255, 255)) -> None:
        cv2.putText(frame, text, (col, pad + row * lh),
                    font, scale, (0, 0, 0), thick + 2, cv2.LINE_AA)
        cv2.putText(frame, text, (col, pad + row * lh),
                    font, scale, color, thick, cv2.LINE_AA)

    put(f"State : {state.value}", 0, color=(255, 220, 80))
    put(f"x={x:+.3f}  y={y:+.3f}  yaw={math.degrees(theta):+.1f} deg", 1)
    put(f"dist to target tag = {dist_to_tag:.3f} m", 2)
    put(f"next tag : {next_name} (id={next_id})", 3, color=(100, 255, 200))


def _print_status(state: State, x: float, y: float, theta: float,
                  dist_to_tag: float, stop_x: float, stop_y: float,
                  tang_err: Optional[float] = None,
                  hdg_err: Optional[float] = None,
                  next_name: str = "") -> None:
    dist_to_stop = math.hypot(stop_x - x, stop_y - y)
    line = (f"[{state.value:<22s}]  "
            f"pose=({x:+.3f},{y:+.3f},{math.degrees(theta):+.1f}°)  "
            f"d_tag={dist_to_tag:.3f}m  d_stop={dist_to_stop:.3f}m")
    if tang_err is not None:
        line += f"  tang={tang_err:+.3f}m"
    if hdg_err is not None:
        line += f"  hdg_err={math.degrees(hdg_err):+.1f}°  →{next_name}"
    print(line, end="\r")


# ===========================================================================
# Drive helper
# ===========================================================================

def _send(lx: float, ly: float, yaw: float) -> None:
    if rc._drive_ser is not None:
        rc.send_drive(lx, ly, yaw)


# ===========================================================================
# Main
# ===========================================================================

def main() -> None:
    # ---------------------------------------------------------------- Config
    if not MISSION_CONFIG.exists():
        print(f"ERROR: mission config not found: {MISSION_CONFIG}")
        sys.exit(1)
    cfg       = _load_mission_config(MISSION_CONFIG)
    landmarks = cfg["map"]["landmarks"]

    # --- Seed pose (mission_config frame → TAG_WORLD_POSES frame) ----------
    init_mc    = cfg["initial_state"]
    init_x     = init_mc["position"]["x"] - _TAG1_MC_X
    init_y     = init_mc["position"]["y"] - _TAG1_MC_Y
    init_theta = float(init_mc["heading"])

    # --- Approach geometry -------------------------------------------------
    tag_x, tag_y, alpha, stage_x, stage_y, stop_x, stop_y = \
        _tag_geometry(TARGET_TAG_ID, STOP_DISTANCE_M, STAGING_DIST_M)
    approach_heading = _wrap_pi(alpha + math.pi)  # robot faces the tag

    # --- Nearest neighbour -------------------------------------------------
    next_lm     = _find_nearest_landmark(TARGET_TAG_ID, landmarks)
    next_tag_id = next_lm["id"]
    next_pose   = TAG_WORLD_POSES[next_tag_id]
    next_x      = float(next_pose["position"][0])
    next_y      = float(next_pose["position"][1])
    target_name = next(lm["name"] for lm in landmarks if lm["id"] == TARGET_TAG_ID)
    next_name   = next_lm["name"]

    print(f"Target tag  : {target_name} (id={TARGET_TAG_ID})")
    print(f"  world pos : ({tag_x:.3f}, {tag_y:.3f})  "
          f"normal={math.degrees(alpha):.1f}°  "
          f"approach_heading={math.degrees(approach_heading):.1f}°")
    print(f"  staging   : ({stage_x:.3f}, {stage_y:.3f})")
    print(f"  stop      : ({stop_x:.3f}, {stop_y:.3f})")
    print(f"Next tag    : {next_name} (id={next_tag_id})")
    print(f"  world pos : ({next_x:.3f}, {next_y:.3f})")
    print()

    # ---------------------------------------------------------------- Camera
    if not CALIB_FILE.exists():
        print(f"ERROR: calibration file not found: {CALIB_FILE}")
        sys.exit(1)
    cal    = np.load(CALIB_FILE)
    K      = cal["camera_matrix"]
    dist_c = cal["dist_coeffs"]

    cap = cv2.VideoCapture(COLOR_CAMERA_DEVICE, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_FPS, FRAME_FPS)
    if not cap.isOpened():
        print(f"ERROR: could not open camera {COLOR_CAMERA_DEVICE}")
        sys.exit(1)

    detector = apriltag.Detector(
        families=TAG_FAMILY,
        nthreads=2,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0,
    )

    # --------------------------------------------------------------- Serial
    try:
        drive_ser = serial.Serial(rc.DRIVE_PORT, rc.BAUD_RATE, timeout=1)
        time.sleep(2)
        drive_ser.reset_input_buffer()
        rc._drive_ser = drive_ser
        threading.Thread(
            target=rc._serial_reader, args=(drive_ser, "drive"), daemon=True
        ).start()
        print(f"Drive serial open: {rc.DRIVE_PORT}")
    except serial.SerialException as e:
        print(f"[WARN] Drive serial unavailable: {e}")
        print("[WARN] Drive commands printed only — no motion.")

    # ------------------------------------------------------------------ EKF
    ekf = EKFLocalizer(init_x, init_y, init_theta, gating=GatingMethod.MAHALANOBIS)
    print(f"EKF seeded : x={init_x:.3f}  y={init_y:.3f}  "
          f"theta={math.degrees(init_theta):.1f}°")
    print()

    # ----------------------------------------- AprilTag worker daemon thread
    shared = {"result": None, "frame": None}
    lock   = threading.Lock()
    threading.Thread(
        target=_apriltag_worker,
        args=(cap, detector, K, dist_c, shared, lock),
        daemon=True,
    ).start()

    # --------------------------------------------------- Waypoint controller
    # Only the staging waypoint is given to WaypointController.
    # The straight-approach phase uses its own locked-heading controller.
    wpc = WaypointController([
        {"x": stage_x, "y": stage_y,
         "theta": approach_heading, "label": "staging"},
    ])

    # ------------------------------------------------------------------- FSM
    state     = State.NAVIGATE_TO_STAGING
    last_t    = time.monotonic()
    dt_target = 1.0 / CONTROL_HZ

    print("Press q/Esc to quit.  Press s to print EKF state.")
    print("-" * 60)

    while state != State.DONE:
        t0 = time.monotonic()
        dt = max(t0 - last_t, 1e-6)
        last_t = t0

        # ---- EKF predict --------------------------------------------------
        enc = rc.get_enc()
        ekf.predict([enc["fl"], enc["bl"], enc["fr"], enc["br"]], dt)

        # ---- EKF update: IMU ----------------------------------------------
        imu = rc.get_imu("drive")
        ekf.update_imu(imu["yaw"])

        # ---- EKF update: AprilTag -----------------------------------------
        with lock:
            tag_result = shared["result"]
            frame      = shared["frame"]

        if tag_result is not None:
            rx, ry, ryaw, n_tags = tag_result
            ekf.update_apriltag(rx, ry, ryaw, n_tags=n_tags)

        x, y, theta, _ = ekf.get_pose()
        dist_to_tag    = math.hypot(tag_x - x, tag_y - y)

        # ==== State machine ================================================

        if state == State.NAVIGATE_TO_STAGING:
            lx, ly, yaw = wpc.tick(x, y, theta)
            _send(lx, ly, yaw)

            if wpc.mission_complete:
                _send(0.0, 0.0, 0.0)
                print(f"\n[FSM] {State.NAVIGATE_TO_STAGING.value} → "
                      f"{State.STRAIGHT_APPROACH.value}")
                state = State.STRAIGHT_APPROACH

            _print_status(state, x, y, theta, dist_to_tag, stop_x, stop_y)

        elif state == State.STRAIGHT_APPROACH:
            dx   = stop_x - x
            dy   = stop_y - y
            dist = math.hypot(dx, dy)

            if dist < WAYPOINT_REACHED_M:
                _send(0.0, 0.0, 0.0)
                print(f"\n[FSM] {State.STRAIGHT_APPROACH.value} → "
                      f"{State.FACE_NEXT_TAG.value}")
                state = State.FACE_NEXT_TAG
            else:
                # Heading toward stop point relative to locked approach heading.
                # cos term → forward; sin term → strafe (corrects lateral offset).
                desired_hdg = math.atan2(dy, dx)
                axis_err    = _wrap_pi(desired_hdg - approach_heading)
                ly  = float(np.clip(KP_LIN    * dist * math.cos(axis_err), -1.0, 1.0))
                lx  = float(np.clip(KP_STRAFE * dist * math.sin(axis_err), -1.0, 1.0))
                # Heading hold: small correction to resist drift, clamped tightly
                hdg_hold = _wrap_pi(approach_heading - theta)
                yaw = float(np.clip(KP_ANG * hdg_hold,
                                    -HEADING_HOLD_MAX, HEADING_HOLD_MAX))
                _send(lx, ly, yaw)

            tang = _tangential_offset(x, y, tag_x, tag_y, alpha)
            _print_status(state, x, y, theta, dist_to_tag, stop_x, stop_y,
                          tang_err=tang)

        elif state == State.FACE_NEXT_TAG:
            desired_hdg = math.atan2(next_y - y, next_x - x)
            hdg_err     = _wrap_pi(desired_hdg - theta)

            if abs(hdg_err) < HEADING_REACHED_RAD:
                _send(0.0, 0.0, 0.0)
                print(f"\n[FSM] {State.FACE_NEXT_TAG.value} → {State.DONE.value}")
                state = State.DONE
            else:
                yaw = float(np.clip(KP_ANG * hdg_err, -1.0, 1.0))
                _send(0.0, 0.0, yaw)

            _print_status(state, x, y, theta, dist_to_tag, stop_x, stop_y,
                          hdg_err=hdg_err, next_name=next_name)

        # ---- OpenCV display -----------------------------------------------
        if frame is not None:
            _draw_hud(frame, state, x, y, theta, dist_to_tag,
                      next_name, next_tag_id)
            cv2.imshow("Waypoint AprilTag Test  (q=quit  s=state)", frame)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            print("\n[USER] Quit.")
            break
        elif key == ord('s'):
            ex, ey, et, P = ekf.get_pose()
            print(f"\n[EKF] x={ex:.4f}  y={ey:.4f}  "
                  f"yaw={math.degrees(et):.2f}°  P_diag={np.diag(P)}")

        # ---- Rate limit ---------------------------------------------------
        elapsed = time.monotonic() - t0
        if elapsed < dt_target:
            time.sleep(dt_target - elapsed)

    # ---------------------------------------------------------------- Shutdown
    _send(0.0, 0.0, 0.0)
    cap.release()
    cv2.destroyAllWindows()

    x, y, theta, _ = ekf.get_pose()
    print()
    print("=" * 60)
    print(f"Final pose : x={x:.3f}  y={y:.3f}  theta={math.degrees(theta):.1f}°")
    print(f"Facing     : {next_name} (id={next_tag_id})")
    print("=" * 60)


if __name__ == "__main__":
    main()
