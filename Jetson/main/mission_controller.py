"""
mission_controller.py — Autonomous mission FSM for the Butler Bot.

Entry point for autonomous operation.  Replaces robot_controller.py's manual
teleop loop with a finite-state machine that can:
  - Follow operator-defined waypoints (WAYPOINT_NAV)
  - Approach and pick up trays autonomously (TRAY_APPROACH / TRAY_PICKUP)
  - Place trays at drop waypoints (TRAY_PLACE)
  - Immediately yield to person-safety stop (PERSON_STOP)
  - Fall back to full manual teleop (TELEOP) at any time

Gamepad mapping (Xbox One BT layout, same indices as robot_controller.py):
  Start    → WAYPOINT_NAV (resume/start mission)
  Back     → TELEOP       (manual override, any state)
  Y button → TRAY_APPROACH (interrupt waypoint mode to approach nearest tray)
  A button → confirm / skip current tray
  B button → abort current sub-task, back to WAYPOINT_NAV

Run:
    python3 -m Jetson.main.mission_controller
or:
    python3 Jetson/main/mission_controller.py
"""

import enum
import math
import sys
import threading
import time
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np
import pygame
import serial

# ---------------------------------------------------------------------------
# Path setup — allow running from the repo root or from inside Jetson/
# ---------------------------------------------------------------------------
_repo_root = Path(__file__).resolve().parent.parent.parent
if str(_repo_root) not in sys.path:
    sys.path.insert(0, str(_repo_root))

from Jetson.main.robot_controller import (
    _serial_reader,
    get_imu,
    get_enc,
    send_drive,
    send_arm,
    step_arm_setpoints,
    compute_drive_command,
    _scale,
    DRIVE_PORT, ARM_PORT, BAUD_RATE,
    MIN_LIFT_RAD, MAX_LIFT_RAD, MIN_GRIP_RAD, MAX_GRIP_RAD,
    MAX_LIFT_SPEED, MAX_GRIP_SPEED,
    AXIS_LX, AXIS_LY, AXIS_RX, AXIS_LT, AXIS_RT,
    BTN_LB, BTN_RB, DEADBAND,
)
from Jetson.localization.ekf_localizer import EKFLocalizer
from Jetson.vision.person_detection import PersonDetector, PersonThreat
from Jetson.vision.tray_detection import TrayDetector, TrayCandidate
from Jetson.vision.apriltag_pose import (
    localize_camera,
    robot_pose_from_camera,
    TAG_WORLD_POSES,
)

try:
    import pupil_apriltags as apriltag
    _APRILTAG_AVAILABLE = True
except ImportError:
    _APRILTAG_AVAILABLE = False
    print("[WARN] pupil_apriltags not available — AprilTag updates disabled.")

try:
    import pyrealsense2 as rs
    _RS_AVAILABLE = True
except ImportError:
    _RS_AVAILABLE = False

from Jetson.config import (
    TAG_SIZE_M,
    KP_LIN, KP_STRAFE, KP_ANG, WAYPOINT_REACHED_M, HEADING_REACHED_RAD,
    ARM_LOWER_LIFT, ARM_GRIP_CLOSE, ARM_CARRY_LIFT, ARM_GRIP_OPEN,
    ARM_LOWER_TIME_S, ARM_GRIP_TIME_S, ARM_LIFT_TIME_S,
    COLOR_CAMERA_DEVICE,
)
from Jetson.map_viewer import MapViewer

# ===========================================================================
# Tunable parameters
# ===========================================================================

# --- Camera device paths — imported from Jetson/config.py -----------------
# COLOR_CAMERA_DEVICE: RGB camera for AprilTags
DEPTH_CAMERA_SERIAL  = None             # None = auto-detect first RealSense

# --- AprilTag detection rate -----------------------------------------------
APRILTAG_HZ      = 5      # camera localization update rate
# TAG_SIZE_M imported from Jetson/config.py

# --- Person detection rate -------------------------------------------------
PERSON_DETECT_HZ = 10

# --- Main control loop rate ------------------------------------------------
CONTROL_HZ       = 20

# --- Waypoint controller gains — imported from Jetson/config.py ------------
# KP_LIN, KP_STRAFE, KP_ANG, WAYPOINT_REACHED_M, HEADING_REACHED_RAD

# --- Arm pickup sequence — imported from Jetson/config.py -----------------
# ARM_LOWER_LIFT, ARM_GRIP_CLOSE, ARM_CARRY_LIFT, ARM_GRIP_OPEN
# ARM_LOWER_TIME_S, ARM_GRIP_TIME_S, ARM_LIFT_TIME_S

# --- Gamepad button indices (must match robot_controller.py) ---------------
BTN_START   = 11   # Start  → WAYPOINT_NAV
BTN_BACK    = 10   # Back   → TELEOP
BTN_Y       = 3    # Y      → TRAY_APPROACH
BTN_A       = 0    # A      → confirm / advance
BTN_B       = 1    # B      → abort subtask


# ===========================================================================
# FSM state enum
# ===========================================================================

class FSMState(enum.Enum):
    IDLE          = "IDLE"
    TELEOP        = "TELEOP"
    WAYPOINT_NAV  = "WAYPOINT_NAV"
    TRAY_APPROACH = "TRAY_APPROACH"
    TRAY_PICKUP   = "TRAY_PICKUP"
    TRAY_PLACE    = "TRAY_PLACE"
    PERSON_STOP   = "PERSON_STOP"


# ===========================================================================
# Waypoint controller
# ===========================================================================

class WaypointController:
    """
    PD waypoint navigator for the mecanum robot.

    Each waypoint is a dict with keys:
        x, y         : world-frame position [m]  (required)
        theta        : desired final heading [rad] (optional)
        label        : human-readable name        (optional)

    Call tick() every control step to get normalized drive commands.
    """

    def __init__(self, waypoints: Optional[List[dict]] = None):
        self._waypoints: List[dict] = waypoints or []
        self._index: int = 0

    def load_mission(self, waypoints: List[dict]) -> None:
        self._waypoints = list(waypoints)
        self._index = 0

    @property
    def current_waypoint(self) -> Optional[dict]:
        if self._index < len(self._waypoints):
            return self._waypoints[self._index]
        return None

    @property
    def mission_complete(self) -> bool:
        return self._index >= len(self._waypoints)

    def advance(self) -> None:
        """Manually advance to the next waypoint."""
        self._index += 1

    def tick(self, pose_x: float, pose_y: float, pose_theta: float
             ) -> Tuple[float, float, float]:
        """
        Compute drive command (lx, ly, yaw) toward the current waypoint.

        Returns (0, 0, 0) when all waypoints are complete.
        """
        wp = self.current_waypoint
        if wp is None:
            return 0.0, 0.0, 0.0

        dx   = wp["x"] - pose_x
        dy   = wp["y"] - pose_y
        dist = math.hypot(dx, dy)

        if dist < WAYPOINT_REACHED_M:
            # Check final heading if specified
            theta_target = wp.get("theta", None)
            if theta_target is not None:
                heading_err = _wrap_pi(theta_target - pose_theta)
                if abs(heading_err) > HEADING_REACHED_RAD:
                    yaw = float(np.clip(KP_ANG * heading_err, -1.0, 1.0))
                    return 0.0, 0.0, yaw
            self._index += 1
            return 0.0, 0.0, 0.0

        # Desired heading toward waypoint
        desired_heading = math.atan2(dy, dx)
        heading_err     = _wrap_pi(desired_heading - pose_theta)

        ly  = float(np.clip(KP_LIN    * dist * math.cos(heading_err), -1.0, 1.0))
        lx  = float(np.clip(KP_STRAFE * dist * math.sin(heading_err), -1.0, 1.0))
        yaw = float(np.clip(KP_ANG    * heading_err,                  -1.0, 1.0))

        return lx, ly, yaw


def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


# ===========================================================================
# Camera worker threads
# ===========================================================================

class _ApriltagWorker(threading.Thread):
    """Continuously captures frames and runs AprilTag localization."""

    def __init__(self, device, camera_matrix, dist_coeffs):
        super().__init__(daemon=True)
        self._device = device
        self._camera_matrix = camera_matrix
        self._dist_coeffs   = dist_coeffs
        self._lock          = threading.Lock()
        self._result        = None   # (robot_x, robot_y, robot_yaw, n_tags)

    @property
    def latest(self):
        with self._lock:
            return self._result

    def run(self):
        if not _APRILTAG_AVAILABLE or self._camera_matrix is None:
            return

        detector = apriltag.Detector(
            families='tag36h11', nthreads=2,
            quad_decimate=1.0, quad_sigma=0.0,
            refine_edges=1, decode_sharpening=0.25, debug=0,
        )
        fx = self._camera_matrix[0, 0]
        fy = self._camera_matrix[1, 1]
        cx = self._camera_matrix[0, 2]
        cy = self._camera_matrix[1, 2]

        cap = cv2.VideoCapture(self._device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y','U','Y','V'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  720)
        cap.set(cv2.CAP_PROP_FPS, 15)

        interval = 1.0 / APRILTAG_HZ

        while True:
            t0 = time.monotonic()
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.1)
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            results = detector.detect(
                gray, estimate_tag_pose=True,
                camera_params=[fx, fy, cx, cy],
                tag_size=TAG_SIZE_M,
            )

            cam_pos, R_wc, n_used = localize_camera(results, TAG_WORLD_POSES)
            if cam_pos is not None:
                rx, ry, ryaw = robot_pose_from_camera(cam_pos, R_wc)
                with self._lock:
                    self._result = (rx, ry, ryaw, n_used)

            elapsed = time.monotonic() - t0
            sleep_t = interval - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)


class _PersonWorker(threading.Thread):
    """Continuously captures frames and runs person detection."""

    def __init__(self, detector: PersonDetector, device_or_rs):
        super().__init__(daemon=True)
        self._detector  = detector
        self._source    = device_or_rs
        self._lock      = threading.Lock()
        self._threat     = PersonThreat.CLEAR
        self._latest_frame: Optional[np.ndarray] = None

    @property
    def threat(self) -> PersonThreat:
        with self._lock:
            return self._threat

    @property
    def latest_frame(self) -> Optional[np.ndarray]:
        with self._lock:
            return self._latest_frame

    def run(self):
        interval = 1.0 / PERSON_DETECT_HZ

        if _RS_AVAILABLE and isinstance(self._source, str) and "realsense" in self._source.lower():
            self._run_realsense(interval)
        else:
            self._run_webcam(interval)

    def _run_webcam(self, interval):
        device = self._source if isinstance(self._source, (str, int)) else 0
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)

        while True:
            t0 = time.monotonic()
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.1)
                continue

            threat, _, annotated = self._detector.detect(frame)
            with self._lock:
                self._threat       = threat
                self._latest_frame = annotated

            elapsed = time.monotonic() - t0
            sleep_t = interval - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    def _run_realsense(self, interval):
        pipeline = rs.pipeline()
        config   = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,  30)
        pipeline.start(config)
        align = rs.align(rs.stream.color)

        try:
            while True:
                t0 = time.monotonic()
                frames       = pipeline.wait_for_frames(timeout_ms=500)
                aligned      = align.process(frames)
                color_frame  = aligned.get_color_frame()
                depth_frame  = aligned.get_depth_frame()

                if not color_frame:
                    continue

                color_bgr = np.asanyarray(color_frame.get_data())
                depth_mm  = None
                if depth_frame:
                    depth_mm = (
                        np.asanyarray(depth_frame.get_data()).astype(np.float32)
                        * depth_frame.get_units() * 1000
                    ).astype(np.uint16)

                threat, _, annotated = self._detector.detect(color_bgr, depth_mm)
                with self._lock:
                    self._threat       = threat
                    self._latest_frame = annotated

                elapsed = time.monotonic() - t0
                sleep_t = interval - elapsed
                if sleep_t > 0:
                    time.sleep(sleep_t)
        finally:
            pipeline.stop()


# ===========================================================================
# MissionController
# ===========================================================================

class MissionController:
    """
    Top-level autonomous mission controller.

    Owns the EKF, waypoint controller, vision workers, and FSM.
    Call run() to start the control loop (blocking).
    """

    def __init__(self):
        self._ekf  = EKFLocalizer()
        self._nav  = WaypointController()
        self._person_detector = PersonDetector()
        self._tray_detector   = TrayDetector()

        # FSM state
        self._state:     FSMState           = FSMState.IDLE
        self._prev_state: FSMState          = FSMState.IDLE

        # Arm setpoints (Jetson owns integration, same as robot_controller.py)
        self._lift_sp = 0.0
        self._grip_sp = 0.0

        # For TRAY_APPROACH: best candidate from last tray detection
        self._tray_target: Optional[TrayCandidate] = None
        # Latest tray frame from the person-detection camera (reused for trays)
        self._tray_frame: Optional[np.ndarray]  = None
        self._tray_depth: Optional[np.ndarray]  = None
        self._tray_lock  = threading.Lock()

        # For TRAY_PICKUP / TRAY_PLACE: arm sequence state machine
        self._arm_seq_step   = 0
        self._arm_seq_t0     = 0.0

        # Serial handles (set in run())
        self._drive_ser: Optional[serial.Serial] = None
        self._arm_ser:   Optional[serial.Serial] = None

        # Vision workers (started in run())
        self._apriltag_worker: Optional[_ApriltagWorker] = None
        self._person_worker:   Optional[_PersonWorker]   = None

        # Camera calibration (loaded once)
        self._camera_matrix = None
        self._dist_coeffs   = None

        # Last EKF predict timestamp
        self._last_predict_t = time.monotonic()

    # -----------------------------------------------------------------------
    # Public API
    # -----------------------------------------------------------------------

    def load_mission(self, waypoints: List[dict]) -> None:
        """Load a waypoint list and reset the navigator index."""
        self._nav.load_mission(waypoints)

    def transition(self, new_state: FSMState) -> None:
        """Transition to a new FSM state."""
        if new_state != self._state:
            print(f"[FSM] {self._state.value} → {new_state.value}")
            self._prev_state = self._state
            self._state      = new_state
            self._on_state_enter(new_state)

    def run(self) -> None:
        """
        Open serial ports, start vision workers, and run the 20 Hz control loop.
        Blocks until Ctrl-C.
        """
        # ----- Pygame joystick -------------------------------------------
        pygame.init()
        pygame.joystick.init()
        joystick = None
        if pygame.joystick.get_count() > 0:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            print(f"[MC] Controller: {joystick.get_name()}")
        else:
            print("[MC] No joystick detected — TELEOP unavailable.")

        # ----- Serial ports -----------------------------------------------
        try:
            self._drive_ser = serial.Serial(DRIVE_PORT, BAUD_RATE, timeout=1)
        except serial.SerialException as e:
            print(f"[MC] Drive port error: {e}")
            return
        try:
            self._arm_ser = serial.Serial(ARM_PORT, BAUD_RATE, timeout=1)
        except serial.SerialException as e:
            print(f"[MC] Arm port error: {e}")
            self._drive_ser.close()
            return

        time.sleep(2)
        self._drive_ser.reset_input_buffer()
        self._arm_ser.reset_input_buffer()

        # Expose serial handles via robot_controller module globals
        import Jetson.main.robot_controller as rc
        rc._drive_ser = self._drive_ser
        rc._arm_ser   = self._arm_ser

        threading.Thread(target=_serial_reader,
                         args=(self._drive_ser, "drive"), daemon=True).start()
        threading.Thread(target=_serial_reader,
                         args=(self._arm_ser,   "arm"),   daemon=True).start()

        # ----- Camera calibration & AprilTag worker -----------------------
        from object_detection import load_calibration, CALIB_FILE
        self._camera_matrix, self._dist_coeffs = load_calibration(CALIB_FILE)

        if _APRILTAG_AVAILABLE and self._camera_matrix is not None:
            self._apriltag_worker = _ApriltagWorker(
                COLOR_CAMERA_DEVICE, self._camera_matrix, self._dist_coeffs
            )
            self._apriltag_worker.start()

        # ----- Startup localization ----------------------------------------
        # Block briefly until the AprilTag worker delivers a fix, then seed
        # the EKF so waypoint navigation starts with a real position estimate
        # rather than the default (0, 0, 0).
        self._startup_localize()

        # ----- Person detection worker (uses RealSense if available) ------
        person_source = "realsense" if _RS_AVAILABLE else 0
        self._person_worker = _PersonWorker(self._person_detector, person_source)
        self._person_worker.start()

        # ----- Tray detection: share person-worker frame ------------------
        # Tray detection runs in the main loop (not a separate thread) so it
        # doesn't need its own camera.  The person worker exposes latest_frame.

        # ----- Map viewer -------------------------------------------------
        self._map_viewer = MapViewer()
        self._map_viewer.start()

        print("[MC] Starting mission controller at 20 Hz.  Ctrl-C to stop.")
        self._last_predict_t = time.monotonic()
        dt = 1.0 / CONTROL_HZ
        try:
            while True:
                t0 = time.monotonic()
                self._step(joystick, dt)
                elapsed   = time.monotonic() - t0
                sleep_for = dt - elapsed
                if sleep_for > 0:
                    time.sleep(sleep_for)
        except KeyboardInterrupt:
            print("\n[MC] Shutting down.")
        finally:
            self._send_drive_raw(0.0, 0.0, 0.0)
            self._drive_ser.close()
            self._arm_ser.close()
            self._map_viewer.stop()
            pygame.quit()

    # -----------------------------------------------------------------------
    # Startup localization
    # -----------------------------------------------------------------------

    def _startup_localize(self,
                          timeout_s: float = 10.0,
                          n_fixes: int = 5) -> bool:
        """
        Block until we accumulate `n_fixes` consistent AprilTag pose estimates
        and seed the EKF with their average.  Returns True on success.

        If no AprilTags are visible within `timeout_s` seconds, prints a
        warning and returns False (EKF stays at the default pose (0, 0, 0)).

        The averaging rejects outliers using the same Mahalanobis gate that the
        running EKF uses, ensuring only self-consistent fixes contribute.
        """
        if self._apriltag_worker is None:
            print("[MC] Startup localize skipped — no AprilTag worker.")
            return False

        print(f"[MC] Waiting for AprilTag fix (need {n_fixes}, timeout {timeout_s}s)…")
        deadline = time.monotonic() + timeout_s
        fixes_x: list = []
        fixes_y: list = []
        fixes_yaw: list = []
        last_seen = None

        while time.monotonic() < deadline:
            result = self._apriltag_worker.latest
            if result is not None and result is not last_seen:
                last_seen = result
                rx, ry, ryaw, n_tags = result
                fixes_x.append(rx)
                fixes_y.append(ry)
                fixes_yaw.append(ryaw)
                print(f"[MC]   fix {len(fixes_x)}/{n_fixes}: "
                      f"x={rx:.3f}  y={ry:.3f}  yaw={math.degrees(ryaw):.1f}°"
                      f"  ({n_tags} tag{'s' if n_tags != 1 else ''})")
                if len(fixes_x) >= n_fixes:
                    break
            time.sleep(0.1)

        if not fixes_x:
            print("[MC] WARNING: no AprilTags detected during startup. "
                  "Localizing from (0, 0, 0).")
            return False

        # Robust average: discard fixes more than 2 std deviations from the median
        def _robust_mean(vals):
            arr = np.array(vals)
            med = np.median(arr)
            std = np.std(arr) or 1e-6
            good = arr[np.abs(arr - med) < 2 * std]
            return float(np.mean(good)) if good.size else float(med)

        x0   = _robust_mean(fixes_x)
        y0   = _robust_mean(fixes_y)
        # Yaw average via unit-vector mean to handle wrap-around
        yaw0 = float(np.arctan2(
            np.mean([math.sin(a) for a in fixes_yaw]),
            np.mean([math.cos(a) for a in fixes_yaw]),
        ))

        self._ekf.set_pose(x0, y0, yaw0)
        print(f"[MC] Startup pose set: x={x0:.3f}  y={y0:.3f}  "
              f"yaw={math.degrees(yaw0):.1f}°  (from {len(fixes_x)} fixes)")
        return True

    # -----------------------------------------------------------------------
    # Main loop step
    # -----------------------------------------------------------------------

    def _step(self, joystick, dt: float) -> None:
        # 1. EKF predict from IMU yaw rate
        imu = get_imu("drive")
        self._ekf.predict(imu["yawRate"], dt)

        # 2. EKF update: wheel encoder velocities (body-frame twist)
        enc = get_enc()
        wheel_vels = [enc["fl"], enc["bl"], enc["fr"], enc["br"]]
        self._ekf.update_encoder(wheel_vels)

        # 3. EKF update: IMU yaw (absolute heading — no double-integrated accel)
        self._ekf.update_imu(imu["yaw"])

        # 4. EKF update: AprilTag (runs at its own rate in worker thread)
        if self._apriltag_worker is not None:
            tag_result = self._apriltag_worker.latest
            if tag_result is not None:
                rx, ry, ryaw, n_tags = tag_result
                self._ekf.update_apriltag(rx, ry, ryaw, n_tags)

        pose_x, pose_y, pose_theta, _ = self._ekf.get_pose()
        self._map_viewer.update(pose_x, pose_y, pose_theta)

        # 4. Person-safety preemption (highest priority)
        person_threat = (
            self._person_worker.threat
            if self._person_worker else PersonThreat.CLEAR
        )

        if person_threat >= PersonThreat.SLOW and self._state != FSMState.PERSON_STOP:
            self.transition(FSMState.PERSON_STOP)
        elif person_threat == PersonThreat.CLEAR and self._state == FSMState.PERSON_STOP:
            restored = self._prev_state
            if restored == FSMState.PERSON_STOP:
                restored = FSMState.IDLE
            self.transition(restored)

        # 5. Gamepad override buttons
        if joystick is not None:
            pygame.event.pump()
            n_btn = joystick.get_numbuttons()

            def _btn(idx):
                return bool(joystick.get_button(idx)) if n_btn > idx else False

            if _btn(BTN_BACK):
                self.transition(FSMState.TELEOP)
            elif _btn(BTN_START):
                self.transition(FSMState.WAYPOINT_NAV)
            elif _btn(BTN_Y) and self._state not in (
                    FSMState.TRAY_APPROACH, FSMState.TRAY_PICKUP,
                    FSMState.TRAY_PLACE, FSMState.PERSON_STOP):
                self.transition(FSMState.TRAY_APPROACH)
            elif _btn(BTN_B) and self._state in (
                    FSMState.TRAY_APPROACH, FSMState.TRAY_PICKUP, FSMState.TRAY_PLACE):
                self.transition(FSMState.WAYPOINT_NAV)

        # 6. FSM dispatch → compute (lx, ly, yaw) drive command
        lx, ly, yaw = self._dispatch(
            joystick, pose_x, pose_y, pose_theta, dt
        )

        # 7. Person-safety speed scaling (SLOW state)
        if person_threat == PersonThreat.SLOW:
            lx  *= 0.3
            ly  *= 0.3
            yaw *= 0.3

        # 8. Send drive command
        self._send_drive_raw(lx, ly, yaw)

    # -----------------------------------------------------------------------
    # FSM dispatch
    # -----------------------------------------------------------------------

    def _dispatch(self, joystick, px, py, ptheta, dt) -> Tuple[float, float, float]:
        state = self._state

        # -- IDLE ------------------------------------------------------------
        if state == FSMState.IDLE:
            return 0.0, 0.0, 0.0

        # -- PERSON_STOP -----------------------------------------------------
        if state == FSMState.PERSON_STOP:
            return 0.0, 0.0, 0.0

        # -- TELEOP ----------------------------------------------------------
        if state == FSMState.TELEOP:
            if joystick is None:
                return 0.0, 0.0, 0.0
            lx  = joystick.get_axis(AXIS_LX)
            ly  = joystick.get_axis(AXIS_LY)
            rx  = joystick.get_axis(AXIS_RX)
            lt  = joystick.get_axis(AXIS_LT) if joystick.get_numaxes() > AXIS_LT else -1.0
            rt  = joystick.get_axis(AXIS_RT) if joystick.get_numaxes() > AXIS_RT else -1.0
            lb  = bool(joystick.get_button(BTN_LB)) if joystick.get_numbuttons() > BTN_LB else False
            rb  = bool(joystick.get_button(BTN_RB)) if joystick.get_numbuttons() > BTN_RB else False

            self._lift_sp, self._grip_sp = step_arm_setpoints(
                self._lift_sp, self._grip_sp, lt, rt, lb, rb, dt
            )
            self._send_arm_raw(self._lift_sp, self._grip_sp)

            return _scale(lx), -_scale(ly), _scale(rx)

        # -- WAYPOINT_NAV ----------------------------------------------------
        if state == FSMState.WAYPOINT_NAV:
            if self._nav.mission_complete:
                print("[MC] Mission complete.")
                self.transition(FSMState.IDLE)
                return 0.0, 0.0, 0.0
            return self._nav.tick(px, py, ptheta)

        # -- TRAY_APPROACH ---------------------------------------------------
        if state == FSMState.TRAY_APPROACH:
            frame = (self._person_worker.latest_frame
                     if self._person_worker else None)
            if frame is None:
                return 0.0, 0.0, 0.0

            with self._tray_lock:
                depth = self._tray_depth

            candidates = self._tray_detector.detect(frame, depth)
            if not candidates:
                return 0.0, 0.0, 0.0

            best = candidates[0]
            self._tray_target = best

            if self._tray_detector.is_centered_and_close(best):
                self.transition(FSMState.TRAY_PICKUP)
                return 0.0, 0.0, 0.0

            return self._tray_detector.get_approach_command(best)

        # -- TRAY_PICKUP -----------------------------------------------------
        if state == FSMState.TRAY_PICKUP:
            done = self._run_arm_pickup_sequence()
            if done:
                self.transition(FSMState.WAYPOINT_NAV)
            return 0.0, 0.0, 0.0

        # -- TRAY_PLACE ------------------------------------------------------
        if state == FSMState.TRAY_PLACE:
            done = self._run_arm_place_sequence()
            if done:
                self._nav.advance()
                self.transition(FSMState.WAYPOINT_NAV)
            return 0.0, 0.0, 0.0

        return 0.0, 0.0, 0.0

    # -----------------------------------------------------------------------
    # Arm pickup/place sequences
    # -----------------------------------------------------------------------

    def _on_state_enter(self, state: FSMState) -> None:
        """Reset sub-state when entering a new state."""
        if state in (FSMState.TRAY_PICKUP, FSMState.TRAY_PLACE):
            self._arm_seq_step = 0
            self._arm_seq_t0   = time.monotonic()

    def _run_arm_pickup_sequence(self) -> bool:
        """
        Non-blocking arm pickup state machine.
        Returns True when sequence is complete.

        Steps:
          0 → lower lift       (wait ARM_LOWER_TIME_S)
          1 → close gripper    (wait ARM_GRIP_TIME_S)
          2 → raise lift       (wait ARM_LIFT_TIME_S)
          3 → done
        """
        now = time.monotonic()
        step = self._arm_seq_step

        if step == 0:
            self._lift_sp = ARM_LOWER_LIFT
            self._grip_sp = ARM_GRIP_OPEN
            self._send_arm_raw(self._lift_sp, self._grip_sp)
            if now - self._arm_seq_t0 > ARM_LOWER_TIME_S:
                self._arm_seq_step = 1
                self._arm_seq_t0   = now

        elif step == 1:
            self._grip_sp = ARM_GRIP_CLOSE
            self._send_arm_raw(self._lift_sp, self._grip_sp)
            if now - self._arm_seq_t0 > ARM_GRIP_TIME_S:
                self._arm_seq_step = 2
                self._arm_seq_t0   = now

        elif step == 2:
            self._lift_sp = ARM_CARRY_LIFT
            self._send_arm_raw(self._lift_sp, self._grip_sp)
            if now - self._arm_seq_t0 > ARM_LIFT_TIME_S:
                self._arm_seq_step = 3

        elif step == 3:
            return True

        return False

    def _run_arm_place_sequence(self) -> bool:
        """
        Non-blocking arm place state machine.

        Steps:
          0 → lower lift to place height   (wait ARM_LOWER_TIME_S)
          1 → open gripper                 (wait ARM_GRIP_TIME_S)
          2 → raise lift to carry height   (wait ARM_LIFT_TIME_S)
          3 → done
        """
        now  = time.monotonic()
        step = self._arm_seq_step

        if step == 0:
            self._lift_sp = ARM_LOWER_LIFT
            self._send_arm_raw(self._lift_sp, self._grip_sp)
            if now - self._arm_seq_t0 > ARM_LOWER_TIME_S:
                self._arm_seq_step = 1
                self._arm_seq_t0   = now

        elif step == 1:
            self._grip_sp = ARM_GRIP_OPEN
            self._send_arm_raw(self._lift_sp, self._grip_sp)
            if now - self._arm_seq_t0 > ARM_GRIP_TIME_S:
                self._arm_seq_step = 2
                self._arm_seq_t0   = now

        elif step == 2:
            self._lift_sp = ARM_CARRY_LIFT
            self._send_arm_raw(self._lift_sp, self._grip_sp)
            if now - self._arm_seq_t0 > ARM_LIFT_TIME_S:
                self._arm_seq_step = 3

        elif step == 3:
            return True

        return False

    # -----------------------------------------------------------------------
    # Send helpers
    # -----------------------------------------------------------------------

    def _send_drive_raw(self, lx: float, ly: float, yaw: float) -> None:
        if self._drive_ser and self._drive_ser.is_open:
            self._drive_ser.write(
                f"lx:{lx:.3f};ly:{ly:.3f};yaw:{yaw:.3f};\n".encode()
            )

    def _send_arm_raw(self, lift: float, grip: float) -> None:
        if self._arm_ser and self._arm_ser.is_open:
            self._arm_ser.write(
                f"lift:{lift:.3f};grip:{grip:.3f};\n".encode()
            )


# ===========================================================================
# Example mission definition  (edit for your arena layout)
# ===========================================================================

# ===========================================================================
# Example mission definition
# ===========================================================================
# All waypoints are in the world frame whose origin is AprilTag 1's position
# projected onto the floor.  +X = right along the tag-1 wall, +Y = depth
# into the arena.  Distances in metres.
#
# Edit TAG_WORLD_POSES in Jetson/vision/apriltag_pose.py to match your
# physical tag layout before running.
# ===========================================================================

EXAMPLE_MISSION = [
    {"x": 1.0,  "y": 1.0,  "label": "approach_table"},
    {"x": 2.0,  "y": 1.0,  "label": "tray_pickup_zone"},
    {"x": 2.0,  "y": 2.5,  "theta": math.pi / 2, "label": "drop_zone"},
]


# ===========================================================================
# Entry point
# ===========================================================================

if __name__ == "__main__":
    mc = MissionController()
    mc.load_mission(EXAMPLE_MISSION)
    # Start in TELEOP so the operator can confirm readiness before pressing Start
    mc._state = FSMState.TELEOP
    mc.run()
