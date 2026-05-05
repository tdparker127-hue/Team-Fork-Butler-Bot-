"""
test_localization.py — Standalone AprilTag localization viewer.

Run this while physically moving the robot (or camera) around your arena to
validate the coordinate system, tag world poses, and EKF fusion BEFORE
running the full autonomous mission.

Usage (from repo root, with conda env active):
    conda activate 2.120
    python3 Jetson/test_localization.py

Controls (OpenCV window):
    q / Esc  — quit
    r        — reset EKF pose to last raw tag estimate
    s        — print current EKF state to terminal

What it shows:
  - Live camera frame with detected AprilTag outlines and IDs
  - Top-left overlay: raw tag estimate (x, y, yaw) for each visible tag
  - Top-right overlay: EKF-fused pose
  - Terminal: one-line updates at the detection rate

The EKF here has NO predict step (no wheel data) so it simply fuses the
AprilTag measurements over time.  Covariance will shrink as more observations
arrive — that's correct; this is a "vision-only" smoothing test.
"""

import math
import sys
import threading
import time
from pathlib import Path

import serial

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Path setup — find the repo root by locating the Jetson package marker
# Works regardless of CWD or how deep the script is invoked from.
# ---------------------------------------------------------------------------
_here = Path(__file__).resolve().parent
_repo_root = next(
    (p for p in [_here, _here.parent, _here.parent.parent]
     if (p / "Jetson" / "__init__.py").exists()),
    _here.parent,  # fallback
)
if str(_repo_root) not in sys.path:
    sys.path.insert(0, str(_repo_root))

from Jetson.vision.apriltag_pose import (
    localize_camera,
    robot_pose_from_camera,
)
from Jetson.config import (
    TAG_WORLD_POSES, TAG_FAMILY, TAG_SIZE_M,
    SIGMA_TAG_XY, SIGMA_TAG_YAW,
    COLOR_CAMERA_DEVICE, DEPTH_CAMERA_DEVICE,
)
from Jetson.localization.ekf_localizer import EKFLocalizer, GatingMethod
from Jetson.main import robot_controller as rc
import enum

try:
    import pygame
    _pygame_available = True
except ImportError:
    _pygame_available = False


class TestMode(enum.Enum):
    """
    Controls which EKF inputs are active during the test run.

    VISION_ONLY  — update_apriltag() only; no predict (original behaviour).
    IMU_ONLY     — update_imu(yaw) only; useful to check IMU drift alone.
    ENCODER_ONLY — predict() only; pure dead-reckoning with no corrections.
    FULL         — predict() + update_imu() + update_apriltag(); full fusion.
    """
    VISION_ONLY  = "vision_only"
    IMU_ONLY     = "imu_only"
    ENCODER_ONLY = "encoder_only"
    FULL         = "full"

try:
    import pupil_apriltags as apriltag
except ImportError:
    print("ERROR: pupil_apriltags not installed.  Run: pip install pupil-apriltags")
    sys.exit(1)

# ---------------------------------------------------------------------------
# Configuration — edit these to match your setup
# ---------------------------------------------------------------------------
CAMERA_DEVICE  = DEPTH_CAMERA_DEVICE   # imported from Jetson/config.py
CALIB_FILE     = Path(__file__).parent / "vision/camera_calibration_live.npz"

FRAME_W = 640
FRAME_H =  360
FRAME_FPS = 15

DETECT_NTHREADS    = 2
DETECT_QUAD_DEC    = 1.0   # 1.0 = full resolution; 2.0 = faster but less accurate
DETECT_REFINE      = 1

# AprilTag detector noise params — imported from Jetson/config.py
# SIGMA_TAG_XY, SIGMA_TAG_YAW

# EKF gating — switch to GatingMethod.EUCLIDEAN if you prefer fixed thresholds
GATING = GatingMethod.MAHALANOBIS

# ---- Test mode — change this one line to switch what the EKF fuses ----
TEST_MODE = TestMode.FULL
#   Options:
#     TestMode.VISION_ONLY  — update_apriltag() only; no predict (original behaviour).
#     TestMode.IMU_ONLY     — update_imu(yaw) only; useful to check IMU drift alone.
#     TestMode.ENCODER_ONLY — predict() only; pure dead-reckoning with no corrections.
#     TestMode.FULL         — predict() + update_imu() + update_apriltag(); full fusion.

DRIVE_PORT = "/dev/ttyACM0"   # drive ESP32 serial port
BAUD_RATE  = 115200

# Set True to also poll an Xbox controller and send drive commands.
# When True, do NOT run robot_controller.py simultaneously — they would
# fight over the serial port and cause the disconnect error.
ENABLE_DRIVE = True
# ---------------------------------------------------------------------------


# ---------------------------------------------------------------------------
# Serial telemetry is owned by robot_controller (rc) imported above.
# rc.get_imu("drive") and rc.get_enc() are used throughout.
# ---------------------------------------------------------------------------


def _rad2deg(r: float) -> float:
    return math.degrees(r)


def _draw_overlay(frame, raw_result, ekf_pose, fps: float, enc_data: dict, imu_data: dict) -> None:
    """Draw pose text onto the frame (in-place)."""
    h, w = frame.shape[:2]
    font       = cv2.FONT_HERSHEY_SIMPLEX
    scale      = 0.55
    thickness  = 1
    line_h     = 22
    pad        = 8

    def put(img, text, row, col, color=(255, 255, 255)):
        cv2.putText(img, text, (col, pad + row * line_h),
                    font, scale, (0, 0, 0), thickness + 2, cv2.LINE_AA)
        cv2.putText(img, text, (col, pad + row * line_h),
                    font, scale, color,     thickness,     cv2.LINE_AA)

    def put_b(img, text, row_from_bottom, col, color=(255, 255, 255)):
        """Draw text anchored from the bottom edge of the frame."""
        y = h - pad - row_from_bottom * line_h
        cv2.putText(img, text, (col, y), font, scale, (0, 0, 0), thickness + 2, cv2.LINE_AA)
        cv2.putText(img, text, (col, y), font, scale, color, thickness, cv2.LINE_AA)

    # ---- left column: raw tag estimate ----
    put(frame, "-- Raw tag estimate --", 0, 8, (180, 255, 180))
    if raw_result is not None:
        rx, ry, ryaw, n = raw_result
        put(frame, f"x   = {rx:+.3f} m",          1, 8)
        put(frame, f"y   = {ry:+.3f} m",          2, 8)
        put(frame, f"yaw = {_rad2deg(ryaw):+.1f} deg", 3, 8)
        put(frame, f"tags: {n}",                   4, 8)
    else:
        put(frame, "No tags in view", 1, 8, (80, 80, 255))

    # ---- right column: EKF pose + FPS ----
    col_r = w // 2
    ex, ey, etheta = ekf_pose
    put(frame, "-- EKF fused pose --",              0, col_r, (255, 220, 100))
    put(frame, f"x   = {ex:+.3f} m",               1, col_r)
    put(frame, f"y   = {ey:+.3f} m",               2, col_r)
    put(frame, f"yaw = {_rad2deg(etheta):+.1f} deg", 3, col_r)
    put(frame, f"FPS: {fps:.1f}",                   4, col_r, (200, 200, 200))

    # ---- bottom-left: IMU (drive ESP) — Euler angles in deg, rates in deg/s ----
    put_b(frame, "-- IMU (drive) --",                                      6, 8, (100, 220, 255))
    put_b(frame, f"roll      = {_rad2deg(imu_data['roll']):+.1f} deg",    5, 8)
    put_b(frame, f"pitch     = {_rad2deg(imu_data['pitch']):+.1f} deg",   4, 8)
    put_b(frame, f"yaw       = {_rad2deg(imu_data['yaw']):+.1f} deg",     3, 8)
    put_b(frame, f"roll rate = {_rad2deg(imu_data['rollRate']):+.1f} d/s",  2, 8)
    put_b(frame, f"pitch rate= {_rad2deg(imu_data['pitchRate']):+.1f} d/s", 1, 8)
    put_b(frame, f"yaw rate  = {_rad2deg(imu_data['yawRate']):+.1f} d/s",   0, 8)

    # ---- bottom-right: wheel encoder velocities (rad/s) ----
    col_enc = w - 195
    put_b(frame, "-- Encoders (rad/s) --",      4, col_enc, (255, 200, 100))
    put_b(frame, f"FL = {enc_data['fl']:+.2f}", 3, col_enc)
    put_b(frame, f"BL = {enc_data['bl']:+.2f}", 2, col_enc)
    put_b(frame, f"FR = {enc_data['fr']:+.2f}", 1, col_enc)
    put_b(frame, f"BR = {enc_data['br']:+.2f}", 0, col_enc)


def _draw_tag_boxes(frame, detections, camera_params) -> None:
    """Draw tag outlines and IDs on the frame."""
    fx, fy, cx, cy = camera_params
    for det in detections:
        pts = det.corners.astype(int)
        cv2.polylines(frame, [pts.reshape(-1, 1, 2)], True, (0, 220, 0), 2)
        cx_tag = int(pts[:, 0].mean())
        cy_tag = int(pts[:, 1].mean())
        cv2.putText(frame, f"id={det.tag_id}", (cx_tag - 20, cy_tag),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 0), 2, cv2.LINE_AA)

# NOTE: encoder data is display-only here.  This test is vision-only; ekf.predict()
# is never called.  To enable odometry fusion, call ekf.predict(enc_data, dt) in
# the main loop using timestamps to compute dt.
def main():
    # ------------------------------------------------------------------ Setup
    # Load calibration
    if not CALIB_FILE.exists():
        print(f"ERROR: calibration file not found: {CALIB_FILE}")
        sys.exit(1)
    cal = np.load(CALIB_FILE)
    K   = cal["camera_matrix"]
    dist = cal["dist_coeffs"]
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    camera_params = (fx, fy, cx, cy)

    print(f"Camera matrix:\n{K}")
    print(f"Dist coeffs: {dist}")

    # Open camera
    cap = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_FPS, FRAME_FPS)
    if not cap.isOpened():
        print(f"ERROR: could not open camera {CAMERA_DEVICE}")
        sys.exit(1)
    print(f"Camera opened: {CAMERA_DEVICE}")

    # Open drive serial — owned exclusively by this process.
    # Do NOT also run robot_controller.py; two processes on one port causes
    # the "device reports readiness" disconnect error.
    try:
        drive_ser = serial.Serial(DRIVE_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)           # allow ESP32 to reboot after DTR assertion
        drive_ser.reset_input_buffer()
        rc._drive_ser = drive_ser
        threading.Thread(
            target=rc._serial_reader, args=(drive_ser, "drive"), daemon=True
        ).start()
        print(f"Drive serial open: {DRIVE_PORT}  (encoder + IMU overlay active)")
    except serial.SerialException as e:
        print(f"[WARN] Drive serial unavailable ({DRIVE_PORT}): {e}")
        print("[WARN] Encoder/IMU overlay will show zeros.")

    # Optional joystick drive control
    joystick = None
    if ENABLE_DRIVE and _pygame_available:
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            print(f"Controller: {joystick.get_name()}  (drive enabled)")
        else:
            print("[WARN] ENABLE_DRIVE=True but no joystick detected.")
    elif ENABLE_DRIVE:
        print("[WARN] ENABLE_DRIVE=True but pygame not installed.")

    # AprilTag detector
    detector = apriltag.Detector(
        families=TAG_FAMILY,
        nthreads=DETECT_NTHREADS,
        quad_decimate=DETECT_QUAD_DEC,
        quad_sigma=0.0,
        refine_edges=DETECT_REFINE,
        decode_sharpening=0.25,
        debug=0,
    )

    # EKF — starts at (0, 0, 0); vision-only (no predict)
    ekf = EKFLocalizer(0.0, 0.0, 0.0, gating=GATING)
    print(f"EKF gating: {GATING.value}")
    print()
    print("Known tag IDs:", sorted(TAG_WORLD_POSES.keys()))
    print()
    print("Press 'q' or Esc to quit, 'r' to reset EKF, 's' to print state.")
    print("-" * 60)

    fps = 0.0
    t_prev = time.monotonic()
    raw_result = None

    print(f"Test mode: {TEST_MODE.value}")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] Frame read failed, retrying…")
            time.sleep(0.05)
            continue

        # ---- Undistort (optional but improves accuracy) ----
        frame = cv2.undistort(frame, K, dist)

        # ---- Flip vertical (camera mounted upside-down) ----
        frame = cv2.flip(frame, -1)

        # ---- AprilTag detection ----
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=list(camera_params),
            tag_size=TAG_SIZE_M,
        )

        # ---- Compute dt for predict step ----
        now = time.monotonic()
        dt = now - t_prev

        # ---- Joystick drive command ----
        if joystick is not None and rc._drive_ser is not None:
            pygame.event.pump()
            lx = joystick.get_axis(rc.AXIS_LX)
            ly = joystick.get_axis(rc.AXIS_LY)
            rx_joy = joystick.get_axis(rc.AXIS_RX)
            rc._drive_ser.write(rc.compute_drive_command(lx, ly, rx_joy).encode())

        # ---- IMU predict (all modes that use sensor data) ----
        # The predict step uses IMU yaw rate; encoder is an update step.
        imu = rc.get_imu("drive")
        if TEST_MODE == TestMode.VISION_ONLY:
            # Inject process noise without IMU-driven heading propagation
            ekf.predict(0.0, dt)
        elif TEST_MODE in (TestMode.ENCODER_ONLY, TestMode.IMU_ONLY, TestMode.FULL):
            ekf.predict(imu["yawRate"], dt)

        # ---- Encoder update (ENCODER_ONLY or FULL) ----
        if TEST_MODE in (TestMode.ENCODER_ONLY, TestMode.FULL):
            enc = rc.get_enc()
            ekf.update_encoder([enc["fl"], enc["bl"], enc["fr"], enc["br"]])

        # ---- IMU absolute-yaw update (IMU_ONLY or FULL) ----
        if TEST_MODE in (TestMode.IMU_ONLY, TestMode.FULL):
            ekf.update_imu(imu["yaw"])

        # ---- Localize ----
        cam_pos, R_wc, n_used = localize_camera(detections, TAG_WORLD_POSES)
        if cam_pos is not None:
            rx, ry, ryaw = robot_pose_from_camera(cam_pos, R_wc)
            raw_result = (rx, ry, ryaw, n_used)

            # ---- AprilTag update (VISION_ONLY or FULL) ----
            if TEST_MODE in (TestMode.VISION_ONLY, TestMode.FULL):
                ekf.update_apriltag(rx, ry, ryaw, n_tags=n_used)

            print(f"[{TEST_MODE.value.upper()}]  x={rx:+.3f}  y={ry:+.3f}  "
                  f"yaw={_rad2deg(ryaw):+.1f} deg  "
                  f"({n_used} tag{'s' if n_used != 1 else ''})   "
                  f"FPS={fps:.1f}", end="\r")
        else:
            raw_result = None

        # ---- Draw ----
        ex, ey, etheta, _ = ekf.get_pose()
        enc_data = rc.get_enc()
        imu_data = rc.get_imu("drive")
        _draw_tag_boxes(frame, detections, camera_params)
        _draw_overlay(frame, raw_result, (ex, ey, etheta), fps, enc_data, imu_data)
        # mode label — bottom centre
        cv2.putText(frame, f"mode: {TEST_MODE.value}",
                    (frame.shape[1] // 2 - 80, frame.shape[0] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, f"mode: {TEST_MODE.value}",
                    (frame.shape[1] // 2 - 80, frame.shape[0] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 180, 255), 1, cv2.LINE_AA)

        cv2.imshow("Localization Test  (q=quit  r=reset EKF  s=print state)", frame)

        # ---- FPS ----
        fps = 0.9 * fps + 0.1 * (1.0 / max(dt, 1e-6))
        t_prev = now

        # ---- Key handling ----
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):    # q or Esc
            break
        elif key == ord('r'):
            if raw_result is not None:
                ekf.set_pose(raw_result[0], raw_result[1], raw_result[2])
                print(f"\n[EKF] Reset to raw estimate: "
                      f"x={raw_result[0]:.3f}  y={raw_result[1]:.3f}  "
                      f"yaw={_rad2deg(raw_result[2]):.1f} deg")
            else:
                print("\n[EKF] No tag visible — reset skipped.")
        elif key == ord('s'):
            ex, ey, etheta, P = ekf.get_pose()
            vx_b, vy_b, om, P_vel = ekf.get_twist()
            print(f"\n[EKF] x={ex:.4f}  y={ey:.4f}  yaw={_rad2deg(etheta):.2f} deg")
            print(f"      vx_b={vx_b:.3f}  vy_b={vy_b:.3f}  omega={om:.3f} rad/s")
            print(f"      P_pose diag: {np.diag(P)}")
            print(f"      P_vel  diag: {np.diag(P_vel)}")

    cap.release()
    cv2.destroyAllWindows()
    print("\nDone.")


if __name__ == "__main__":
    main()
