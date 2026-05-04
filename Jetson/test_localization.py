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
import time
from pathlib import Path

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Path setup — allow running from the repo root
# ---------------------------------------------------------------------------
_repo_root = Path(__file__).resolve().parent
if str(_repo_root) not in sys.path:
    sys.path.insert(0, str(_repo_root))

from Jetson.vision.apriltag_pose import (
    localize_camera,
    robot_pose_from_camera,
)
from Jetson.config import (
    TAG_WORLD_POSES, TAG_FAMILY, TAG_SIZE_M,
    SIGMA_TAG_XY, SIGMA_TAG_YAW,
    COLOR_CAMERA_DEVICE,
)
from Jetson.localization.ekf_localizer import EKFLocalizer, GatingMethod

try:
    import pupil_apriltags as apriltag
except ImportError:
    print("ERROR: pupil_apriltags not installed.  Run: pip install pupil-apriltags")
    sys.exit(1)

# ---------------------------------------------------------------------------
# Configuration — edit these to match your setup
# ---------------------------------------------------------------------------
CAMERA_DEVICE  = COLOR_CAMERA_DEVICE   # imported from Jetson/config.py
CALIB_FILE     = Path(__file__).parent / "vision/camera_calibration_live.npz"

FRAME_W = 1280
FRAME_H =  720
FRAME_FPS = 15

DETECT_NTHREADS    = 2
DETECT_QUAD_DEC    = 1.0   # 1.0 = full resolution; 2.0 = faster but less accurate
DETECT_REFINE      = 1

# AprilTag detector noise params — imported from Jetson/config.py
# SIGMA_TAG_XY, SIGMA_TAG_YAW

# EKF gating — switch to GatingMethod.EUCLIDEAN if you prefer fixed thresholds
GATING = GatingMethod.MAHALANOBIS
# ---------------------------------------------------------------------------


def _rad2deg(r: float) -> float:
    return math.degrees(r)


def _draw_overlay(frame, raw_result, ekf_pose, fps: float) -> None:
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

    # ---- right column: EKF pose ----
    col_r = w // 2
    ex, ey, etheta = ekf_pose
    put(frame, "-- EKF fused pose --",              0, col_r, (255, 220, 100))
    put(frame, f"x   = {ex:+.3f} m",               1, col_r)
    put(frame, f"y   = {ey:+.3f} m",               2, col_r)
    put(frame, f"yaw = {_rad2deg(etheta):+.1f} deg", 3, col_r)

    # ---- bottom: FPS ----
    put(frame, f"FPS: {fps:.1f}", h // line_h - 1, 8, (200, 200, 200))


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

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] Frame read failed, retrying…")
            time.sleep(0.05)
            continue

        # ---- Undistort (optional but improves accuracy) ----
        frame = cv2.undistort(frame, K, dist)

        # ---- AprilTag detection ----
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=list(camera_params),
            tag_size=TAG_SIZE_M,
        )

        # ---- Localize ----
        cam_pos, R_wc, n_used = localize_camera(detections, TAG_WORLD_POSES)
        if cam_pos is not None:
            rx, ry, ryaw = robot_pose_from_camera(cam_pos, R_wc)
            raw_result = (rx, ry, ryaw, n_used)

            # Update EKF (no predict since no wheel data in this test)
            ekf.update_apriltag(rx, ry, ryaw, n_tags=n_used)

            print(f"[RAW]  x={rx:+.3f}  y={ry:+.3f}  "
                  f"yaw={_rad2deg(ryaw):+.1f} deg  "
                  f"({n_used} tag{'s' if n_used != 1 else ''})   "
                  f"FPS={fps:.1f}", end="\r")
        else:
            raw_result = None

        # ---- Draw ----
        ex, ey, etheta, _ = ekf.get_pose()
        _draw_tag_boxes(frame, detections, camera_params)
        _draw_overlay(frame, raw_result, (ex, ey, etheta), fps)

        cv2.imshow("Localization Test  (q=quit  r=reset EKF  s=print state)", frame)

        # ---- FPS ----
        now = time.monotonic()
        fps = 0.9 * fps + 0.1 * (1.0 / max(now - t_prev, 1e-6))
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
            print(f"\n[EKF] x={ex:.4f}  y={ey:.4f}  yaw={_rad2deg(etheta):.2f} deg")
            print(f"      P diag: {np.diag(P)}")

    cap.release()
    cv2.destroyAllWindows()
    print("\nDone.")


if __name__ == "__main__":
    main()
