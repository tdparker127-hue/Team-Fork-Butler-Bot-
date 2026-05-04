"""
tray_detection.py — Bright-tray detection pipeline with approach command
generation for autonomous pickup alignment.

Extends the existing color-blob detection in object_detection.py with:
  - High-saturation preprocessing to robustly detect brightly-colored trays
  - RealSense depth map integration for 3-D distance estimation
  - Centering error (pixel offset from image centre) for visual servoing
  - Approach command generator: returns normalized (lx, ly, yaw) drive
    commands to center the robot on the tray and close to pickup distance

Usage:
    from Jetson.vision.tray_detection import TrayDetector

    detector = TrayDetector()
    candidates = detector.detect(color_bgr, depth_mm)
    if candidates:
        lx, ly, yaw = detector.get_approach_command(candidates[0])
        # send to robot drive
"""

import sys
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np

# Re-use calibration and blob utilities from the existing object_detection module
_vision_dir = Path(__file__).parent
if str(_vision_dir) not in sys.path:
    sys.path.insert(0, str(_vision_dir))

from object_detection import (
    load_calibration,
    load_color_ranges,
    load_table_calibration,
    undistort_frame,
    find_blobs,
    pixel_to_world,
    CALIB_FILE,
    MIN_BLOB_AREA,
)

_repo_root = Path(__file__).resolve().parent.parent.parent
if str(_repo_root) not in sys.path:
    sys.path.insert(0, str(_repo_root))

from Jetson.config import (
    BRIGHT_TRAY_LOWER_HSV, BRIGHT_TRAY_UPPER_HSV,
    TRAY_LABELS, TRAY_MIN_AREA,
    TARGET_PICKUP_DIST_MM, CENTERING_DEAD_ZONE_PX,
    KP_APPROACH_LATERAL, KP_APPROACH_YAW, KP_APPROACH_FORWARD,
    MAX_APPROACH_LX, MAX_APPROACH_LY, MAX_APPROACH_YAW,
)

# ===========================================================================
# Tunable parameters — all imported from Jetson/config.py
# ===========================================================================

# --- Depth sampling ---------------------------------------------------------
DEPTH_PATCH_HALF = 5    # half-size of patch around centroid
DEPTH_MAX_MM     = 4000

# --- Fallback depth from calibration height ---------------------------------
# Used when no depth camera is attached.  Match CAM_HEIGHT_MM from
# object_detection.py or your physical mount.
FALLBACK_DIST_MM = 800.0


# ===========================================================================
# Data type
# ===========================================================================

@dataclass
class TrayCandidate:
    """A single detected tray candidate."""
    label:            str
    centroid_px:      Tuple[int, int]       # (u, v) image pixel
    world_xy_mm:      Optional[Tuple[float, float]]  # (X, Y) table-frame mm
    dist_mm:          float                 # estimated distance from camera [mm]
    centering_err_px: int                   # centroid_u - image_center_u; pos = tray right of center
    bbox:             Tuple[int, int, int, int]  # (x, y, w, h)
    confidence:       float = 1.0           # area-based proxy confidence [0,1]


# ===========================================================================
# TrayDetector
# ===========================================================================

class TrayDetector:
    """
    Detects brightly-colored trays in a color image and optionally uses a
    RealSense depth map to compute 3-D distance.

    Parameters
    ----------
    calib_file : Path-like, optional
        Path to camera calibration .npz.  Defaults to the shared
        camera_calibration_live.npz in the vision folder.
    undistort : bool
        Whether to apply lens undistortion before detection.
    """

    def __init__(self, calib_file=CALIB_FILE, undistort: bool = True):
        self._camera_matrix, self._dist_coeffs = load_calibration(
            Path(calib_file)
        )
        self._do_undistort = undistort and (self._camera_matrix is not None)
        self._color_ranges = load_color_ranges()
        self._homography = load_table_calibration()

    # -----------------------------------------------------------------------
    # Main detection
    # -----------------------------------------------------------------------

    def detect(
        self,
        color_bgr: np.ndarray,
        depth_mm:  Optional[np.ndarray] = None,
    ) -> List[TrayCandidate]:
        """
        Detect trays in a single color frame.

        Parameters
        ----------
        color_bgr : HxWx3 uint8 BGR image
        depth_mm  : HxW uint16 (or float32) depth in mm, aligned to color.
                    Pass None to use a fallback distance estimate.

        Returns
        -------
        List of TrayCandidate, sorted by confidence (largest blob first).
        """
        if self._do_undistort:
            frame = undistort_frame(color_bgr, self._camera_matrix,
                                    self._dist_coeffs)
        else:
            frame = color_bgr

        frame_h, frame_w = frame.shape[:2]
        img_cx = frame_w // 2

        candidates: List[TrayCandidate] = []

        # --- Channel 1: labelled tray colors from color_ranges.json ---------
        for label, (lower, upper, roi) in self._color_ranges.items():
            if label not in TRAY_LABELS:
                continue
            blobs = find_blobs(frame, label, lower, upper, roi)
            for b in blobs:
                if b["area"] < TRAY_MIN_AREA:
                    continue
                cand = self._build_candidate(
                    label, b, img_cx, frame_h, frame_w, depth_mm
                )
                candidates.append(cand)

        # --- Channel 2: generic high-saturation bright-tray mask -------------
        bright_blobs = self._find_bright_tray_blobs(frame)
        for b in bright_blobs:
            # Avoid duplicating detections already found via labeled channel
            if not self._overlaps_existing(b["centroid"], candidates):
                cand = self._build_candidate(
                    "tray_bright", b, img_cx, frame_h, frame_w, depth_mm
                )
                candidates.append(cand)

        # Sort largest (highest confidence) first
        candidates.sort(key=lambda c: c.confidence, reverse=True)
        return candidates

    # -----------------------------------------------------------------------
    # Approach command generation
    # -----------------------------------------------------------------------

    def get_approach_command(
        self,
        candidate: TrayCandidate,
        target_dist_mm: float = TARGET_PICKUP_DIST_MM,
    ) -> Tuple[float, float, float]:
        """
        Generate normalized drive commands (lx, ly, yaw) to approach a tray.

        Strategy:
          - lx  (strafe): proportional to lateral pixel error from centre
          - yaw (turn):   also proportional to pixel error (combined with lx
                          so the robot rotates to face the tray as it strafes)
          - ly  (forward): proportional to (dist_mm - target_dist_mm)
            positive ly = move forward, negative = back up

        Returns
        -------
        (lx, ly, yaw) each in [-1, 1]
        """
        err_px  = candidate.centering_err_px          # positive = tray right
        dist_err_mm = candidate.dist_mm - target_dist_mm  # positive = too far

        lx  = float(np.clip( KP_APPROACH_LATERAL * err_px,
                             -MAX_APPROACH_LX,  MAX_APPROACH_LX))
        yaw = float(np.clip( KP_APPROACH_YAW    * err_px,
                             -MAX_APPROACH_YAW, MAX_APPROACH_YAW))
        ly  = float(np.clip( KP_APPROACH_FORWARD * dist_err_mm,
                             -MAX_APPROACH_LY,  MAX_APPROACH_LY))

        # Dead-zone: if already centred, remove lateral commands
        if abs(err_px) < CENTERING_DEAD_ZONE_PX:
            lx  = 0.0
            yaw = 0.0

        return lx, ly, yaw

    def is_centered_and_close(
        self,
        candidate: TrayCandidate,
        target_dist_mm: float = TARGET_PICKUP_DIST_MM,
        dist_tol_mm: float = 60.0,
    ) -> bool:
        """
        Returns True when the tray is both centred in the image and within
        distance tolerance for pickup.
        """
        return (abs(candidate.centering_err_px) < CENTERING_DEAD_ZONE_PX and
                abs(candidate.dist_mm - target_dist_mm) < dist_tol_mm)

    # -----------------------------------------------------------------------
    # Annotate
    # -----------------------------------------------------------------------

    @staticmethod
    def annotate(frame: np.ndarray, candidates: List[TrayCandidate]) -> np.ndarray:
        """Draw bounding boxes and metadata onto a copy of the frame."""
        out = frame.copy()
        h, w = out.shape[:2]
        cx = w // 2

        # Draw image-centre line
        cv2.line(out, (cx, 0), (cx, h), (255, 255, 0), 1)

        for c in candidates:
            bx, by, bw, bh = c.bbox
            cv2.rectangle(out, (bx, by), (bx + bw, by + bh), (0, 180, 255), 2)

            u, v = c.centroid_px
            cv2.circle(out, (u, v), 5, (0, 255, 255), -1)

            label = (f"{c.label}  err={c.centering_err_px:+d}px  "
                     f"d={c.dist_mm:.0f}mm")
            cv2.putText(out, label, (bx, max(by - 6, 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 180, 255), 2)

        return out

    # -----------------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------------

    def _build_candidate(self, label, blob, img_cx, frame_h, frame_w,
                         depth_mm) -> TrayCandidate:
        u, v   = blob["centroid"]
        area   = blob["area"]
        bbox   = blob["bbox"]

        # Distance estimate
        dist_mm = self._sample_depth(depth_mm, u, v, frame_h, frame_w)

        # World position (requires homography calibration)
        world_xy = None
        if self._homography is not None:
            result = pixel_to_world(u, v, self._camera_matrix, self._homography)
            if result is not None:
                world_xy = (result[0], result[1])

        # Area-based confidence (normalised 0–1 relative to TRAY_MIN_AREA)
        max_area = frame_h * frame_w * 0.5
        conf = min(1.0, (area - TRAY_MIN_AREA) / max(1, max_area - TRAY_MIN_AREA))

        return TrayCandidate(
            label=label,
            centroid_px=(u, v),
            world_xy_mm=world_xy,
            dist_mm=dist_mm,
            centering_err_px=u - img_cx,
            bbox=bbox,
            confidence=conf,
        )

    @staticmethod
    def _find_bright_tray_blobs(frame: np.ndarray) -> list:
        """
        Detect highly-saturated blobs in the frame as generic bright trays.

        Applies a saturation boost preprocessing step before thresholding to
        make faint-but-colorful trays stand out more robustly.
        """
        # Boost saturation by converting to HSV, scaling S channel, back to BGR
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV).astype(np.float32)
        hsv[:, :, 1] = np.clip(hsv[:, :, 1] * 1.3, 0, 255)
        boosted_hsv = hsv.astype(np.uint8)

        mask = cv2.inRange(boosted_hsv, BRIGHT_TRAY_LOWER_HSV, BRIGHT_TRAY_UPPER_HSV)

        # Morphological cleanup
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        blobs = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < TRAY_MIN_AREA:
                continue
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            bx, by, bw, bh = cv2.boundingRect(cnt)
            blobs.append({
                "centroid": (cx, cy),
                "area":     area,
                "bbox":     (bx, by, bw, bh),
                "contour":  cnt,
            })

        # Return only the largest blob to avoid clutter
        if blobs:
            blobs.sort(key=lambda b: b["area"], reverse=True)
            return blobs[:3]
        return []

    @staticmethod
    def _overlaps_existing(centroid, existing: List[TrayCandidate],
                           threshold_px: int = 50) -> bool:
        """Check if a centroid is within threshold_px of any existing candidate."""
        cx, cy = centroid
        for e in existing:
            eu, ev = e.centroid_px
            if math.hypot(cx - eu, cy - ev) < threshold_px:
                return True
        return False

    @staticmethod
    def _sample_depth(depth_mm, u, v, frame_h, frame_w) -> float:
        """Return depth in mm at (u, v) from a patch median, or fallback."""
        if depth_mm is None:
            return FALLBACK_DIST_MM

        p = DEPTH_PATCH_HALF
        r0 = max(0, v - p);  r1 = min(frame_h, v + p + 1)
        c0 = max(0, u - p);  c1 = min(frame_w, u + p + 1)
        patch = depth_mm[r0:r1, c0:c1].astype(np.float32)
        valid = patch[(patch > 0) & (patch < DEPTH_MAX_MM)]
        if valid.size == 0:
            return FALLBACK_DIST_MM
        return float(np.median(valid))


# ===========================================================================
# Standalone test
# ===========================================================================

if __name__ == "__main__":
    import sys

    detector = TrayDetector(undistort=False)

    if len(sys.argv) > 1:
        img = cv2.imread(sys.argv[1])
        if img is None:
            print(f"Could not read image: {sys.argv[1]}")
            sys.exit(1)
        candidates = detector.detect(img)
        print(f"Found {len(candidates)} tray candidate(s):")
        for c in candidates:
            print(f"  {c.label}  centroid={c.centroid_px}  "
                  f"dist={c.dist_mm:.0f}mm  err={c.centering_err_px:+d}px")
            lx, ly, yaw = detector.get_approach_command(c)
            print(f"    approach cmd: lx={lx:+.3f}  ly={ly:+.3f}  yaw={yaw:+.3f}")
        annotated = TrayDetector.annotate(img, candidates)
        cv2.imshow("tray_detection", annotated)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        cap = cv2.VideoCapture(48)
        print("Press q to quit.")
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            cands = detector.detect(frame)
            out   = TrayDetector.annotate(frame, cands)
            cv2.imshow("tray_detection", out)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()
