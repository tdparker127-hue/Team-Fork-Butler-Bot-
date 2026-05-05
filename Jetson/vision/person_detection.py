"""
person_detection.py — YOLOv8-based person detection with depth-augmented
threat assessment for robot safety stops.

The detector classifies each frame into one of three threat levels:
    PersonThreat.CLEAR — no person detected above confidence threshold
    PersonThreat.SLOW  — person detected but far away (reduce speed)
    PersonThreat.STOP  — person detected close (full stop)

Distance is estimated from:
  1. RealSense depth map (5×5 patch at bbox centre) — primary
  2. Bounding-box area fraction of the frame             — fallback

All decision thresholds are tunable constants at the top of this file.

Usage:
    from Jetson.vision.person_detection import PersonDetector, PersonThreat

    detector = PersonDetector()
    threat, detections, annotated_frame = detector.detect(color_bgr, depth_mm)
    if threat >= PersonThreat.SLOW:
        # reduce speed or stop
"""

import enum
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np

try:
    from ultralytics import YOLO
    _YOLO_AVAILABLE = True
except ImportError:
    _YOLO_AVAILABLE = False
    print("[WARN] ultralytics not installed — PersonDetector will return CLEAR always.")

try:
    from Jetson.config import (
        PERSON_CONF, SLOW_DIST_M, STOP_DIST_M, SLOW_BBOX_FRAC, STOP_BBOX_FRAC,
    )
except ModuleNotFoundError:
    from config import (
        PERSON_CONF, SLOW_DIST_M, STOP_DIST_M, SLOW_BBOX_FRAC, STOP_BBOX_FRAC,
    )

# ===========================================================================
# Tunable parameters — imported from Jetson/config.py
# ===========================================================================

# --- Model ------------------------------------------------------------------
YOLO_MODEL       = "yolov8n.pt"   # nano model — fast enough on Jetson
YOLO_DEVICE      = "cpu"          # "cuda:0" if GPU available on Jetson

# --- Detection confidence, distances, bbox fractions -----------------------
# PERSON_CONF, SLOW_DIST_M, STOP_DIST_M, SLOW_BBOX_FRAC, STOP_BBOX_FRAC

# --- Depth sampling ---------------------------------------------------------
DEPTH_PATCH_HALF = 3              # half-size of the patch around bbox centre
DEPTH_MAX_MM     = 6000           # ignore depth readings beyond this (noise)


# ===========================================================================
# Data types
# ===========================================================================

class PersonThreat(enum.IntEnum):
    """Ordered threat levels — higher = more dangerous."""
    CLEAR = 0
    SLOW  = 1
    STOP  = 2


@dataclass
class PersonDetection:
    """A single detected person."""
    bbox_xyxy: Tuple[int, int, int, int]   # (x1, y1, x2, y2) pixel coords
    confidence: float
    dist_m: Optional[float] = None         # None if depth unavailable
    threat: PersonThreat = PersonThreat.CLEAR

    @property
    def bbox_area_px(self) -> int:
        x1, y1, x2, y2 = self.bbox_xyxy
        return max(0, (x2 - x1) * (y2 - y1))


# ===========================================================================
# PersonDetector
# ===========================================================================

class PersonDetector:
    """
    Wraps a YOLOv8 model and adds depth-based threat classification.

    Parameters
    ----------
    model_path : str
        Path to (or name of) the YOLOv8 model weights file.
        If just a filename like "yolov8n.pt" is given, ultralytics will
        download it automatically on first use.
    conf_threshold : float
        Override the default PERSON_CONF constant.
    """

    def __init__(self, model_path: str = YOLO_MODEL,
                 conf_threshold: float = PERSON_CONF):
        self.conf_threshold = conf_threshold
        self._model = None
        if _YOLO_AVAILABLE:
            self._model = YOLO(model_path)
            # Warm up so the first real inference isn't slow
            self._model.predict(
                np.zeros((320, 320, 3), dtype=np.uint8),
                classes=[0], conf=conf_threshold,
                device=YOLO_DEVICE, verbose=False
            )

    # -----------------------------------------------------------------------
    # Main detection method
    # -----------------------------------------------------------------------

    def detect(
        self,
        color_bgr: np.ndarray,
        depth_mm:  Optional[np.ndarray] = None,
    ) -> Tuple[PersonThreat, List[PersonDetection], np.ndarray]:
        """
        Run person detection on a single frame.

        Parameters
        ----------
        color_bgr : HxWx3 uint8 BGR image
        depth_mm  : HxW uint16 (or float32) depth in millimetres,
                    aligned to color_bgr.  Pass None to use bbox fallback.

        Returns
        -------
        threat         : PersonThreat — highest threat among all detections
        detections     : list of PersonDetection
        annotated_frame: color_bgr with bounding boxes and threat overlay
        """
        if self._model is None:
            return PersonThreat.CLEAR, [], color_bgr.copy()

        frame_h, frame_w = color_bgr.shape[:2]
        frame_area = max(1, frame_h * frame_w)

        # --- Run YOLO -------------------------------------------------------
        results = self._model.predict(
            color_bgr,
            classes=[0],             # class 0 = person in COCO
            conf=self.conf_threshold,
            device=YOLO_DEVICE,
            verbose=False,
        )

        detections: List[PersonDetection] = []

        if results and results[0].boxes is not None:
            for box in results[0].boxes:
                conf = float(box.conf[0])
                if conf < self.conf_threshold:
                    continue

                x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].tolist())
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2

                # --- Estimate distance -------------------------------------
                dist_m = self._sample_depth(depth_mm, cx, cy, frame_h, frame_w)

                # --- Classify threat level ---------------------------------
                if dist_m is not None:
                    if dist_m < STOP_DIST_M:
                        threat = PersonThreat.STOP
                    elif dist_m < SLOW_DIST_M:
                        threat = PersonThreat.SLOW
                    else:
                        threat = PersonThreat.CLEAR
                else:
                    # Fallback: use bbox area fraction
                    bbox_area = max(0, (x2 - x1) * (y2 - y1))
                    frac = bbox_area / frame_area
                    if frac >= STOP_BBOX_FRAC:
                        threat = PersonThreat.STOP
                    elif frac >= SLOW_BBOX_FRAC:
                        threat = PersonThreat.SLOW
                    else:
                        threat = PersonThreat.CLEAR

                detections.append(PersonDetection(
                    bbox_xyxy=(x1, y1, x2, y2),
                    confidence=conf,
                    dist_m=dist_m,
                    threat=threat,
                ))

        # --- Determine overall threat level (highest among all detections) --
        overall_threat = (
            max(d.threat for d in detections)
            if detections else PersonThreat.CLEAR
        )

        # --- Annotate frame -------------------------------------------------
        annotated = self._annotate(color_bgr.copy(), detections, overall_threat)

        return overall_threat, detections, annotated

    # -----------------------------------------------------------------------
    # Internals
    # -----------------------------------------------------------------------

    @staticmethod
    def _sample_depth(
        depth_mm: Optional[np.ndarray],
        cx: int, cy: int,
        frame_h: int, frame_w: int,
    ) -> Optional[float]:
        """
        Sample a robust depth value from a patch around (cx, cy).

        Returns depth in metres, or None if depth_mm is None or all-zero
        in the sampled patch.
        """
        if depth_mm is None:
            return None

        p = DEPTH_PATCH_HALF
        r0 = max(0, cy - p);  r1 = min(frame_h, cy + p + 1)
        c0 = max(0, cx - p);  c1 = min(frame_w, cx + p + 1)
        patch = depth_mm[r0:r1, c0:c1].astype(np.float32)

        valid = patch[(patch > 0) & (patch < DEPTH_MAX_MM)]
        if valid.size == 0:
            return None

        # Use the median to reject edge noise
        return float(np.median(valid)) / 1000.0

    @staticmethod
    def _annotate(
        frame: np.ndarray,
        detections: List[PersonDetection],
        overall_threat: PersonThreat,
    ) -> np.ndarray:
        """Draw bounding boxes and a threat-level HUD on the frame."""
        THREAT_COLORS = {
            PersonThreat.CLEAR: (0, 220, 0),     # green
            PersonThreat.SLOW:  (0, 165, 255),   # orange
            PersonThreat.STOP:  (0, 0, 220),     # red
        }

        for d in detections:
            x1, y1, x2, y2 = d.bbox_xyxy
            color = THREAT_COLORS[d.threat]
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

            label = f"person {d.confidence:.2f}"
            if d.dist_m is not None:
                label += f"  {d.dist_m:.2f}m"
            cv2.putText(frame, label, (x1, max(y1 - 6, 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

        # Overall threat banner at the bottom
        h, w = frame.shape[:2]
        threat_names = {
            PersonThreat.CLEAR: "PERSON: CLEAR",
            PersonThreat.SLOW:  "PERSON: SLOW",
            PersonThreat.STOP:  "PERSON: STOP",
        }
        banner_color = THREAT_COLORS[overall_threat]
        cv2.rectangle(frame, (0, h - 36), (w, h), (30, 30, 30), -1)
        cv2.putText(frame, threat_names[overall_threat], (10, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, banner_color, 2)

        return frame


# ===========================================================================
# Standalone test
# ===========================================================================

if __name__ == "__main__":
    import sys

    detector = PersonDetector()

    if len(sys.argv) > 1:
        img = cv2.imread(sys.argv[1])
        if img is None:
            print(f"Could not read image: {sys.argv[1]}")
            sys.exit(1)
        threat, dets, annotated = detector.detect(img)
        print(f"Threat: {threat.name}  |  detections: {len(dets)}")
        for d in dets:
            print(f"  bbox={d.bbox_xyxy}  conf={d.confidence:.2f}  "
                  f"dist={d.dist_m}  threat={d.threat.name}")
        cv2.imshow("person_detection", annotated)
        cv2.waitKey(0)
    else:
        # Live webcam demo
        cap = cv2.VideoCapture(48)
        print("Press q to quit.")
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            threat, _, annotated = detector.detect(frame)
            cv2.imshow("person_detection", annotated)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cap.release()
    cv2.destroyAllWindows()
