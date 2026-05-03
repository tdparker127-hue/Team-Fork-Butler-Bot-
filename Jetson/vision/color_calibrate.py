"""
color_calibrate.py — Interactive HSV color-range calibration + ROI tool.

Workflow
--------
1. Click an object button (right panel) to select which object to calibrate.
2. Click multiple pixels on the object in the camera feed to accumulate
   color samples.  Each click adds one sample; the range expands to cover
   min/max across all samples ± tolerances.  Matched pixels are tinted
   green in real time.  Press U to undo the last sample.
3. Optionally click "Set ROI" then drag a rectangle on the camera feed.
   The ROI is stored per-object and used by object_detection.py to
   restrict blob search to the tape-bounded region.
4. Tune tolerances with keyboard shortcuts (shown in panel).
5. Press Enter / Shift+S or click "SAVE ALL" to write color_ranges.json.

JSON format saved
-----------------
  {
    "cup_red": {
      "lower": [H, S, V],
      "upper": [H, S, V],
      "roi":   [x, y, w, h]   <- only present if you drew an ROI
    },
    ...
  }

Key shortcuts
-------------
  H/h   Hue tolerance +/-
  A/a   Saturation tolerance +/-
  V/v   Value (brightness) tolerance +/-
  C     Toggle table-corner picking (click 4 corners for homography scale)
  D     Toggle live detection overlay
  U     Undo last sample point
  R     Reset current object (color + ROI)
  Enter / Shift+S   Save all to JSON
  Q     Quit
"""

import json
import sys
from pathlib import Path

import cv2
import numpy as np

# Import detection helper for live preview
sys.path.insert(0, str(Path(__file__).parent))
try:
    from object_detection import find_blobs as _find_blobs, pixel_to_world as _pixel_to_world
    _DETECT_AVAILABLE = True
except ImportError:
    _DETECT_AVAILABLE = False

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
CALIB_FILE  = Path(__file__).parent / "camera_calibration_live.npz"
OUTPUT_JSON = Path(__file__).parent / "color_ranges.json"
DEFAULT_DEVICE = "/dev/video52"

OBJECT_LABELS = ["cup_red", "cup_blue", "bottle", "bowl", "plate", "tray"]

LABEL_COLORS = {
    "cup_red":  (0,   0,   220),
    "cup_blue": (220, 80,   0),
    "bottle":   (0,   200,  50),
    "bowl":     (0,   180, 255),
    "plate":    (180, 180, 180),
    "tray":     (100,  60,   20),
}

PANEL_W    = 330
BTN_H      = 46
BTN_MARGIN = 7
FONT       = cv2.FONT_HERSHEY_SIMPLEX
FS         = 0.45
FM         = 0.58
FL         = 0.72

DEFAULT_TOL = {"H": 15, "S": 60, "V": 60}
TOL_STEP    = 5
SAMPLE_HALF = 4

# Table corner calibration — 4 corners clicked clockwise from near-left.
# Near edge = y closest to camera; Far edge = y furthest from camera.
# World coords match TABLE_PARAMS in object_detection.py.
CORNER_ORDER = [
    ("Near-Left",  (-455,   0)),
    ("Near-Right", ( 455,   0)),
    ("Far-Right",  ( 455, 540)),
    ("Far-Left",   (-455, 540)),
]
TABLE_CALIB_JSON = Path(__file__).parent / "table_calibration.json"


# ---------------------------------------------------------------------------
# Camera helpers
# ---------------------------------------------------------------------------

def load_calibration():
    if not CALIB_FILE.exists():
        print(f"[WARN] {CALIB_FILE} not found — skipping undistortion.")
        return None, None
    data = np.load(str(CALIB_FILE))
    print(f"[INFO] Loaded calibration from {CALIB_FILE}")
    return data["camera_matrix"], data["dist_coeffs"]


def open_camera(device=DEFAULT_DEVICE):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  720)
    cap.set(cv2.CAP_PROP_FPS, 15)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera: {device}")
        sys.exit(1)
    return cap


def undistort(frame, cam_mat, dist):
    h, w = frame.shape[:2]
    new_cam, roi = cv2.getOptimalNewCameraMatrix(cam_mat, dist, (w, h), 1, (w, h))
    out = cv2.undistort(frame, cam_mat, dist, None, new_cam)
    x, y, rw, rh = roi
    if rw > 0 and rh > 0:
        out = out[y:y+rh, x:x+rw]
    return out


# ---------------------------------------------------------------------------
# HSV sampling
# ---------------------------------------------------------------------------

def sample_hsv(frame_bgr, cx, cy):
    h, w = frame_bgr.shape[:2]
    x0 = max(0, cx - SAMPLE_HALF);  x1 = min(w, cx + SAMPLE_HALF + 1)
    y0 = max(0, cy - SAMPLE_HALF);  y1 = min(h, cy + SAMPLE_HALF + 1)
    patch = frame_bgr[y0:y1, x0:x1]
    patch_hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV).reshape(-1, 3).astype(float)
    return patch_hsv.mean(axis=0), patch_hsv.std(axis=0)


def build_range_multi(samples, tol):
    """
    Compute HSV lower/upper bounds from a list of [H,S,V] sample means.
    lower[i] = min across all samples - tol
    upper[i] = max across all samples + tol
    """
    arr      = np.array(samples, dtype=float)  # Nx3
    keys     = ["H", "S", "V"]
    lims_hi  = [179, 255, 255]
    lo = [int(np.clip(arr[:, i].min() - tol[k], 0, lims_hi[i]))
          for i, k in enumerate(keys)]
    hi = [int(np.clip(arr[:, i].max() + tol[k], 0, lims_hi[i]))
          for i, k in enumerate(keys)]
    return lo, hi


# ---------------------------------------------------------------------------
# JSON persistence
# ---------------------------------------------------------------------------

def load_existing_json():
    if OUTPUT_JSON.exists():
        with open(OUTPUT_JSON) as f:
            return json.load(f)
    return {}


def save_json(ranges: dict):
    with open(OUTPUT_JSON, "w") as f:
        json.dump(ranges, f, indent=2)
    print(f"[INFO] Saved -> {OUTPUT_JSON}")


# ---------------------------------------------------------------------------
# ROI helpers
# ---------------------------------------------------------------------------

def roi_from_drag(p1, p2):
    """Return [x, y, w, h] from two arbitrary corner points."""
    x = min(p1[0], p2[0]);  y = min(p1[1], p2[1])
    w = abs(p2[0] - p1[0]); h = abs(p2[1] - p1[1])
    return [x, y, w, h]


# ---------------------------------------------------------------------------
# Panel drawing
# ---------------------------------------------------------------------------

def draw_panel(panel, state):
    panel[:] = (30, 30, 30)
    ph = panel.shape[0]
    y = 10

    cv2.putText(panel, "HSV Color Calibration", (10, y + 18),
                FONT, FM, (220, 220, 220), 1, cv2.LINE_AA)
    y += 30
    cv2.line(panel, (0, y), (PANEL_W, y), (80, 80, 80), 1)
    y += 8

    # Object buttons — suffix shows [C] for color set, [R] for ROI set
    for label in OBJECT_LABELS:
        bclr   = LABEL_COLORS[label]
        active = (label == state["selected"])
        border = (255, 255, 255) if active else (60, 60, 60)
        thick  = 2 if active else 1
        x0, y0 = BTN_MARGIN, y
        x1, y1 = PANEL_W - BTN_MARGIN, y + BTN_H
        cv2.rectangle(panel, (x0, y0), (x1, y1), bclr, -1)
        cv2.rectangle(panel, (x0, y0), (x1, y1), border, thick)
        entry     = state["ranges"].get(label, {})
        suffix = (" [C]" if "lower" in entry else "") + (" [R]" if "roi" in entry else "")
        cv2.putText(panel, label + suffix,
                    (x0 + 8, y0 + BTN_H // 2 + 6),
                    FONT, FM, (255, 255, 255), 1, cv2.LINE_AA)
        state["buttons"][label] = (x0, y0, x1, y1)
        y += BTN_H + BTN_MARGIN

    cv2.line(panel, (0, y), (PANEL_W, y), (80, 80, 80), 1)
    y += 10

    sel   = state["selected"]
    entry = state["ranges"].get(sel, {})
    mode_txt = " [ROI MODE - drag]" if state["roi_mode"] else ""
    cv2.putText(panel, f"Active: {sel}{mode_txt}",
                (10, y + 12), FONT, FS, (220, 200, 100), 1, cv2.LINE_AA)
    y += 24

    # Color sample swatch + multi-sample info
    sel_samples = state["samples"].get(sel, [])
    n = len(sel_samples)
    if n > 0:
        mean = state["last_sample"]
        lo, hi = build_range_multi(sel_samples, state["tol"])
        swatch_bgr = cv2.cvtColor(
            np.array([[[int(mean[0]), int(mean[1]), int(mean[2])]]],
                     dtype=np.uint8),
            cv2.COLOR_HSV2BGR)[0, 0].tolist()
        cv2.rectangle(panel, (10, y), (55, y + 36), swatch_bgr, -1)
        cv2.rectangle(panel, (10, y), (55, y + 36), (160, 160, 160), 1)
        cv2.putText(panel, f"{n} sample{'s' if n != 1 else ''}  [U=undo]",
                    (62, y + 10), FONT, FS, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(panel, f"Lo [{lo[0]},{lo[1]},{lo[2]}]",
                    (62, y + 24), FONT, FS, (130, 230, 130), 1, cv2.LINE_AA)
        cv2.putText(panel, f"Hi [{hi[0]},{hi[1]},{hi[2]}]",
                    (62, y + 36), FONT, FS, (130, 230, 130), 1, cv2.LINE_AA)
        y += 50
    else:
        cv2.putText(panel, "Click frame to add color samples",
                    (10, y + 12), FONT, FS, (120, 120, 120), 1, cv2.LINE_AA)
        y += 24

    roi = entry.get("roi")
    if roi:
        cv2.putText(panel, f"ROI x={roi[0]} y={roi[1]} w={roi[2]} h={roi[3]}",
                    (10, y + 12), FONT, FS, (100, 200, 255), 1, cv2.LINE_AA)
    else:
        cv2.putText(panel, "ROI: not set  (click SET ROI to draw)",
                    (10, y + 12), FONT, FS, (100, 100, 100), 1, cv2.LINE_AA)
    y += 22

    # Corner calibration status line
    cpts   = state["corners_px"]
    n_cpts = len(cpts)
    if state["corner_mode"]:
        next_lbl = CORNER_ORDER[n_cpts][0] if n_cpts < 4 else "done"
        cv2.putText(panel, f"CORNERS [{n_cpts}/4] - click {next_lbl}",
                    (10, y + 12), FONT, FS, (0, 255, 180), 1, cv2.LINE_AA)
    elif n_cpts == 4:
        hom_txt = "homography ready" if state["corner_homography"] is not None else "computing..."
        cv2.putText(panel, f"Corners: 4/4  {hom_txt}",
                    (10, y + 12), FONT, FS, (0, 255, 180), 1, cv2.LINE_AA)
    elif n_cpts > 0:
        cv2.putText(panel, f"Corners: {n_cpts}/4  (resume with CORNERS)",
                    (10, y + 12), FONT, FS, (100, 200, 150), 1, cv2.LINE_AA)
    else:
        cv2.putText(panel, "Corners: none  (C or CORNERS to pick)",
                    (10, y + 12), FONT, FS, (90, 90, 90), 1, cv2.LINE_AA)
    y += 18

    cv2.line(panel, (0, y), (PANEL_W, y), (80, 80, 80), 1)
    y += 10

    # World-frame readout (shown when detect mode is active)
    if state["detect_mode"]:
        cv2.putText(panel, "World frame (mm):", (10, y + 12),
                    FONT, FS, (255, 200, 80), 1, cv2.LINE_AA)
        y += 18
        det_world = state.get("det_world", [])
        if det_world:
            for lbl, wx, wy in det_world:
                dc = LABEL_COLORS.get(lbl, (200, 200, 200))
                cv2.putText(panel, f"  {lbl}: ({wx:.0f}, {wy:.0f})",
                            (10, y), FONT, FS, dc, 1, cv2.LINE_AA)
                y += 16
        else:
            cv2.putText(panel, "  (no objects detected)",
                        (10, y), FONT, FS, (100, 100, 100), 1, cv2.LINE_AA)
            y += 16
        cv2.line(panel, (0, y + 4), (PANEL_W, y + 4), (80, 80, 80), 1)
        y += 12

    cv2.putText(panel, "Tolerances (+/-):", (10, y + 12),
                FONT, FS, (200, 200, 200), 1, cv2.LINE_AA)
    y += 18
    for k, v in state["tol"].items():
        cv2.putText(panel, f"  {k}: {v}", (10, y),
                    FONT, FS, (160, 210, 255), 1, cv2.LINE_AA)
        y += 16

    cv2.line(panel, (0, y + 4), (PANEL_W, y + 4), (80, 80, 80), 1)
    y += 14

    for hint in ["H/h  Hue tol +/-", "A/a  Sat tol +/-", "V/v  Val tol +/-",
                 "U    Undo last sample", "R    Reset object",
                 "Enter/S  Save all", "Q    Quit"]:
        cv2.putText(panel, hint, (10, y), FONT, FS, (130, 130, 130), 1, cv2.LINE_AA)
        y += 15

    # Bottom action buttons
    y_bot = ph - 5 * (BTN_H + BTN_MARGIN) - 6
    for btn_label, btn_color, btn_key in [
        ("  SAVE ALL",  (0, 150, 70),  "save"),
        ("  DETECT",    (100, 0, 180), "detect_toggle"),
        ("  CORNERS",   (0,  80, 160), "corner_toggle"),
        ("  SET ROI",   (0, 100, 200), "roi_toggle"),
        ("  RESET SEL", (140, 60, 0),  "reset"),
    ]:
        active_roi     = (btn_key == "roi_toggle"    and state["roi_mode"])
        active_detect  = (btn_key == "detect_toggle" and state["detect_mode"])
        active_corners = (btn_key == "corner_toggle" and
                          (state["corner_mode"] or len(state["corners_px"]) == 4))
        bclr = ((0, 200, 80)  if active_roi     else
                (0, 220, 180) if active_detect  else
                (0, 200, 120) if active_corners else btn_color)
        x0, y0 = BTN_MARGIN, y_bot
        x1, y1 = PANEL_W - BTN_MARGIN, y_bot + BTN_H
        cv2.rectangle(panel, (x0, y0), (x1, y1), bclr, -1)
        cv2.rectangle(panel, (x0, y0), (x1, y1), (180, 180, 180), 1)
        cv2.putText(panel, btn_label, (x0 + 8, y0 + BTN_H // 2 + 7),
                    FONT, FL, (255, 255, 255), 2, cv2.LINE_AA)
        state["buttons"][btn_key] = (x0, y0, x1, y1)
        y_bot += BTN_H + BTN_MARGIN


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    import argparse
    parser = argparse.ArgumentParser(description="HSV color calibration tool")
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--no-undistort", action="store_true")
    args = parser.parse_args()

    cam_mat, dist = load_calibration()
    cap = open_camera(args.device)
    for _ in range(5):
        cap.read()

    state = {
        "selected":     OBJECT_LABELS[0],
        "last_sample":  None,     # most recently added [H,S,V] mean (for swatch)
        "samples":      {},       # label -> list of [H,S,V] float lists
        "sample_pts":   {},       # label -> list of (cx,cy) pixel positions
        "tol":          dict(DEFAULT_TOL),
        "ranges":       load_existing_json(),
        "buttons":      {},
        "cam_w":        0,
        "click":        None,
        # detect / ROI drag state
        "detect_mode":  False,
        "roi_mode":     False,
        "roi_dragging": False,
        "roi_start":    None,
        "roi_current":  None,
        # world-frame coords for last detected objects (label, wx, wy)
        "det_world":    [],
        # table corner calibration
        "corner_mode":       False,
        "corners_px":        [],   # list of (x,y) pixel coords, up to 4
        "corner_homography": None, # 3x3 px->world homography once 4 corners placed
    }

    WIN = "Color Calibration"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.waitKey(1)

    def on_mouse(event, x, y, flags, param):
        cam_w  = state["cam_w"]
        on_cam = x < cam_w

        if event == cv2.EVENT_LBUTTONDOWN:
            if on_cam:
                if state["corner_mode"]:
                    if len(state["corners_px"]) < 4:
                        state["corners_px"].append((x, y))
                        lbl = CORNER_ORDER[len(state["corners_px"]) - 1][0]
                        print(f"[INFO] Corner {len(state['corners_px'])}/4 "
                              f"({lbl}) at ({x},{y})")
                        if len(state["corners_px"]) == 4:
                            state["corner_mode"] = False
                elif state["roi_mode"]:
                    state["roi_dragging"] = True
                    state["roi_start"]    = (x, y)
                    state["roi_current"]  = (x, y)
                else:
                    state["click"] = (x, y)
            else:
                px, py = x - cam_w, y
                for key, (x0, y0, x1, y1) in state["buttons"].items():
                    if x0 <= px <= x1 and y0 <= py <= y1:
                        _handle_btn(key, state)
                        break

        elif event == cv2.EVENT_MOUSEMOVE:
            if state["roi_dragging"] and on_cam:
                state["roi_current"] = (x, y)

        elif event == cv2.EVENT_LBUTTONUP:
            if state["roi_dragging"]:
                state["roi_dragging"] = False
                if state["roi_start"] and state["roi_current"]:
                    roi = roi_from_drag(state["roi_start"], state["roi_current"])
                    if roi[2] > 4 and roi[3] > 4:
                        sel = state["selected"]
                        state["ranges"].setdefault(sel, {})["roi"] = roi
                        print(f"[INFO] ROI set for [{sel}]: {roi}")
                state["roi_start"]   = None
                state["roi_current"] = None
                state["roi_mode"]    = False

    cv2.setMouseCallback(WIN, on_mouse)
    print("[INFO] Running — press Q to quit, Enter to save.")

    while True:
        ret, raw = cap.read()
        if not ret:
            print("[WARN] Dropped frame.")
            continue

        frame = (undistort(raw, cam_mat, dist)
                 if (not args.no_undistort and cam_mat is not None)
                 else raw)
        fh, fw = frame.shape[:2]
        state["cam_w"] = fw

        # Process color click — accumulate samples
        if state["click"] is not None:
            cx, cy = state["click"]
            state["click"] = None
            mean_hsv, std_hsv = sample_hsv(frame, cx, cy)
            state["last_sample"] = mean_hsv
            sel = state["selected"]
            state["samples"].setdefault(sel, []).append(mean_hsv.tolist())
            state["sample_pts"].setdefault(sel, []).append((cx, cy))
            all_samples = state["samples"][sel]
            lo, hi = build_range_multi(all_samples, state["tol"])
            state["ranges"].setdefault(sel, {}).update({"lower": lo, "upper": hi})
            print(f"[INFO] [{sel}] sample #{len(all_samples)}  "
                  f"HSV=({int(mean_hsv[0])},{int(mean_hsv[1])},{int(mean_hsv[2])})  "
                  f"std=({std_hsv[0]:.1f},{std_hsv[1]:.1f},{std_hsv[2]:.1f})  "
                  f"range: lower={lo}  upper={hi}")

        # Draw crosshairs for all accumulated sample points for selected object
        sel = state["selected"]
        for px, py in state["sample_pts"].get(sel, []):
            cv2.drawMarker(frame, (px, py), (0, 255, 255),
                           cv2.MARKER_CROSS, 20, 2)

        # Color mask overlay for selected object
        entry = state["ranges"].get(sel, {})
        if "lower" in entry and "upper" in entry:
            lo_arr = np.array(entry["lower"], dtype=np.uint8)
            hi_arr = np.array(entry["upper"], dtype=np.uint8)
            hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lo_arr, hi_arr)
            tint = frame.copy()
            tint[mask > 0] = np.clip(
                tint[mask > 0].astype(np.int32) // 2 + np.array([0, 100, 0]),
                0, 255).astype(np.uint8)
            frame = tint

        # Draw committed ROI rectangles for ALL objects
        for lbl, lentry in state["ranges"].items():
            if "roi" in lentry:
                rx, ry, rw, rh = lentry["roi"]
                rc    = LABEL_COLORS.get(lbl, (200, 200, 200))
                thick = 3 if lbl == sel else 1
                cv2.rectangle(frame, (rx, ry), (rx + rw, ry + rh), rc, thick)
                cv2.putText(frame, lbl, (rx + 4, ry + 14),
                            FONT, 0.45, rc, 1, cv2.LINE_AA)

        # ROI drag live preview
        if state["roi_dragging"] and state["roi_start"] and state["roi_current"]:
            cv2.rectangle(frame, state["roi_start"], state["roi_current"],
                          (0, 255, 255), 2)

        # Table corners overlay
        cpts = state["corners_px"]
        if cpts:
            ccolors = [(0,255,255),(0,200,255),(0,150,255),(0,100,255)]
            for i, (px, py) in enumerate(cpts):
                c_name, (wx, wy) = CORNER_ORDER[i]
                cv2.circle(frame, (px, py), 9, ccolors[i], -1)
                cv2.circle(frame, (px, py), 9, (255,255,255), 1)
                line1 = f"{i+1}: {c_name}"
                line2 = f"({wx:+d},{wy:+d})mm"
                cv2.putText(frame, line1, (px + 11, py - 8),
                            FONT, 0.42, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(frame, line2, (px + 11, py + 8),
                            FONT, 0.42, ccolors[i], 1, cv2.LINE_AA)
            for i in range(len(cpts) - 1):
                cv2.line(frame, cpts[i], cpts[i+1], (0, 220, 220), 1)
            if len(cpts) == 4:
                cv2.line(frame, cpts[3], cpts[0], (0, 220, 220), 1)
                if state["corner_homography"] is None:
                    src = np.array(cpts, dtype=np.float32)
                    dst = np.array([c[1] for c in CORNER_ORDER], dtype=np.float32)
                    H, _ = cv2.findHomography(src, dst)
                    state["corner_homography"] = H
                    print("[INFO] Table homography computed from corners.")
        if len(cpts) < 4 and state["corner_homography"] is not None:
            state["corner_homography"] = None

        # Live detection overlay
        if state["detect_mode"] and _DETECT_AVAILABLE:
            det_world_this_frame = []
            for lbl, lentry in state["ranges"].items():
                if "lower" not in lentry or "upper" not in lentry:
                    continue
                lo_d  = np.array(lentry["lower"], dtype=np.uint8)
                hi_d  = np.array(lentry["upper"], dtype=np.uint8)
                roi_d = lentry.get("roi", None)
                dets  = _find_blobs(frame, lbl, lo_d, hi_d, roi=roi_d)
                for d in dets:
                    bx, by, bw, bh = d["bbox"]
                    dcx, dcy = d["centroid"]
                    dc = LABEL_COLORS.get(lbl, (255, 255, 255))
                    cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), dc, 2)
                    cv2.circle(frame, (dcx, dcy), 5, dc, -1)
                    if cam_mat is not None or state.get("corner_homography") is not None:
                        hom = state.get("corner_homography")
                        world = _pixel_to_world(dcx, dcy, cam_mat, homography=hom)
                        if world is not None:
                            wx, wy, _ = world
                            det_world_this_frame.append((lbl, wx, wy))
                            label_txt = f"{lbl}  W=({wx:.0f},{wy:.0f})mm"
                        else:
                            label_txt = f"{lbl}  (no corners set)"
                    else:
                        label_txt = lbl
                    cv2.putText(frame, label_txt, (bx, by - 6),
                                FONT, 0.45, dc, 1, cv2.LINE_AA)
            state["det_world"] = det_world_this_frame
        else:
            state["det_world"] = []

        panel = np.zeros((fh, PANEL_W, 3), dtype=np.uint8)
        draw_panel(panel, state)
        cv2.imshow(WIN, np.hstack([frame, panel]))

        key = cv2.waitKey(1) & 0xFF
        if   key in (ord('q'), ord('Q')): break
        elif key == 13:                   _do_save(state)
        elif key == ord('S'):             _do_save(state)
        elif key == ord('H'):             _adj_tol(state, "H", +TOL_STEP)
        elif key == ord('h'):             _adj_tol(state, "H", -TOL_STEP)
        elif key == ord('A'):             _adj_tol(state, "S", +TOL_STEP)
        elif key == ord('a'):             _adj_tol(state, "S", -TOL_STEP)
        elif key == ord('V'):             _adj_tol(state, "V", +TOL_STEP)
        elif key == ord('v'):             _adj_tol(state, "V", -TOL_STEP)
        elif key in (ord('d'), ord('D')): _handle_btn("detect_toggle", state)
        elif key in (ord('c'), ord('C')): _handle_btn("corner_toggle", state)
        elif key in (ord('u'), ord('U')): _undo_sample(state)
        elif key in (ord('r'), ord('R')): _do_reset(state)

    cap.release()
    cv2.destroyAllWindows()


# ---------------------------------------------------------------------------
# State helpers
# ---------------------------------------------------------------------------

def _handle_btn(key, state):
    if key in OBJECT_LABELS:
        state["selected"] = key
        state["roi_mode"] = False
        # Restore last_sample from this label's history if any
        prev = state["samples"].get(key, [])
        state["last_sample"] = np.array(prev[-1]) if prev else None
        print(f"[INFO] Selected: {key}  ({len(prev)} sample(s))")
    elif key == "save":
        _do_save(state)
    elif key == "detect_toggle":
        if not _DETECT_AVAILABLE:
            print("[WARN] object_detection.py not importable — detect unavailable.")
        else:
            state["detect_mode"] = not state["detect_mode"]
            print(f"[INFO] Detect mode {'ON' if state['detect_mode'] else 'OFF'}")
    elif key == "corner_toggle":
        if state["corner_mode"]:
            state["corner_mode"] = False
            print("[INFO] Corner mode OFF")
        elif len(state["corners_px"]) == 4:
            state["corners_px"]        = []
            state["corner_homography"] = None
            state["corner_mode"]       = True
            print("[INFO] Corners reset — click Near-Left corner first.")
        else:
            state["corner_mode"] = True
            n   = len(state["corners_px"])
            nxt = CORNER_ORDER[n][0] if n < 4 else "done"
            print(f"[INFO] Corner mode ON  [{n}/4] — click {nxt}")
    elif key == "roi_toggle":
        state["roi_mode"] = not state["roi_mode"]
        print(f"[INFO] ROI mode {'ON' if state['roi_mode'] else 'OFF'} "
              f"for [{state['selected']}]")
    elif key == "reset":
        _do_reset(state)


def _adj_tol(state, channel, delta):
    limits = {"H": (1, 89), "S": (1, 127), "V": (1, 127)}
    lo_lim, hi_lim = limits[channel]
    state["tol"][channel] = int(np.clip(state["tol"][channel] + delta, lo_lim, hi_lim))
    sel     = state["selected"]
    samples = state["samples"].get(sel, [])
    if samples:
        lo_v, hi_v = build_range_multi(samples, state["tol"])
        state["ranges"].setdefault(sel, {}).update({"lower": lo_v, "upper": hi_v})
        print(f"[INFO] Tol {channel}={state['tol'][channel]}  "
              f"lower={lo_v}  upper={hi_v}")


def _do_save(state):
    save_json(state["ranges"])
    # Save table corner calibration if all 4 corners are placed
    hom  = state.get("corner_homography")
    cpts = state.get("corners_px", [])
    if hom is not None and len(cpts) == 4:
        calib = {
            "corners_px":       [list(p) for p in cpts],
            "corners_world_mm": [list(c[1]) for c in CORNER_ORDER],
            "homography":       hom.tolist(),
        }
        with open(TABLE_CALIB_JSON, "w") as f:
            json.dump(calib, f, indent=2)
        print(f"[INFO] Table calibration saved -> {TABLE_CALIB_JSON}")


def _undo_sample(state):
    sel     = state["selected"]
    samples = state["samples"].get(sel, [])
    pts     = state["sample_pts"].get(sel, [])
    if not samples:
        print(f"[INFO] No samples to undo for [{sel}]")
        return
    samples.pop()
    if pts:
        pts.pop()
    if samples:
        lo, hi = build_range_multi(samples, state["tol"])
        state["ranges"].setdefault(sel, {}).update({"lower": lo, "upper": hi})
        state["last_sample"] = np.array(samples[-1])
    else:
        entry = state["ranges"].get(sel, {})
        entry.pop("lower", None)
        entry.pop("upper", None)
        state["last_sample"] = None
    print(f"[INFO] Undid last sample for [{sel}]. {len(samples)} remaining.")


def _do_reset(state):
    sel = state["selected"]
    state["ranges"].pop(sel, None)
    state["samples"].pop(sel, None)
    state["sample_pts"].pop(sel, None)
    state["last_sample"] = None
    state["roi_mode"]    = False
    print(f"[INFO] Reset [{sel}]")


if __name__ == "__main__":
    main()
