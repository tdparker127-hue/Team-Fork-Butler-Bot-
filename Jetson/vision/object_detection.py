"""
object_detection.py — Color-based blob detection and edge detection for
tabletop items (cups, bottle, bowl, plate, tray).

Usage:
    # Capture a new photo from the camera and process it
    python object_detection.py --capture

    # Process an existing image file
    python object_detection.py --image path/to/image.jpg

    # Save the captured/processed image to a specific path
    python object_detection.py --capture --output result.jpg

Optional flags:
    --no-undistort      Skip lens-undistortion (faster, less accurate)
    --show              Display the annotated image in a window
    --device N          Camera device index or path (default: /dev/video4)
    --edges             Also run Canny edge detection overlay
"""

import argparse
import json
import os
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

try:
    import pyrealsense2 as rs
    _RS_AVAILABLE = True
except ImportError:
    _RS_AVAILABLE = False

_repo_root = Path(__file__).resolve().parent.parent.parent
if str(_repo_root) not in sys.path:
    sys.path.insert(0, str(_repo_root))

from Jetson.config import (
    DEPTH_CAMERA_DEVICE,
    OBJECT_COLORS, DISPLAY_COLORS, OBJECT_LABELS,
    MIN_BLOB_AREA,
    TABLE_WIDTH_MM, TABLE_DEPTH_MM, CAM_HEIGHT_MM, CAM_WORLD_X_MM, CAM_WORLD_Y_MM,
)

# ---------------------------------------------------------------------------
# Camera / calibration
# ---------------------------------------------------------------------------
CALIB_FILE        = Path(__file__).parent / "camera_calibration_live.npz"
COLOR_JSON        = Path(__file__).parent / "color_ranges.json"
TABLE_CALIB_JSON  = Path(__file__).parent / "table_calibration.json"

# Camera device used when capturing a new frame — imported from Jetson/config.py
DEFAULT_DEVICE = DEPTH_CAMERA_DEVICE

# ---------------------------------------------------------------------------
# HSV color ranges — imported from Jetson/config.py (OBJECT_COLORS)
# Display colors — imported from Jetson/config.py (DISPLAY_COLORS)
# ---------------------------------------------------------------------------


def load_color_ranges() -> dict:
    """
    Load HSV color ranges from color_ranges.json (written by color_calibrate.py).
    Falls back to the hardcoded OBJECT_COLORS defaults for any missing labels.
    Returns a dict mapping label -> (lower_np, upper_np, roi_or_None)
    where roi_or_None is [x, y, w, h] or None.
    """
    result = {label: (lo.copy(), hi.copy(), None)
              for label, (lo, hi) in OBJECT_COLORS.items()}

    if not COLOR_JSON.exists():
        print(f"[INFO] No color_ranges.json found — using hardcoded HSV defaults.")
        return result

    with open(COLOR_JSON) as f:
        data = json.load(f)

    overridden = []
    for label, entry in data.items():
        roi = entry.get("roi", None)  # [x,y,w,h] or absent
        result[label] = (
            np.array(entry["lower"], dtype=np.uint8),
            np.array(entry["upper"], dtype=np.uint8),
            roi,
        )
        overridden.append(label)

    print(f"[INFO] Loaded color ranges from {COLOR_JSON}  "
          f"(overrode: {', '.join(overridden) if overridden else 'none'})")
    return result


def load_table_calibration():
    """
    Load the pixel→world homography saved by color_calibrate.py.
    Returns a 3×3 np.float32 matrix, or None if the file doesn't exist.
    """
    if not TABLE_CALIB_JSON.exists():
        return None
    with open(TABLE_CALIB_JSON) as f:
        data = json.load(f)
    if "homography" not in data:
        return None
    H = np.array(data["homography"], dtype=np.float32)
    corners = data.get("corners_px", [])
    print(f"[INFO] Loaded table homography from {TABLE_CALIB_JSON}  "
          f"(corners: {corners})")
    return H


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def load_calibration(path: Path):
    """Load camera_matrix and dist_coeffs from an .npz file."""
    if not path.exists():
        print(f"[WARN] Calibration file not found: {path}")
        print("       Lens undistortion will be skipped.")
        return None, None
    data = np.load(str(path))
    camera_matrix = data["camera_matrix"]
    dist_coeffs   = data["dist_coeffs"]
    print(f"[INFO] Loaded calibration from {path}")
    print(f"       fx={camera_matrix[0,0]:.2f}  fy={camera_matrix[1,1]:.2f}"
          f"  cx={camera_matrix[0,2]:.2f}  cy={camera_matrix[1,2]:.2f}")
    return camera_matrix, dist_coeffs


def undistort_frame(frame, camera_matrix, dist_coeffs):
    """Undistort a frame using the loaded intrinsics."""
    h, w = frame.shape[:2]
    new_cam, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (w, h), 1, (w, h))
    undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs,
                                None, new_cam)
    # Crop to the valid region
    x, y, rw, rh = roi
    if rw > 0 and rh > 0:
        undistorted = undistorted[y:y+rh, x:x+rw]
    return undistorted


def capture_frame(device=DEFAULT_DEVICE):
    """Open the camera, grab one frame, and release."""
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  720)
    cap.set(cv2.CAP_PROP_FPS, 15)

    if not cap.isOpened():
        print(f"[ERROR] Could not open camera: {device}")
        sys.exit(1)

    # Discard a few frames so the sensor exposure settles
    for _ in range(5):
        cap.read()

    ret, frame = cap.read()
    cap.release()

    if not ret or frame is None:
        print("[ERROR] Failed to capture frame from camera.")
        sys.exit(1)

    print(f"[INFO] Captured frame  ({frame.shape[1]}x{frame.shape[0]})")
    return frame


def capture_realsense():
    """
    Capture a color frame + aligned depth frame via the RealSense SDK.

    Returns:
        color_bgr   — HxWx3 uint8 BGR image
        depth_mm    — HxW uint16 array of depth in millimetres
        intr        — rs.intrinsics of the colour stream (for back-projection)
    """
    if not _RS_AVAILABLE:
        print("[ERROR] pyrealsense2 not available.  "
              "Install it or use --capture instead of --depth.")
        sys.exit(1)

    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8,  15)
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16,   15)

    profile  = pipeline.start(config)
    align    = rs.align(rs.stream.color)

    try:
        # Warm up — let AE/AF settle
        for _ in range(10):
            pipeline.wait_for_frames()

        frames       = pipeline.wait_for_frames()
        aligned      = align.process(frames)
        color_frame  = aligned.get_color_frame()
        depth_frame  = aligned.get_depth_frame()

        if not color_frame or not depth_frame:
            print("[ERROR] RealSense: could not get aligned frames.")
            sys.exit(1)

        color_bgr = np.asanyarray(color_frame.get_data())
        depth_mm  = (np.asanyarray(depth_frame.get_data())
                     .astype(np.float32) * depth_frame.get_units() * 1000
                     ).astype(np.uint16)

        intr = (profile.get_stream(rs.stream.color)
                .as_video_stream_profile().get_intrinsics())

        print(f"[INFO] RealSense color {color_bgr.shape[1]}x{color_bgr.shape[0]}  "
              f"depth {depth_mm.shape[1]}x{depth_mm.shape[0]}")
        return color_bgr, depth_mm, intr
    finally:
        pipeline.stop()


# Default assumed table distance from camera when depth mode is off
DEFAULT_TABLE_DEPTH_MM = 800


def depth_correct_centroid(u, v, depth_mm_map, intr, table_depth_mm):
    """
    Given the image-space centroid of the *top* of a tall object and a depth
    map, return the image-space position of the object's *base* on the table
    and its 3-D position in camera-frame millimetres.

    Args:
        u, v            — centroid pixel column / row (top of object)
        depth_mm_map    — HxW uint16 depth array in mm
        intr            — rs.intrinsics (or dict with fx,fy,ppx,ppy)
        table_depth_mm  — known depth of the table surface in mm

    Returns:
        u_base, v_base  — corrected centroid on the table plane
        pos_3d_mm       — (X, Y, Z) in camera-frame mm at table depth
        height_mm       — estimated object height in mm
    """
    h_map, w_map = depth_mm_map.shape[:2]

    # Sample a 5x5 patch around the centroid to get a robust depth estimate
    PATCH = 2
    r0 = max(0, v - PATCH);  r1 = min(h_map, v + PATCH + 1)
    c0 = max(0, u - PATCH);  c1 = min(w_map, u + PATCH + 1)
    patch = depth_mm_map[r0:r1, c0:c1].astype(np.float32)
    valid = patch[patch > 0]
    z_top = float(np.median(valid)) if valid.size > 0 else float(table_depth_mm)

    # Perspective correction: project centroid back to the table plane
    # rs.intrinsics attributes: fx, fy, ppx, ppy
    if hasattr(intr, 'fx'):
        fx, fy, cx, cy = intr.fx, intr.fy, intr.ppx, intr.ppy
    else:  # dict fallback
        fx, fy, cx, cy = intr['fx'], intr['fy'], intr['ppx'], intr['ppy']

    z_table = float(table_depth_mm)
    scale   = z_table / z_top if z_top > 0 else 1.0

    u_base = int((u - cx) * scale + cx)
    v_base = int((v - cy) * scale + cy)

    # 3-D position of the base in camera frame (mm)
    X = (u_base - cx) * z_table / fx
    Y = (v_base - cy) * z_table / fy
    Z = z_table

    height_mm = max(0.0, z_table - z_top)
    return (u_base, v_base), (X, Y, Z), height_mm


def pixel_to_world(u, v, camera_matrix, homography=None):
    """
    Map image pixel (u, v) to world-frame coordinates (mm) using the
    4-corner homography computed from the table calibration.

    The homography is a 3x3 matrix computed by cv2.findHomography from
    the 4 table corners clicked in color_calibrate.py mapped to their
    known world positions (see CORNER_ORDER / table_calibration.json).

    Returns (X_mm, Y_mm, 0.0), or None if no homography is available.
    """
    if homography is None:
        return None
    pt = np.array([[[float(u), float(v)]]], dtype=np.float32)
    world_pt = cv2.perspectiveTransform(pt, homography)[0][0]
    return (float(world_pt[0]), float(world_pt[1]), 0.0)


# ---------------------------------------------------------------------------
# Detection
# ---------------------------------------------------------------------------

def find_blobs(frame_bgr, label, lower_hsv, upper_hsv, roi=None):
    """
    Find the dominant color blob for label within the optional roi=[x,y,w,h].
    Always returns at most one detection (the largest qualifying blob).
    Bounding box is derived from the contour shape.
    """
    if roi is not None:
        rx, ry, rw, rh = roi
        region = frame_bgr[ry:ry + rh, rx:rx + rw]
        ox, oy = rx, ry
    else:
        region = frame_bgr
        ox, oy = 0, 0

    hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)

    # Red wraps around 0/180 — handle both halves
    if label == "cup_red":
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, lower_hsv, upper_hsv),
            cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255])))
    else:
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return []

    cnt = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(cnt)
    if area < MIN_BLOB_AREA:
        return []

    M = cv2.moments(cnt)
    if M["m00"] == 0:
        return []

    cx = int(M["m10"] / M["m00"]) + ox
    cy = int(M["m01"] / M["m00"]) + oy
    bx, by, bw, bh = cv2.boundingRect(cnt)

    return [{
        "label":    label,
        "centroid": (cx, cy),
        "area":     area,
        "bbox":     (bx + ox, by + oy, bw, bh),
        "contour":  cnt + np.array([[[ox, oy]]]),
    }]


def run_edge_detection(frame_bgr):
    """Return a Canny edge image (3-channel BGR for overlay)."""
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, threshold1=50, threshold2=150)
    return cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)


def annotate(frame, detections, edges=None):
    """Draw bounding boxes, centroids, and labels onto the frame."""
    out = frame.copy()

    if edges is not None:
        # Blend edge overlay at 30% opacity
        out = cv2.addWeighted(out, 1.0, edges, 0.3, 0)

    for det in detections:
        label   = det["label"]
        cx, cy  = det["centroid"]
        x, y, w, h = det["bbox"]
        color   = DISPLAY_COLORS.get(label, (255, 255, 255))

        # Bounding rectangle
        cv2.rectangle(out, (x, y), (x + w, y + h), color, 2)
        # Contour fill (semi-transparent)
        overlay = out.copy()
        cv2.drawContours(overlay, [det["contour"]], -1, color, -1)
        out = cv2.addWeighted(out, 0.75, overlay, 0.25, 0)
        # Centroid dot
        cv2.circle(out, (cx, cy), 5, color, -1)
        # If depth-corrected base is available, draw it differently
        if "base_px" in det:
            bx, by = det["base_px"]
            cv2.circle(out, (bx, by), 7, color, 2)
            cv2.line(out, (cx, cy), (bx, by), color, 1)
            h_mm = det.get("height_mm", 0)
            p3   = det.get("pos_3d_mm", (0, 0, 0))
            text = (f"{label}  base=({bx},{by})  "
                    f"3D=({p3[0]:.0f},{p3[1]:.0f},{p3[2]:.0f})mm  H={h_mm:.0f}mm")
        else:
            text = f"{label}  ({cx},{cy})"
            if "world_mm" in det:
                wx, wy, _ = det["world_mm"]
                text += f"  W=({wx:.0f},{wy:.0f})mm"
        cv2.putText(out, text, (x, y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

    return out


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Tabletop object color-blob detector")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--capture", action="store_true",
                      help="Capture a new photo from the camera (V4L2)")
    mode.add_argument("--image",   type=str, metavar="PATH",
                      help="Path to an existing image file")
    mode.add_argument("--depth",   action="store_true",
                      help="Capture color + depth via RealSense SDK")

    parser.add_argument("--table-depth", type=int, default=DEFAULT_TABLE_DEPTH_MM,
                        metavar="MM",
                        help=f"Table surface depth in mm (default: {DEFAULT_TABLE_DEPTH_MM})")
    parser.add_argument("--output",       type=str, default=None,
                        help="Save annotated image to this path")
    parser.add_argument("--save-raw",     type=str, default=None,
                        help="Also save the raw (undistorted) capture")
    parser.add_argument("--no-undistort", action="store_true",
                        help="Skip lens undistortion")
    parser.add_argument("--show",         action="store_true",
                        help="Display result in a GUI window")
    parser.add_argument("--edges",        action="store_true",
                        help="Overlay Canny edge detection")
    parser.add_argument("--device",       type=str, default=DEFAULT_DEVICE,
                        help=f"Camera device (default: {DEFAULT_DEVICE})")
    args = parser.parse_args()

    # ------------------------------------------------------------------
    # 1. Load calibration (used for undistortion; depth uses RS intrinsics)
    # ------------------------------------------------------------------
    camera_matrix, dist_coeffs = load_calibration(CALIB_FILE)
    table_homography = load_table_calibration()

    # ------------------------------------------------------------------
    # 2. Acquire frame
    # ------------------------------------------------------------------
    depth_mm = None
    rs_intr  = None

    if args.depth:
        frame, depth_mm, rs_intr = capture_realsense()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        if args.save_raw:
            cv2.imwrite(args.save_raw, frame)
            print(f"[INFO] Raw color saved → {args.save_raw}")
    elif args.capture:
        frame = capture_frame(args.device)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        raw_path  = args.save_raw or f"capture_{timestamp}.jpg"
        cv2.imwrite(raw_path, frame)
        print(f"[INFO] Raw capture saved → {raw_path}")
    else:
        if not os.path.isfile(args.image):
            print(f"[ERROR] File not found: {args.image}")
            sys.exit(1)
        frame = cv2.imread(args.image)
        if frame is None:
            print(f"[ERROR] Could not read image: {args.image}")
            sys.exit(1)
        print(f"[INFO] Loaded image  ({frame.shape[1]}x{frame.shape[0]})  "
              f"← {args.image}")

    # ------------------------------------------------------------------
    # 3. Undistort (skip in depth mode — RS provides undistorted frames)
    # ------------------------------------------------------------------
    if not args.no_undistort and not args.depth and camera_matrix is not None:
        frame = undistort_frame(frame, camera_matrix, dist_coeffs)
        print(f"[INFO] Undistorted frame  ({frame.shape[1]}x{frame.shape[0]})")
    else:
        print("[INFO] Skipping undistortion.")

    # ------------------------------------------------------------------
    # 4. Blob detection for each object class
    # ------------------------------------------------------------------
    color_ranges = load_color_ranges()
    all_detections = []
    for label, (lo, hi, roi) in color_ranges.items():
        dets = find_blobs(frame, label, lo, hi, roi=roi)
        if dets:
            for d in dets:
                roi_tag = f"  roi={roi}" if roi else ""
                cx, cy  = d["centroid"]

                # World-frame projection via corner homography
                if camera_matrix is not None or table_homography is not None:
                    world = pixel_to_world(cx, cy, camera_matrix,
                                           homography=table_homography)
                    if world is not None:
                        d["world_mm"] = world
                    else:
                        world = None
                    wx, wy, wz = world if world else (None, None, None)
                else:
                    wx, wy, wz = None, None, None

                # Depth-based perspective correction
                if depth_mm is not None and rs_intr is not None:
                    base_px, pos_3d, height = depth_correct_centroid(
                        cx, cy, depth_mm, rs_intr, args.table_depth)
                    d["base_px"]    = base_px
                    d["pos_3d_mm"]  = pos_3d
                    d["height_mm"]  = height
                    print(f"  [{label}]  top=({cx},{cy})  "
                          f"base=({base_px[0]},{base_px[1]})  "
                          f"3D=({pos_3d[0]:.1f},{pos_3d[1]:.1f},{pos_3d[2]:.1f})mm  "
                          f"H={height:.1f}mm{roi_tag}")
                else:
                    w_str = (f"  world=({wx:.1f},{wy:.1f},0)mm"
                             if wx is not None else "")
                    print(f"  [{label}]  centroid=({cx},{cy})  "
                          f"area={int(d['area'])}{w_str}{roi_tag}")
        all_detections.extend(dets)

    if not all_detections:
        print("[INFO] No objects detected — try tuning HSV ranges in OBJECT_COLORS.")

    # ------------------------------------------------------------------
    # 5. Edge detection (optional)
    # ------------------------------------------------------------------
    edges = run_edge_detection(frame) if args.edges else None

    # ------------------------------------------------------------------
    # 6. Annotate and save/show
    # ------------------------------------------------------------------
    annotated = annotate(frame, all_detections, edges)

    output_path = args.output
    if output_path is None:
        timestamp   = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = f"detected_{timestamp}.jpg"

    cv2.imwrite(output_path, annotated)
    print(f"[INFO] Annotated image saved → {output_path}")

    if args.show:
        cv2.imshow("Object Detection", annotated)
        print("[INFO] Press any key to close.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
