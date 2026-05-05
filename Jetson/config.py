"""
config.py — Single source of truth for all tunable parameters.

Every constant listed in TUNING.md lives here.  Edit this file to tune the
robot; no other Python files need to be touched.

Sections mirror TUNING.md:
  1.1  Robot Geometry
  1.2  Camera Mounting
  1.3  AprilTag Layout
  1.4  Arm Travel Limits
  2    Arm Pickup Sequence Setpoints
  5    Waypoint Navigation Gains
  6    EKF Localizer Noise
  7    Person Detection Thresholds
"""

import numpy as np


# ===========================================================================
# Section 1.1 — Robot Geometry
# TUNING.md reference: "1.1 Robot Geometry"
# ===========================================================================

WHEEL_R = 0.045   # wheel radius [m]  — TUNING.md lists 0.06; measure and update
L_X     = 0.1665  # half wheelbase front-to-back (longitudinal) [m]
L_Y     = 0.2075    # half wheelbase left-to-right  (lateral)     [m]

MAX_Y = 0.525 # most forward point on robot relative to center [m]


# ===========================================================================
# Section 1.2 — Camera Mounting
# TUNING.md reference: "1.2 Camera Mounting"
# ===========================================================================

# T_CAM_ROBOT: 4x4 rigid transform mapping a point from the CAMERA frame to
# the ROBOT BASE frame.
#   p_robot = T_CAM_ROBOT[:3,:3] @ p_cam + T_CAM_ROBOT[:3,3]
#
# Row/col 3 encodes translation (metres):
#   [0,3] = lateral offset (+ = right of robot centre)
#   [1,3] = forward offset (+ = forward of robot centre)
#   [2,3] = height of camera above robot base origin
T_CAM_DEPTH: np.ndarray = np.array(
    [
        [1.0, 0.0, 0.0, 0.00],  #DEBUG TDP edited for new depth camera 
        [0.0, 0.0, 1.0, 0.0762],  
        [0.0, -1.0, 0.0, 0.953],  
        [0.0, 0.0, 0.0, 1.00],
         
     
    ],
    dtype=float,
)
T_CAM_ROBOT: np.ndarray = np.array(
    [
        # Rotation: maps camera-frame vectors into robot-frame vectors.
        # Camera frame (OpenCV): +X right, +Y down, +Z forward (into scene)
        # Robot  frame:          +X right, +Y forward, +Z up
        #   cam X (right)   → robot  X (right) .............. col 0 = [1,  0,  0]
        #   cam Y (down)    → robot -Z (-up) ................. col 1 = [0,  0, -1]
        #   cam Z (forward) → robot  Y (forward) ............. col 2 = [0,  1,  0]
        # Translation: camera position in robot frame [right, forward, up]
        [1.0, 0.0, 0.0, 0.00],  #Original values These seem to be for the camera down below not the depth camera?
        [0.0, 1.0, 0.0, 0.28],
        [0.0, 0.0, 1.0, 0.2155],
        [0.0, 0.0, 0.0, 1.00],
    ],
    dtype=float,
)


# ===========================================================================
# Section 1.3 — AprilTag Layout
# TUNING.md reference: "1.3 AprilTag Layout"
# ===========================================================================

TAG_FAMILY  = "tag36h11"
TAG_SIZE_M  = 0.10   # printed tag side length (outer black border) [m]
TAG_HEIGHT_M = 0.305  # tag centre height above the floor [m]  — measure and update

# ── World-frame coordinate convention ──────────────────────────────────────
# Origin (0, 0, 0) = AprilTag 1's position projected onto the floor.
#   +X  → rightward  (when facing the tag-1 wall from inside the arena)
#   +Y  → away from the tag-1 wall (depth into arena)
#   +Z  → up


def wall_tag_rotation(facing_angle_deg: float) -> np.ndarray:
    """
    Build the 3x3 rotation matrix R_wt for a wall-mounted AprilTag.

    facing_angle_deg: direction the tag FACE points (CCW from world +X)
      0   → faces +X (EAST,  tag is on west  wall)
     90   → faces +Y (NORTH, tag is on south wall)
    180   → faces -X (WEST,  tag is on east  wall)
    270   → faces -Y (SOUTH, tag is on north wall)
    """
    phi = np.radians(facing_angle_deg)
    tag_z = np.array([ np.cos(phi),  np.sin(phi), 0.0])
    tag_x = np.array([-np.sin(phi),  np.cos(phi), 0.0])
    tag_y = np.array([ 0.0,          0.0,          1.0])
    return np.column_stack([tag_x, tag_y, tag_z])


def tag_pose(x: float, y: float, facing_angle_deg: float,
             height_m: float = TAG_HEIGHT_M) -> dict:
    """
    Define a tag world pose in arena coordinates (origin = arena corner (0,0)).

    x, y             : absolute arena position [m] — matches mission_config.yaml
    facing_angle_deg : direction the tag face points (see wall_tag_rotation)
    height_m         : tag centre height above the floor [m]
    """
    return {
        "position": np.array([x, y, height_m]),
        "rotation": wall_tag_rotation(facing_angle_deg),
    }


# All positions match mission_config.yaml exactly.
# Origin = arena corner (0, 0).  +X rightward, +Y away from front wall.
#
# facing_angle_deg convention:
#   0   → EAST  (tag on west  wall, faces into arena along +X)
#  90   → NORTH (tag on south wall, faces into arena along +Y)
# 180   → WEST  (tag on east  wall, faces into arena along -X)
# 270   → SOUTH (tag on north wall, faces into arena along -Y)
TAG_WORLD_POSES = {
    # heading converted from radians to degrees (facing_angle_deg = outward normal direction)

    # Tag 1 — table_tray.  heading=0 rad → faces +X (0°)
    1: tag_pose(x=3.5700, y=2.5650, facing_angle_deg=  0.0),

    # Tag 2 — table_back_left.  heading=3.14 rad → faces -X (180°)
    2: tag_pose(x=3.8550, y=2.8650, facing_angle_deg=180.0),

    # Tag 3 — ramp_tag_2.  heading=0 rad → faces +X (0°)
    3: tag_pose(x=2.0575, y=2.8525, facing_angle_deg=  0.0),

    # Tag 4 — ramp_tag_1.  heading=-1.57 rad → faces -Y (270°)
    4: tag_pose(x=1.2725, y=1.9900, facing_angle_deg=270.0),

    # Tag 5 — left_tag_table.  heading=0 rad → faces +X (0°)
    5: tag_pose(x=0.3150, y=0.5900, facing_angle_deg=  0.0),

    # Tag 6 — right_tag_table.  heading=0 rad → faces +X (0°)
    6: tag_pose(x=0.3150, y=1.1750, facing_angle_deg=  0.0),

    # Tag 7 — table_back_right.  heading=3.14 rad → faces -X (180°)
    7: tag_pose(x=4.6550, y=1.5300, facing_angle_deg=180.0),

    # Tag 8 — dishwasher_tag.  heading=1.57 rad → faces +Y (90°)
    8: tag_pose(x=3.9725, y=0.3625, facing_angle_deg= 90.0),

    # Tag 9 — front_wall.  heading=1.57 rad → faces +Y (90°)
    9: tag_pose(x=0.9700, y=0.0600, facing_angle_deg= 90.0),

    # Tag 0 — back_wall.  heading=0 rad → faces +X (0°)
    0: tag_pose(x=2.5950, y=3.5450, facing_angle_deg=  0.0),
}


# ===========================================================================
# Section 1.4 — Arm Travel Limits
# TUNING.md reference: "1.4 Arm Travel Limits"
# Must also match MIN/MAX_LIFT_RAD and MIN/MAX_GRIP_RAD in include/arm_drive.h
# ===========================================================================

MIN_LIFT_RAD = 0 # encoder pos when fully lowered  (TUNING: 0.0 ✅ — TODO update)
MAX_LIFT_RAD =  3.6 # encoder pos when fully raised   (TUNING: 2.2 ✅ — TODO update)
MIN_GRIP_RAD = 0   # encoder pos when fully closed   (TUNING: -2.0 ⚠️ TODO)
MAX_GRIP_RAD = 1.85 # encoder pos when fully open     (TUNING: 2.0  ⚠️ TODO)


# ===========================================================================
# Section 2 — Arm Pickup Sequence Setpoints
# TUNING.md reference: "2. Arm Pickup Sequence Setpoints"
# ===========================================================================

ARM_LOWER_LIFT   = -2.5   # lift position for reaching the tray [rad]
ARM_GRIP_CLOSE   =  0.0   # gripper closed position [rad]
ARM_CARRY_LIFT   =  1.5   # lift position while carrying [rad]
ARM_GRIP_OPEN    =  1.85   # gripper open position for release [rad]

ARM_LOWER_TIME_S =  0.1   # wait after sending lower command [s]
ARM_GRIP_TIME_S  =  0.1   # wait after sending grip command [s]
ARM_LIFT_TIME_S  =  0.1   # wait after lifting [s]


# ===========================================================================
# Section 5 — Waypoint Navigation Gains
# TUNING.md reference: "5. Waypoint Navigation Gains"
# ===========================================================================

KP_LIN              = 0.35   # forward gain   (dist error → ly)
KP_STRAFE           = 0.25   # strafe gain    (lateral error → lx)
KP_ANG              = 0.70   # angular gain   (heading error → yaw)
WAYPOINT_REACHED_M  = 0.10   # distance threshold to mark waypoint reached [m]
HEADING_REACHED_RAD = 0.08   # heading tolerance for final orientation [rad] (~5°)


# ===========================================================================
# Section 6 — EKF Localizer Noise
# TUNING.md reference: "6. EKF Localizer Noise Tuning"
# ===========================================================================

# 6.1 Process noise
SIGMA_XY    = 0.05   # position process noise density [m/s]
SIGMA_THETA = 0.02   # heading process noise density [rad/s]  (unused in predict; kept for Q_theta row)
SIGMA_VXY   = 0.30   # body-frame velocity process noise density [m/s²]
SIGMA_OMEGA_PROC = 0.20  # angular velocity process noise density [rad/s²]

# 6.2 IMU measurement noise
SIGMA_IMU_YAW = 0.035   # BNO08x heading noise [rad]

# 6.3 AprilTag measurement noise (for 1 tag; scales by 1/√n_tags)
SIGMA_TAG_XY  = 0.15     # per-axis position noise [m]  — ~15 cm realistic for camera at 1-3 m
SIGMA_TAG_YAW = 0.08     # heading noise [rad]          — ~4.5° realistic

# 6.3b Encoder velocity measurement noise
SIGMA_ENC_VXY   = 0.05   # body-frame translational velocity noise [m/s]
SIGMA_ENC_OMEGA = 0.04   # body-frame angular velocity noise [rad/s]

# 6.4 Gating thresholds — Mahalanobis
MAHAL_THRESH_IMU = 3.841   # chi²(1, 0.95) — IMU yaw gate
MAHAL_THRESH_TAG = 7.815   # chi²(3, 0.95) — AprilTag pose gate
MAHAL_THRESH_ENC = 7.815   # chi²(3, 0.95) — encoder velocity gate

# 6.4 Gating thresholds — Euclidean
EUCLID_THRESH_IMU_RAD         = 0.50   # max acceptable |yaw innovation| [rad]
EUCLID_THRESH_TAG_POS_M       = 0.50   # max acceptable sqrt(dx²+dy²) [m]
EUCLID_THRESH_TAG_YAW_RAD     = 0.40   # max acceptable |heading innovation| [rad]
EUCLID_THRESH_ENC_VXY_M_S     = 1.00   # max acceptable |velocity innovation| [m/s]
EUCLID_THRESH_ENC_OMEGA_RAD_S = 1.00   # max acceptable |omega innovation| [rad/s]

# 6.5 Motor sign convention  [FrLft, BkLft, FrRgt, BkRgt]
# Flip individual elements if odometry drifts in a consistent direction.
MOTOR_SIGNS = np.array([1.0, 1.0, 1.0, 1.0])


# ===========================================================================
# Section 7 — Person Detection Thresholds
# TUNING.md reference: "7. Person Detection Thresholds"
# ===========================================================================

PERSON_CONF    = 0.50   # YOLO minimum confidence
SLOW_DIST_M    = 2.0    # enter SLOW zone if person within this distance [m]
STOP_DIST_M    = 0.8    # enter STOP zone if person within this distance [m]
SLOW_BBOX_FRAC = 0.10   # fallback: SLOW if person bbox > 10% of frame area
STOP_BBOX_FRAC = 0.25   # fallback: STOP if person bbox > 25% of frame area


# ===========================================================================
# Section 8 — Camera Devices
# ===========================================================================

COLOR_CAMERA_DEVICE = "/dev/video6"    # RGB camera (AprilTags / color blob)
DEPTH_CAMERA_DEVICE = "/dev/video0"   # Depth / secondary RGB camera


# ===========================================================================
# Section 9 — Camera Calibration (chessboard tool)
# ===========================================================================

CALIB_CHESSBOARD_SIZE = (8, 6)   # interior corner count (cols, rows)
CALIB_SQUARE_SIZE_M   = 0.068    # physical square side length [m]


# ===========================================================================
# Section 10 — Object Detection (color blob)
# ===========================================================================

# HSV color ranges for each tracked object class: (lower_hsv, upper_hsv)
OBJECT_COLORS = {
    "cup_red": (
        np.array([0,   120,  70]),
        np.array([10,  255, 255]),
    ),
    "cup_blue": (
        np.array([100, 100,  50]),
        np.array([130, 255, 255]),
    ),
    "bottle": (
        np.array([35,  50,   50]),
        np.array([85,  255, 255]),
    ),
    "bowl": (
        np.array([15,  100,  80]),
        np.array([35,  255, 255]),
    ),
    "plate": (
        np.array([0,   0,   180]),
        np.array([180, 40,  255]),
    ),
    "tray": (
        np.array([0,   0,    50]),
        np.array([180, 40,  180]),
    ),
}

# Display colors (BGR) per object class — used in overlay annotations
DISPLAY_COLORS = {
    "cup_red":  (0,   0,   220),
    "cup_blue": (220, 80,   0),
    "bottle":   (0,   200,  50),
    "bowl":     (0,   180, 255),
    "plate":    (200, 200, 200),
    "tray":     (120,  60,   0),
}

# Ordered list of object class labels (matches OBJECT_COLORS keys)
OBJECT_LABELS = list(OBJECT_COLORS.keys())

# Minimum blob area in pixels — blobs smaller than this are ignored
MIN_BLOB_AREA = 500

# Table / world-frame geometry [mm]
TABLE_WIDTH_MM  = 910    # full width  (±455 mm in X)
TABLE_DEPTH_MM  = 540    # full depth  (0 → 540 mm in Y)
CAM_HEIGHT_MM   = 1230   # camera height above table surface
CAM_WORLD_X_MM  =   0    # camera principal axis X offset from table centre
CAM_WORLD_Y_MM  = 425    # camera principal axis Y position along table depth


# ===========================================================================
# Section 11 — Tray Detection & Approach
# ===========================================================================

# Bright-tray HSV mask (highly saturated colors)
BRIGHT_TRAY_LOWER_HSV = np.array([ 0,  150,  80], dtype=np.uint8)
BRIGHT_TRAY_UPPER_HSV = np.array([180, 255, 255], dtype=np.uint8)

# Labels from OBJECT_COLORS that are treated as trays
TRAY_LABELS = {"tray"}

# Minimum blob area for tray detection (pixels)
TRAY_MIN_AREA = 1500

# Distance from camera to ideal pickup position [mm]
TARGET_PICKUP_DIST_MM = 400.0

# Pixel offset from image centre below which tray is considered centred
CENTERING_DEAD_ZONE_PX = 20

# Proportional gains: pixel/mm error → normalized drive command
KP_APPROACH_LATERAL = 0.0015   # pixel error  → lx  (strafe)
KP_APPROACH_YAW     = 0.0008   # pixel error  → yaw (turn)
KP_APPROACH_FORWARD = 0.0008   # mm dist error → ly  (forward)

# Max approach command magnitudes
MAX_APPROACH_LX  = 0.40
MAX_APPROACH_LY  = 0.40
MAX_APPROACH_YAW = 0.30
