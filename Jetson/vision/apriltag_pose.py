"""
apriltag_pose.py — AprilTag-based robot localization.

Provides pure-geometry functions consumed by the _ApriltagWorker thread in
mission_controller.py:

  localize_camera(detections, tag_world_poses)
      → (cam_pos, R_wc, n_used)
      Estimates the camera's world-frame pose by weighted-averaging over all
      detected tags with known world poses.

  robot_pose_from_camera(cam_pos, R_wc, T_cam_robot=None)
      → (robot_x, robot_y, robot_yaw)
      Converts the camera world pose to a 2D robot floor pose using the
      camera-to-robot extrinsic transform.

Tag world poses are defined below in TAG_WORLD_POSES.  Tag 1 is the world
origin; all other tags are specified as (dx, dy) offsets from tag 1 using
the tag_pose() convenience function.
"""

import math

import cv2
import numpy as np

# ======================================================================
# AprilTag detection parameters
# ======================================================================
TAG_FAMILY = "tag36h11"
TAG_SIZE_M  = 0.10   # metres — physical tag side length (black border included)

# ======================================================================
# World-frame coordinate convention
# ======================================================================
# Origin (0, 0, 0) = AprilTag 1's position projected onto the floor.
#
#   +X  → rightward  (when facing the tag-1 wall from inside the arena)
#   +Y  → away from the tag-1 wall (depth into arena)
#   +Z  → up
#
# All tag positions and all waypoints share this coordinate system.
# ======================================================================

# Height of all wall-mounted AprilTags above the floor [m].
# Adjust to your physical setup.
TAG_HEIGHT_M = 1.20   # metres  <-- measure and update


# ======================================================================
# Wall-tag rotation helper
# ======================================================================

def wall_tag_rotation(facing_angle_deg: float) -> np.ndarray:
    """
    Build the 3x3 rotation matrix R_wt for a wall-mounted AprilTag.

    R_wt columns are the tag's local X, Y, Z axes in the world frame
    (pupil_apriltags convention).

    AprilTag local axes:
        +X  -> rightward on the tag face
        +Y  -> upward on the tag face
        +Z  -> out of the tag face (toward the camera)

    facing_angle_deg:
        Direction in world XY that the tag FACE points toward (deg CCW from +X).
          0   -> west wall,  facing east  (+X)
         90   -> south wall, facing north (+Y)
        180   -> east wall,  facing west  (-X)
        270   -> north wall, facing south (-Y)
    """
    phi = np.radians(facing_angle_deg)
    tag_z = np.array([ np.cos(phi),  np.sin(phi), 0.0])   # face normal
    tag_x = np.array([-np.sin(phi),  np.cos(phi), 0.0])   # right on face
    tag_y = np.array([ 0.0,          0.0,          1.0])   # up = world +Z
    return np.column_stack([tag_x, tag_y, tag_z])


def tag_pose(dx: float, dy: float, facing_angle_deg: float,
             height_m: float = TAG_HEIGHT_M) -> dict:
    """
    Convenience: define a tag world pose as an offset from tag 1 (origin).

    dx, dy           : position offset from tag 1 in metres (world X / Y).
    facing_angle_deg : direction the tag face points (see wall_tag_rotation).
    height_m         : tag centre height above the floor in metres.
    """
    return {
        "position": np.array([dx, dy, height_m]),
        "rotation": wall_tag_rotation(facing_angle_deg),
    }


# ======================================================================
# AprilTag World Pose Configuration
# ======================================================================
# Tag 1 is the world origin: position = (0, 0, TAG_HEIGHT_M).
# All other tags are (dx, dy) offsets from tag 1.
#
# facing_angle_deg:
#   0   -> tag faces +X (EAST  -- tag is on the west  wall)
#  90   -> tag faces +Y (NORTH -- tag is on the south wall)
# 180   -> tag faces -X (WEST  -- tag is on the east  wall)
# 270   -> tag faces -Y (SOUTH -- tag is on the north wall)
#
# TODO: measure your physical tag positions/facing directions and update.
# ======================================================================
TAG_WORLD_POSES = {
    # Tag 1 -- WORLD ORIGIN.  West wall, facing east (0 deg).
    1: tag_pose(dx=0.0, dy=0.0, facing_angle_deg=0.0),

    # West wall (same wall as tag 1)
    2: tag_pose(dx=0.0, dy=1.5, facing_angle_deg=0.0),

    # North wall (facing south = 270 deg)
    3: tag_pose(dx=1.0, dy=3.0, facing_angle_deg=270.0),
    4: tag_pose(dx=3.0, dy=3.0, facing_angle_deg=270.0),

    # East wall (facing west = 180 deg)
    5: tag_pose(dx=4.0, dy=1.5, facing_angle_deg=180.0),

    # South wall (facing north = 90 deg)
    6: tag_pose(dx=2.0, dy=0.0, facing_angle_deg=90.0),
}


# ======================================================================
# Camera -> Robot extrinsic
# ======================================================================
# T_CAM_ROBOT: 4x4 rigid transform mapping a point from the CAMERA
# frame to the ROBOT BASE frame.
#
#   p_robot = T_CAM_ROBOT[:3,:3] @ p_cam + T_CAM_ROBOT[:3,3]
#
# Default: camera centred over robot base, 0.20 m above robot origin,
# with identity rotation (camera and robot share the same yaw).
# Adjust translation (and rotation if camera is tilted/yawed).
# ======================================================================
T_CAM_ROBOT: np.ndarray = np.array([
    [1.0, 0.0, 0.0,  0.00],   # no lateral offset
    [0.0, 1.0, 0.0,  0.00],   # no forward offset
    [0.0, 0.0, 1.0,  0.20],   # camera 0.20 m above robot origin
    [0.0, 0.0, 0.0,  1.00],
], dtype=float)


# ======================================================================
# Core functions
# ======================================================================

def localize_camera(detections, tag_world_poses: dict) -> tuple:
    """
    Estimate the camera world-frame pose from a list of AprilTag detections.

    For each detection whose tag_id appears in tag_world_poses, the camera
    position and orientation in world frame are computed from the tag-relative
    pose provided by pupil_apriltags.  Results are weighted-averaged by each
    tag's decision_margin score.

    Parameters
    ----------
    detections      : list of pupil_apriltags.Detection objects
    tag_world_poses : mapping  tag_id -> {"position": [3], "rotation": 3x3}

    Returns
    -------
    cam_pos : np.ndarray [3]   -- camera position in world frame, or None
    R_wc    : np.ndarray [3,3] -- camera orientation in world (columns =
              camera +X/+Y/+Z in world), or None
    n_used  : int              -- number of tags that contributed
    """
    positions = []
    rotations = []
    weights   = []

    for det in detections:
        tag_id = det.tag_id
        if tag_id not in tag_world_poses:
            continue
        if det.pose_R is None or det.pose_t is None:
            continue

        p_wt = tag_world_poses[tag_id]["position"]         # (3,) world pos
        R_wt = tag_world_poses[tag_id]["rotation"]         # (3,3) world rot

        R_ct = np.array(det.pose_R, dtype=float)           # (3,3) tag->cam
        t_ct = np.array(det.pose_t, dtype=float).ravel()   # (3,)  tag in cam

        # Camera position in world:
        #   tag origin in cam frame = t_ct
        #   cam origin in tag frame = -R_ct.T @ t_ct
        #   cam origin in world     = p_wt + R_wt @ (-R_ct.T @ t_ct)
        cam_pos = p_wt + R_wt @ (-R_ct.T @ t_ct)

        # Camera orientation in world:
        #   R_ct : tag frame -> cam frame
        #   R_ct.T : cam frame -> tag frame
        #   R_wt @ R_ct.T : cam frame -> world frame
        R_wc = R_wt @ R_ct.T

        # Re-orthogonalize via SVD to suppress numerical drift
        U, _, Vt = np.linalg.svd(R_wc)
        R_wc = U @ Vt

        positions.append(cam_pos)
        rotations.append(R_wc)
        weights.append(float(det.decision_margin))

    if not positions:
        return None, None, 0

    w = np.array(weights, dtype=float)
    w /= w.sum()

    # Weighted mean position
    cam_pos_avg = sum(wi * p for wi, p in zip(w, positions))

    # Weighted mean rotation (average matrices, then re-orthogonalize)
    R_avg = sum(wi * R for wi, R in zip(w, rotations))
    U, _, Vt = np.linalg.svd(R_avg)
    R_wc_avg = U @ Vt

    return cam_pos_avg, R_wc_avg, len(positions)


def robot_pose_from_camera(
    cam_pos: np.ndarray,
    R_wc: np.ndarray,
    T_cam_robot: np.ndarray = None,
) -> tuple:
    """
    Convert a camera world pose to a 2D robot floor pose.

    Parameters
    ----------
    cam_pos      : camera position in world frame [3]
    R_wc         : 3x3 camera orientation in world
    T_cam_robot  : 4x4 camera-in-robot extrinsic.  Defaults to T_CAM_ROBOT.

    Returns
    -------
    (robot_x, robot_y, robot_yaw)
      x, y  : metres in world frame
      yaw   : heading in radians, wrapped to (-pi, pi]
    """
    if T_cam_robot is None:
        T_cam_robot = T_CAM_ROBOT

    R_rc = T_cam_robot[:3, :3]
    t_rc = T_cam_robot[:3, 3]   # camera position in robot frame

    # Robot origin in camera frame (inverse of T_cam_robot applied to origin)
    p_robot_in_cam = -R_rc.T @ t_rc

    # Robot position in world (floor projection -- drop Z)
    robot_pos = cam_pos + R_wc @ p_robot_in_cam
    robot_x   = float(robot_pos[0])
    robot_y   = float(robot_pos[1])

    # Robot heading: camera +Z in world, projected to XY plane.
    # Valid when camera yaw equals robot yaw (camera not rotated about Z).
    cam_fwd_world = R_wc[:, 2]
    robot_yaw = math.atan2(float(cam_fwd_world[1]), float(cam_fwd_world[0]))

    return robot_x, robot_y, robot_yaw
