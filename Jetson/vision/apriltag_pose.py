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
import sys
from pathlib import Path

import cv2
import numpy as np

# Ensure repo root is on sys.path so `from Jetson.config import` works
# whether this module is imported directly or as part of the package.
_here = Path(__file__).resolve().parent
_repo_root = next(
    (p for p in [_here, _here.parent, _here.parent.parent, _here.parent.parent.parent]
     if (p / "Jetson" / "__init__.py").exists()),
    _here.parent.parent,  # fallback
)
if str(_repo_root) not in sys.path:
    sys.path.insert(0, str(_repo_root))

from Jetson.config import (
    TAG_FAMILY, TAG_SIZE_M, TAG_HEIGHT_M,
    T_CAM_ROBOT,
    wall_tag_rotation, tag_pose, TAG_WORLD_POSES,
)


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
