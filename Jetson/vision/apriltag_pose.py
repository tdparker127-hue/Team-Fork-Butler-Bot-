import cv2
import numpy as np
import pupil_apriltags as apriltag

# ======================================================================
# AprilTag World Pose Configuration
# ======================================================================
# Specify the known pose of each AprilTag in the world frame by tag ID.
#
#   'position': np.array([x, y, z]) in metres — the tag centre in world.
#   'rotation': 3×3 numpy rotation matrix whose columns are the tag's
#               X, Y, Z axes expressed in world coordinates.
#               Use np.eye(3) when the tag frame is aligned with world.
#
# AprilTag local frame (pupil_apriltags convention):
#   +X → right,  +Y → up,  +Z → out of the tag face (toward the camera)
#
# Example: four tags lying flat on the floor in a 1 m grid, all facing up
#   (tag +Z = world +Z).  Adjust positions / rotations to match your setup.
# ======================================================================
TAG_WORLD_POSES = {
    0: {
        'position': np.array([0.00, 0.00, 0.00]),
        'rotation': np.eye(3),
    },
    1: {
        'position': np.array([1.00, 1.00, 0.00]),
        'rotation': np.eye(3),
    },
    2: {
        'position': np.array([1.00, 0.00, 0.00]),
        'rotation': np.eye(3),
    },
    3: {
        'position': np.array([0.00, 1.00, 0.00]),
        'rotation': np.eye(3),
    },
    # Add more tags here …
}


# ======================================================================
# Rotation helpers
# ======================================================================

def rotation_matrix_to_quaternion(R):
    """3×3 rotation matrix → unit quaternion [w, x, y, z]."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        return np.array([0.25 / s,
                         (R[2, 1] - R[1, 2]) * s,
                         (R[0, 2] - R[2, 0]) * s,
                         (R[1, 0] - R[0, 1]) * s])
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        return np.array([(R[2, 1] - R[1, 2]) / s, 0.25 * s,
                         (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s])
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        return np.array([(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s,
                         0.25 * s, (R[1, 2] + R[2, 1]) / s])
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        return np.array([(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
                         (R[1, 2] + R[2, 1]) / s, 0.25 * s])


def quaternion_to_rotation_matrix(q):
    """Unit quaternion [w, x, y, z] → 3×3 rotation matrix."""
    q = q / np.linalg.norm(q)
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def average_quaternions(quats, weights=None):
    """
    Weighted average of unit quaternions [w, x, y, z].
    All quaternions are flipped into the same hemisphere as quats[0]
    before accumulation to avoid cancellation artefacts.
    """
    if weights is None:
        weights = np.ones(len(quats))
    weights = np.array(weights, dtype=float)
    weights /= weights.sum()

    q_ref = quats[0]
    q_sum = np.zeros(4)
    for q, w in zip(quats, weights):
        if np.dot(q_ref, q) < 0:
            q = -q
        q_sum += w * q
    return q_sum / np.linalg.norm(q_sum)


def rotation_matrix_to_euler_deg(R):
    """
    XYZ Euler angles (roll, pitch, yaw) from a rotation matrix, in degrees.
    """
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    if sy > 1e-6:
        roll  = np.degrees(np.arctan2( R[2, 1], R[2, 2]))
        pitch = np.degrees(np.arctan2(-R[2, 0], sy))
        yaw   = np.degrees(np.arctan2( R[1, 0], R[0, 0]))
    else:
        roll  = np.degrees(np.arctan2(-R[1, 2], R[1, 1]))
        pitch = np.degrees(np.arctan2(-R[2, 0], sy))
        yaw   = 0.0
    return roll, pitch, yaw


# ======================================================================
# Camera localisation
# ======================================================================

def localize_camera(detections, tag_world_poses):
    """
    Estimate the camera position and orientation in the world frame.

    For each detected tag whose world pose is known, the camera pose is
    derived from:

        R_cw  = R_det @ R_w.T             # world → camera rotation
        cam_pos = T_w - R_w @ R_det.T @ t_det   # camera origin in world

    where
        R_det, t_det  — tag-to-camera rotation / translation from detector
        R_w, T_w      — known tag orientation / position in world frame

    Multi-tag averaging strategy
    ----------------------------
    Individual position estimates are combined as a weighted mean and
    orientations as a weighted quaternion average.  Weights are set to
    each tag's detection decision_margin (a proxy for confidence).

    Returns
    -------
    cam_pos   : (3,) camera position in world frame, or None
    R_wc      : (3,3) rotation that maps camera axes → world axes
    n_used    : int, number of known tags that contributed
    """
    positions = []
    quats     = []
    weights   = []

    for r in detections:
        if r.tag_id not in tag_world_poses:
            continue
        if r.pose_R is None or r.pose_t is None:
            continue

        T_w   = tag_world_poses[r.tag_id]['position']   # (3,)
        R_w   = tag_world_poses[r.tag_id]['rotation']   # (3,3) tag→world
        R_det = r.pose_R                                 # (3,3) tag→camera
        t_det = r.pose_t.ravel()                         # (3,)

        # World → camera rotation
        R_cw = R_det @ R_w.T

        # Camera origin expressed in world frame
        cam_pos = T_w - R_w @ R_det.T @ t_det

        positions.append(cam_pos)
        quats.append(rotation_matrix_to_quaternion(R_cw))
        weights.append(float(r.decision_margin))

    if not positions:
        return None, None, 0

    # Weighted position average
    w = np.array(weights) / np.sum(weights)
    cam_pos_avg = np.sum([wi * p for wi, p in zip(w, positions)], axis=0)

    # Weighted orientation average (via quaternions)
    R_cw_avg = quaternion_to_rotation_matrix(average_quaternions(quats, weights))
    R_wc_avg = R_cw_avg.T   # camera axes expressed in world coords

    return cam_pos_avg, R_wc_avg, len(positions)


def main():
    #----------------------------------------------------------------------
    # 1. Load camera calibration data
    #----------------------------------------------------------------------
    calibration_data = np.load('camera_calibration_live.npz')
    camera_matrix = calibration_data['camera_matrix']  # shape (3, 3)
    dist_coeffs   = calibration_data['dist_coeffs']    # shape (n,)

    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    #----------------------------------------------------------------------
    # 2. Set up the AprilTag detector
    #----------------------------------------------------------------------
    at_detector = apriltag.Detector(
        families='tag36h11',  # or 'tag25h9', etc.
        nthreads=1,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0
    )

    # The length of one side of the tag in meters
    TAG_SIZE = 0.10  # 10 cm

    #----------------------------------------------------------------------
    # 3. Initialize Webcam
    #----------------------------------------------------------------------
    # Force V4L2 backend and use the correct device path
    cap = cv2.VideoCapture("/dev/video4", cv2.CAP_V4L2)

    # Set YUYV — this is the ONLY format available on the color stream
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 15)  # max for 1280x720

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    print("Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read from the webcam.")
            break

        # Option A: Undistort the entire frame
##        h, w = frame.shape[:2]
##        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
##            camera_matrix, dist_coeffs, (w, h), 1, (w, h))
##        undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs, None, new_camera_matrix)

        undistorted = frame  # distortion correction disabled

        gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)

        #------------------------------------------------------------------
        # 4. Detect AprilTags
        #------------------------------------------------------------------
        results = at_detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=[fx, fy, cx, cy],
            tag_size=TAG_SIZE
        )

        #------------------------------------------------------------------
        # 5. Per-tag visualisation
        #------------------------------------------------------------------
        for r in results:
            tag_id = r.tag_id
            corners = r.corners.astype(int)
            known   = tag_id in TAG_WORLD_POSES

            # Green outline = tag has a known world pose; yellow = unknown
            outline_color = (0, 255, 0) if known else (0, 255, 255)
            for i in range(4):
                cv2.line(undistorted,
                         tuple(corners[i]), tuple(corners[(i + 1) % 4]),
                         outline_color, 2)

            center_xy = (int(r.center[0]), int(r.center[1]))
            label = f"ID:{tag_id}" + (" [W]" if known else "")
            cv2.putText(undistorted, label, center_xy,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            if r.pose_t is not None:
                t = r.pose_t
                cv2.putText(undistorted,
                            f"X:{t.item(0):.2f} Y:{t.item(1):.2f} Z:{t.item(2):.2f}",
                            corners[0], cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (0, 255, 255), 2)

        #------------------------------------------------------------------
        # 6. Camera localisation using all visible known tags
        #------------------------------------------------------------------
        cam_pos, R_wc, n_used = localize_camera(results, TAG_WORLD_POSES)

        if cam_pos is not None:
            roll, pitch, yaw = rotation_matrix_to_euler_deg(R_wc)
            print(f"Camera world pos (x,y,z) [m]: {cam_pos.round(3)}"
                  f"  |  yaw={yaw:.1f}°  pitch={pitch:.1f}°  roll={roll:.1f}°"
                  f"  |  tags used: {n_used}")

            # HUD overlay (drawn twice: thick white shadow + thin coloured text)
            lines = [
                f"Camera pos  ({n_used} tag{'s' if n_used > 1 else ''})",
                f"  X={cam_pos[0]:.3f}  Y={cam_pos[1]:.3f}  Z={cam_pos[2]:.3f} m",
                f"  Yaw={yaw:.1f}  Pitch={pitch:.1f}  Roll={roll:.1f} deg",
            ]
            y0 = 30
            for line in lines:
                cv2.putText(undistorted, line, (10, y0),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 3)
                cv2.putText(undistorted, line, (10, y0),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 120, 255), 1)
                y0 += 28
        else:
            cv2.putText(undistorted, "No known tags visible", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

        #------------------------------------------------------------------
        # 7. Show result
        #------------------------------------------------------------------
        cv2.imshow('AprilTag Localisation', undistorted)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

