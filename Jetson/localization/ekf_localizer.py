"""
ekf_localizer.py — Extended Kalman Filter for 2D robot localization.

State vector: [x, y, theta]
  x, y   — robot position in the world frame [m]
  theta  — robot heading in the world frame [rad], CCW positive from world +X

Three sensor update sources:
  1. Wheel encoder odometry (predict step — mecanum kinematics)
  2. IMU absolute yaw from BNO08x (heading-only update; accel NOT used)
  3. AprilTag camera pose from apriltag_pose.localize_camera()
     + robot_pose_from_camera()

Mahalanobis distance gating is applied to both IMU and AprilTag updates to
reject outliers.  When multiple tags are visible the AprilTag position/heading
noise covariance is scaled by 1/n_tags so more tags give tighter updates.

Usage (from mission_controller or standalone):
    from Jetson.localization.ekf_localizer import EKFLocalizer

    ekf = EKFLocalizer()
    ekf.predict(wheel_vels, dt)          # every control loop step
    ekf.update_imu(yaw_rad)             # whenever new IMU yaw is available
    ekf.update_apriltag(x, y, yaw, n)   # whenever camera localizes
    x, y, theta, P = ekf.get_pose()
"""

import enum
import math
import threading
import time
import numpy as np

from Jetson.config import (
    WHEEL_R, L_X, L_Y, MOTOR_SIGNS,
    SIGMA_XY, SIGMA_THETA, SIGMA_IMU_YAW,
    SIGMA_TAG_XY, SIGMA_TAG_YAW,
    MAHAL_THRESH_IMU, MAHAL_THRESH_TAG,
    EUCLID_THRESH_IMU_RAD, EUCLID_THRESH_TAG_POS_M, EUCLID_THRESH_TAG_YAW_RAD,
)

# -- Initial state covariance ------------------------------------------------
INIT_P_XY    = 1.0    # [m²]   — large uncertainty at startup
INIT_P_THETA = (math.pi ** 2)  # [rad²] — full 360° uncertainty


# ===========================================================================
# Gating method enum
# ===========================================================================

class GatingMethod(enum.Enum):
    """
    Innovation gating strategy applied before every EKF measurement update.

    MAHALANOBIS (default)
        d² = νᵀ S⁻¹ ν, where S = H P Hᵀ + R is the innovation covariance.
        Accounts for current filter uncertainty — a large innovation is
        accepted when P is large and rejected once the filter has converged.
        Threshold is a chi-squared bound (95 %) on the innovation dimension.

    EUCLIDEAN
        Gate on the raw L2 norm of the innovation vector, independent of P.
        Simpler and more predictable once the environment is well-known.
        Thresholds are the EUCLID_THRESH_* constants in config.py.
    """
    MAHALANOBIS = "mahalanobis"
    EUCLIDEAN   = "euclidean"


# ===========================================================================
# Helper
# ===========================================================================

def _wrap_pi(angle: float) -> float:
    """Wrap angle to (-π, π]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


# ===========================================================================
# EKFLocalizer
# ===========================================================================

class EKFLocalizer:
    """
    Thread-safe 2D EKF localizer.

    predict() and all update_*() methods can be called from different
    threads (e.g., a camera thread and a control loop thread).  A single
    re-entrant lock serializes all state mutations.
    """

    def __init__(self, x0: float = 0.0, y0: float = 0.0, theta0: float = 0.0,
                 gating: GatingMethod = GatingMethod.MAHALANOBIS):
        self._lock   = threading.Lock()
        self._gating = gating

        # State mean: [x, y, theta]
        self._x = np.array([x0, y0, theta0], dtype=float)

        # State covariance
        self._P = np.diag([INIT_P_XY, INIT_P_XY, INIT_P_THETA])

        self._last_predict_t = time.monotonic()

    # -----------------------------------------------------------------------
    # Predict step — mecanum wheel odometry
    # -----------------------------------------------------------------------

    def predict(self, wheel_vels, dt: float) -> None:
        """
        Propagate state forward using mecanum odometry.

        Parameters
        ----------
        wheel_vels : array-like, length 4
            Wheel angular velocities in rad/s ordered [FrLft, BkLft, FrRgt, BkRgt].
            Values must already be sign-corrected (positive = forward contribution).
        dt : float
            Time step in seconds.
        """
        if dt <= 0:
            return

        vels = MOTOR_SIGNS * np.asarray(wheel_vels, dtype=float)
        fl, bl, fr, br = vels

        # Mecanum forward kinematics
        vx_body  = WHEEL_R / 4.0 * ( fl + bl + fr + br)
        vy_body  = WHEEL_R / 4.0 * (-fl + bl + fr - br)
        omega    = WHEEL_R / (4.0 * (L_X + L_Y)) * (-fl - bl + fr + br)

        with self._lock:
            th = self._x[2]
            cos_th, sin_th = math.cos(th), math.sin(th)

            # State transition
            dx    = (vx_body * cos_th - vy_body * sin_th) * dt
            dy    = (vx_body * sin_th + vy_body * cos_th) * dt
            dtheta = omega * dt

            self._x[0] += dx
            self._x[1] += dy
            self._x[2]  = _wrap_pi(self._x[2] + dtheta)

            # Jacobian of state transition w.r.t. state
            F = np.eye(3)
            F[0, 2] = (-vx_body * sin_th - vy_body * cos_th) * dt
            F[1, 2] = ( vx_body * cos_th - vy_body * sin_th) * dt

            # Process noise covariance
            sigma_xy_step    = SIGMA_XY    * dt
            sigma_theta_step = SIGMA_THETA * dt
            Q = np.diag([sigma_xy_step**2, sigma_xy_step**2, sigma_theta_step**2])

            self._P = F @ self._P @ F.T + Q

    # -----------------------------------------------------------------------
    # IMU update — absolute yaw only
    # -----------------------------------------------------------------------

    def update_imu(self, yaw: float) -> bool:
        """
        Correct the heading estimate using the IMU's absolute yaw (radians).

        Returns True if the update was applied, False if gated out.
        The gating method is set at construction time (default: MAHALANOBIS).
        """
        R_imu = np.array([[SIGMA_IMU_YAW ** 2]])
        H = np.array([[0.0, 0.0, 1.0]])

        with self._lock:
            # Innovation (wrap to [-π, π])
            innovation = _wrap_pi(yaw - float(H @ self._x))

            if self._gating is GatingMethod.EUCLIDEAN:
                if abs(innovation) > EUCLID_THRESH_IMU_RAD:
                    return False
            else:  # MAHALANOBIS
                S_val = float(H @ self._P @ H.T + R_imu)
                if innovation ** 2 / max(S_val, 1e-12) > MAHAL_THRESH_IMU:
                    return False

            # Innovation covariance and Kalman gain
            S = H @ self._P @ H.T + R_imu
            K = self._P @ H.T @ np.linalg.inv(S)

            self._x += (K * innovation).ravel()
            self._x[2] = _wrap_pi(self._x[2])
            self._P = (np.eye(3) - K @ H) @ self._P

        return True

    # -----------------------------------------------------------------------
    # AprilTag update — full 2D pose
    # -----------------------------------------------------------------------

    def update_apriltag(self, robot_x: float, robot_y: float,
                        robot_yaw: float, n_tags: int = 1) -> bool:
        """
        Correct the full 2D pose using a camera-derived robot pose estimate.

        Parameters
        ----------
        robot_x, robot_y : float
            Robot position in world frame [m], from robot_pose_from_camera().
        robot_yaw : float
            Robot heading in world frame [rad].
        n_tags : int
            Number of AprilTags that contributed to this estimate.
            More tags → lower measurement noise.

        Returns True if the update was applied, False if gated out.
        """
        n = max(1, n_tags)
        R_tag = np.diag([
            (SIGMA_TAG_XY  / math.sqrt(n)) ** 2,
            (SIGMA_TAG_XY  / math.sqrt(n)) ** 2,
            (SIGMA_TAG_YAW / math.sqrt(n)) ** 2,
        ])
        H = np.eye(3)
        z = np.array([robot_x, robot_y, robot_yaw])

        with self._lock:
            # Innovation with yaw wrap
            innovation = z - self._x.copy()
            innovation[2] = _wrap_pi(innovation[2])

            if self._gating is GatingMethod.EUCLIDEAN:
                pos_err = math.hypot(innovation[0], innovation[1])
                if (pos_err > EUCLID_THRESH_TAG_POS_M or
                        abs(innovation[2]) > EUCLID_THRESH_TAG_YAW_RAD):
                    return False
                S_inv = None  # computed below only if we pass the gate
            else:  # MAHALANOBIS
                # Innovation covariance
                S = H @ self._P @ H.T + R_tag
                try:
                    S_inv = np.linalg.inv(S)
                except np.linalg.LinAlgError:
                    return False
                if float(innovation @ S_inv @ innovation) > MAHAL_THRESH_TAG:
                    return False

            # Re-compute S / S_inv for Kalman gain (Euclidean path skipped above)
            if S_inv is None:
                S     = H @ self._P @ H.T + R_tag
                S_inv = np.linalg.inv(S)

            K = self._P @ H.T @ S_inv

            self._x += K @ innovation
            self._x[2] = _wrap_pi(self._x[2])
            self._P = (np.eye(3) - K @ H) @ self._P

        return True

    # -----------------------------------------------------------------------
    # State accessors
    # -----------------------------------------------------------------------

    def get_pose(self):
        """
        Return (x, y, theta, P) — a snapshot of the current state.

        x, y    : robot position [m]
        theta   : robot heading  [rad]
        P       : 3×3 covariance matrix (copy)
        """
        with self._lock:
            return float(self._x[0]), float(self._x[1]), float(self._x[2]), self._P.copy()

    def set_pose(self, x: float, y: float, theta: float) -> None:
        """Force the state (e.g., at mission start or after relocalization)."""
        with self._lock:
            self._x[:] = [x, y, theta]
            self._P = np.diag([INIT_P_XY, INIT_P_XY, INIT_P_THETA])
