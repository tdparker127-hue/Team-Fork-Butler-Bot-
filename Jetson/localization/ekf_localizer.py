"""
ekf_localizer.py — Extended Kalman Filter for 2D robot localization.

State vector: [x, y, theta, vx_b, vy_b, omega]
  x, y     — robot position in the world frame [m]
  theta    — robot heading in the world frame [rad], CCW positive from world +X
  vx_b     — body-frame forward velocity [m/s]
  vy_b     — body-frame lateral velocity [m/s] (left positive)
  omega    — body-frame angular velocity [rad/s] (CCW positive)

Four sensor sources:
  1. IMU yaw rate from BNO08x (predict step — gyro angular velocity)
  2. Wheel encoder odometry (update step — mecanum kinematics → body velocities)
  3. IMU absolute yaw from BNO08x (heading-only update; accel NOT used)
  4. AprilTag camera pose from apriltag_pose.localize_camera()
     + robot_pose_from_camera()

Mahalanobis distance gating is applied to all measurement updates to reject
outliers.  When multiple tags are visible the AprilTag position/heading noise
covariance is scaled by 1/n_tags so more tags give tighter updates.

Usage (from mission_controller or standalone):
    from Jetson.localization.ekf_localizer import EKFLocalizer

    ekf = EKFLocalizer()
    ekf.predict(yaw_rate, dt)            # every control loop step (IMU gyro)
    ekf.update_encoder(wheel_vels)       # whenever new encoder data is available
    ekf.update_imu(yaw_rad)             # whenever new IMU yaw is available
    ekf.update_apriltag(x, y, yaw, n)   # whenever camera localizes
    x, y, theta, P = ekf.get_pose()
    vx_b, vy_b, omega, P_vel = ekf.get_twist()
"""

import enum
import math
import threading
import time
import numpy as np

from Jetson.config import (
    WHEEL_R, L_X, L_Y, MOTOR_SIGNS,
    SIGMA_XY, SIGMA_THETA, SIGMA_VXY, SIGMA_OMEGA_PROC,
    SIGMA_IMU_YAW,
    SIGMA_TAG_XY, SIGMA_TAG_YAW,
    SIGMA_ENC_VXY, SIGMA_ENC_OMEGA,
    MAHAL_THRESH_IMU, MAHAL_THRESH_TAG, MAHAL_THRESH_ENC,
    EUCLID_THRESH_IMU_RAD, EUCLID_THRESH_TAG_POS_M, EUCLID_THRESH_TAG_YAW_RAD,
    EUCLID_THRESH_ENC_VXY_M_S, EUCLID_THRESH_ENC_OMEGA_RAD_S,
)

# -- Initial state covariance ------------------------------------------------
INIT_P_XY    = 1.0    # [m²]   — large uncertainty at startup
INIT_P_THETA = (math.pi ** 2)  # [rad²] — full 360° uncertainty
INIT_P_VEL   = 1.0    # [(m/s)² or (rad/s)²] — velocity uncertainty at startup


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

        # State mean: [x, y, theta, vx_b, vy_b, omega]
        self._x = np.array([x0, y0, theta0, 0.0, 0.0, 0.0], dtype=float)

        # State covariance (6×6)
        self._P = np.diag([
            INIT_P_XY, INIT_P_XY, INIT_P_THETA,
            INIT_P_VEL, INIT_P_VEL, INIT_P_VEL,
        ])

        self._last_predict_t = time.monotonic()

    # -----------------------------------------------------------------------
    # Predict step — IMU yaw rate drives heading; velocity states drive position
    # -----------------------------------------------------------------------

    def predict(self, yaw_rate_imu: float, dt: float) -> None:
        """
        Propagate state forward using IMU gyro yaw rate.

        Position is integrated from the current body-frame velocity estimates.
        Heading is propagated using the IMU angular rate (yawRate field from
        BNO08x).  Body-frame velocities are held constant (constant-velocity
        model) — they are corrected by update_encoder().

        Parameters
        ----------
        yaw_rate_imu : float
            IMU angular velocity about the vertical axis [rad/s], CCW positive.
            Use the yawRate field from get_imu("drive").  Negate if the IMU
            convention is CW-positive.
        dt : float
            Time step in seconds.
        """
        if dt <= 0:
            return

        with self._lock:
            px, py, th, vx_b, vy_b, om = self._x
            cos_th, sin_th = math.cos(th), math.sin(th)

            # State transition
            new_px = px + (vx_b * cos_th - vy_b * sin_th) * dt
            new_py = py + (vx_b * sin_th + vy_b * cos_th) * dt
            new_th = _wrap_pi(th + yaw_rate_imu * dt)
            # vx_b, vy_b, om unchanged (constant-velocity model)

            self._x[0] = new_px
            self._x[1] = new_py
            self._x[2] = new_th

            # Jacobian F (6×6) of state transition w.r.t. state
            F = np.eye(6)
            # ∂px/∂th, ∂px/∂vx_b, ∂px/∂vy_b
            F[0, 2] = (-vx_b * sin_th - vy_b * cos_th) * dt
            F[0, 3] =  cos_th * dt
            F[0, 4] = -sin_th * dt
            # ∂py/∂th, ∂py/∂vx_b, ∂py/∂vy_b
            F[1, 2] = ( vx_b * cos_th - vy_b * sin_th) * dt
            F[1, 3] =  sin_th * dt
            F[1, 4] =  cos_th * dt

            # Process noise Q (6×6 diagonal)
            sxy  = SIGMA_XY         * dt
            sth  = SIGMA_THETA      * dt
            svxy = SIGMA_VXY        * dt
            som  = SIGMA_OMEGA_PROC * dt
           # In predict():
            Q = np.diag([
                SIGMA_XY**2         * dt,
                SIGMA_XY**2         * dt,
                SIGMA_THETA**2      * dt,
                SIGMA_VXY**2        * dt,
                SIGMA_VXY**2        * dt,
                SIGMA_OMEGA_PROC**2 * dt,
            ])

            self._P = F @ self._P @ F.T + Q

    # -----------------------------------------------------------------------
    # Encoder update — body-frame velocity correction
    # -----------------------------------------------------------------------

    def update_encoder(self, wheel_vels) -> bool:
        """
        Correct the body-frame velocity states using mecanum wheel odometry.

        Runs mecanum forward kinematics on the four wheel velocities to produce
        a 3-element body-twist measurement [vx_b, vy_b, omega], then applies a
        standard EKF measurement update against the velocity sub-state.

        Parameters
        ----------
        wheel_vels : array-like, length 4
            Wheel angular velocities in rad/s ordered [FrLft, BkLft, FrRgt, BkRgt].

        Returns True if the update was applied, False if gated out.
        """
        vels = MOTOR_SIGNS * np.asarray(wheel_vels, dtype=float)
        fl, bl, fr, br = vels

        # Mecanum forward kinematics → body twist
        vx_meas  = WHEEL_R / 4.0 * ( fl + bl + fr + br)
        vy_meas  = WHEEL_R / 4.0 * (-fl + bl + fr - br)
        om_meas  = WHEEL_R / (4.0 * (L_X + L_Y)) * (-fl - bl + fr + br)

        z = np.array([vx_meas, vy_meas, om_meas])

        # H selects [vx_b, vy_b, omega] from [x, y, theta, vx_b, vy_b, omega]
        H = np.zeros((3, 6))
        H[0, 3] = 1.0
        H[1, 4] = 1.0
        H[2, 5] = 1.0

        R_enc = np.diag([SIGMA_ENC_VXY**2, SIGMA_ENC_VXY**2, SIGMA_ENC_OMEGA**2])

        with self._lock:
            innovation = z - H @ self._x

            if self._gating is GatingMethod.EUCLIDEAN:
                vel_err = math.hypot(innovation[0], innovation[1])
                if (vel_err > EUCLID_THRESH_ENC_VXY_M_S or
                        abs(innovation[2]) > EUCLID_THRESH_ENC_OMEGA_RAD_S):
                    return False
                S_inv = None
            else:  # MAHALANOBIS
                S = H @ self._P @ H.T + R_enc
                try:
                    S_inv = np.linalg.inv(S)
                except np.linalg.LinAlgError:
                    return False
                if float(innovation @ S_inv @ innovation) > MAHAL_THRESH_ENC:
                    return False

            if S_inv is None:
                S     = H @ self._P @ H.T + R_enc
                S_inv = np.linalg.inv(S)

            K = self._P @ H.T @ S_inv

            self._x += K @ innovation
            self._x[2] = _wrap_pi(self._x[2])
            self._P = (np.eye(6) - K @ H) @ self._P

        return True

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
        H = np.array([[0.0, 0.0, 1.0, 0.0, 0.0, 0.0]])

        with self._lock:
            # Innovation (wrap to [-π, π])
            innovation = _wrap_pi(yaw - float((H @ self._x)[0]))

            if self._gating is GatingMethod.EUCLIDEAN:
                if abs(innovation) > EUCLID_THRESH_IMU_RAD:
                    return False
            else:  # MAHALANOBIS
                S_val = float((H @ self._P @ H.T + R_imu).item())
                if innovation ** 2 / max(S_val, 1e-12) > MAHAL_THRESH_IMU:
                    return False

            # Innovation covariance and Kalman gain
            S = H @ self._P @ H.T + R_imu
            K = self._P @ H.T @ np.linalg.inv(S)

            self._x += (K * innovation).ravel()
            self._x[2] = _wrap_pi(self._x[2])
            self._P = (np.eye(6) - K @ H) @ self._P

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
        # H selects [x, y, theta] from [x, y, theta, vx_b, vy_b, omega]
        H = np.zeros((3, 6))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0
        z = np.array([robot_x, robot_y, robot_yaw])

        with self._lock:
            # Innovation with yaw wrap
            innovation = z - H @ self._x
            innovation[2] = _wrap_pi(innovation[2])

            if self._gating is GatingMethod.EUCLIDEAN:
                pos_err = math.hypot(innovation[0], innovation[1])
                if (pos_err > EUCLID_THRESH_TAG_POS_M or
                        abs(innovation[2]) > EUCLID_THRESH_TAG_YAW_RAD):
                    return False
                S_inv = None
            else:  # MAHALANOBIS
                S = H @ self._P @ H.T + R_tag
                try:
                    S_inv = np.linalg.inv(S)
                except np.linalg.LinAlgError:
                    return False
                if float(innovation @ S_inv @ innovation) > MAHAL_THRESH_TAG:
                    return False

            if S_inv is None:
                S     = H @ self._P @ H.T + R_tag
                S_inv = np.linalg.inv(S)

            K = self._P @ H.T @ S_inv

            self._x += K @ innovation
            self._x[2] = _wrap_pi(self._x[2])
            self._P = (np.eye(6) - K @ H) @ self._P

        return True

    # -----------------------------------------------------------------------
    # State accessors
    # -----------------------------------------------------------------------

    def get_pose(self):
        """
        Return (x, y, theta, P) — a snapshot of the current pose state.

        x, y    : robot position [m]
        theta   : robot heading  [rad]
        P       : 3×3 position+heading covariance submatrix (copy)
                  Rows/cols correspond to [x, y, theta].
        """
        with self._lock:
            return (float(self._x[0]), float(self._x[1]), float(self._x[2]),
                    self._P[:3, :3].copy())

    def get_twist(self):
        """
        Return (vx_b, vy_b, omega, P_vel) — a snapshot of the velocity state.

        vx_b    : body-frame forward velocity  [m/s]
        vy_b    : body-frame lateral velocity  [m/s] (left positive)
        omega   : body-frame angular velocity  [rad/s] (CCW positive)
        P_vel   : 3×3 velocity covariance submatrix (copy)
                  Rows/cols correspond to [vx_b, vy_b, omega].
        """
        with self._lock:
            return (float(self._x[3]), float(self._x[4]), float(self._x[5]),
                    self._P[3:, 3:].copy())

    def set_pose(self, x: float, y: float, theta: float) -> None:
        """Force the pose state (e.g., at mission start or after relocalization).
        Velocity states are zeroed and all covariances are reset."""
        with self._lock:
            self._x[:] = [x, y, theta, 0.0, 0.0, 0.0]
            self._P = np.diag([
                INIT_P_XY, INIT_P_XY, INIT_P_THETA,
                INIT_P_VEL, INIT_P_VEL, INIT_P_VEL,
            ])
