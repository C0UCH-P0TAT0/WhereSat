import numpy as np
from scipy.spatial.transform import Rotation as R

class MEKF:
    def __init__(self, initial_q: np.ndarray, initial_cov: float = 0.1):
        """
        Multiplicative Extended Kalman Filter for 4D Quaternion fusion.
        """
        # Ensure q is scalar-last [x, y, z, w]
        self.q = initial_q / np.linalg.norm(initial_q)
        
        # 3x3 Covariance Matrix (tracking error in 3D tangent space)
        self.P = np.eye(3) * initial_cov
        
        # Gyroscope Process Noise (How much we trust the gyro)
        self.Q = np.eye(3) * 1e-5
        
    def predict(self, omega: np.ndarray, dt: float):
        """
        Propagates the state forward using high-speed Gyroscope rates.
        """
        # 1. Quaternion Kinematics (Propagate the 4D state)
        omega_norm = np.linalg.norm(omega)
        if omega_norm > 1e-8:
            angle = omega_norm * dt
            axis = omega / omega_norm
            # Create a delta rotation quaternion
            dq = R.from_rotvec(axis * angle).as_quat()
            
            # Multiply current state by delta (Hamilton convention)
            r_current = R.from_quat(self.q)
            r_delta = R.from_quat(dq)
            self.q = (r_current * r_delta).as_quat()
            
        # 2. Covariance Propagation (Propagate the 3D error bounds)
        # For small dt, the state transition matrix Phi is approximated via Rodrigues
        wx, wy, wz = omega
        Omega_cross = np.array([
            [0, -wz, wy],
            [wz, 0, -wx],
            [-wy, wx, 0]
        ])
        Phi = np.eye(3) - (Omega_cross * dt)
        
        self.P = Phi @ self.P @ Phi.T + (self.Q * dt)
        
    def update(self, q_meas: np.ndarray, R_noise: np.ndarray):
        """
        Corrects the drifting gyro state using an absolute Star Tracker measurement.
        """
        # 1. Calculate Error Quaternion: q_err = q_pred^-1 * q_meas
        r_pred_inv = R.from_quat(self.q).inv()
        r_meas = R.from_quat(q_meas)
        q_err = (r_pred_inv * r_meas).as_quat() # [ex, ey, ez, ew]
        
        # Force shortest path (prevent 359 degree corrections)
        if q_err[3] < 0:
            q_err = -q_err
            
        # Extract the 3D error vector (approx 2 * vector component for small angles)
        error_vector = 2.0 * q_err[:3]
        
        # 2. Calculate Kalman Gain
        # H is the measurement matrix. Since we directly measure attitude, H is the Identity matrix.
        H = np.eye(3)
        S = H @ self.P @ H.T + R_noise
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # 3. Calculate the State Correction (delta theta)
        delta_theta = K @ error_vector
        
        # 4. Apply Multiplicative Update to Quaternion
        delta_q = np.array([
            delta_theta[0] / 2.0,
            delta_theta[1] / 2.0,
            delta_theta[2] / 2.0,
            1.0
        ])
        delta_q /= np.linalg.norm(delta_q) # Normalize the update
        
        r_current = R.from_quat(self.q)
        r_update = R.from_quat(delta_q)
        self.q = (r_current * r_update).as_quat()
        
        # 5. Update Covariance (Joseph form for numerical stability)
        I_KH = np.eye(3) - (K @ H)
        self.P = I_KH @ self.P @ I_KH.T + (K @ R_noise @ K.T)

        return self.q