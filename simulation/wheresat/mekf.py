import numpy as np
from scipy.spatial.transform import Rotation as R

class MEKF:
    def __init__(self, initial_q: np.ndarray, initial_cov: float = 0.1):
        """
        6-State Multiplicative Extended Kalman Filter.
        Estimates both Attitude (Quaternion) and Gyroscope Drift (Bias).
        """
        self.q = initial_q / np.linalg.norm(initial_q)
        
        # New State: The estimated Gyroscope Bias (starts at 0)
        self.beta = np.zeros(3) 
        
        # 6x6 Covariance Matrix
        # [Top Left 3x3]: Attitude Error
        # [Bottom Right 3x3]: Gyro Bias Error
        self.P = np.eye(6) * initial_cov
        self.P[3:, 3:] = np.eye(3) * 0.01 # Initial bias uncertainty
        
        # Process Noise 
        self.sigma_v = 1e-4  # Gyroscope white noise
        self.sigma_u = 3e-6  # Bias random walk (how fast thermal drift changes)
        
    def predict(self, omega_meas: np.ndarray, dt: float):
        """ Propagates the state forward using bias-corrected Gyroscope rates. """
        # 1. Correct the raw hardware reading using our internal bias estimate
        omega_est = omega_meas - self.beta
        
        # 2. Quaternion Kinematics
        omega_norm = np.linalg.norm(omega_est)
        if omega_norm > 1e-8:
            dq = R.from_rotvec((omega_est / omega_norm) * (omega_norm * dt)).as_quat()
            self.q = (R.from_quat(self.q) * R.from_quat(dq)).as_quat()
            
        # 3. State Transition Matrix (Phi) - 6x6
        # Calculates how current errors propagate into future errors
        wx, wy, wz = omega_est
        Omega_cross = np.array([
            [0, -wz, wy],
            [wz, 0, -wx],
            [-wy, wx, 0]
        ])
        
        Phi = np.eye(6)
        Phi[0:3, 0:3] = np.eye(3) - (Omega_cross * dt)
        Phi[0:3, 3:6] = -np.eye(3) * dt # Gyro bias directly degrades attitude over time
        
        # 4. Process Noise Covariance (Q) - 6x6
        Q = np.zeros((6, 6))
        Q[0:3, 0:3] = np.eye(3) * (self.sigma_v**2) * dt
        Q[3:6, 3:6] = np.eye(3) * (self.sigma_u**2) * dt
        
        # 5. Propagate Covariance
        self.P = Phi @ self.P @ Phi.T + Q
        
    def update(self, q_meas: np.ndarray, R_noise: np.ndarray):
        """ Corrects attitude and deduces gyro bias using Star Tracker data. """
        # 1. Error Quaternion
        q_err = (R.from_quat(self.q).inv() * R.from_quat(q_meas)).as_quat()
        if q_err[3] < 0:
            q_err = -q_err
        error_vector = 2.0 * q_err[:3]
        
        # 2. Measurement Matrix (H) - 3x6
        # We only have a camera, so we only directly measure attitude (first 3 states)
        H = np.zeros((3, 6))
        H[0:3, 0:3] = np.eye(3)
        
        # 3. Kalman Gain (6x3)
        S = H @ self.P @ H.T + R_noise
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # 4. Calculate the 6D State Correction
        correction = K @ error_vector
        delta_theta = correction[0:3] # Attitude correction
        delta_beta = correction[3:6]  # Bias correction (Learned from the cross-covariance)
        
        # 5. Apply Updates
        delta_q = np.array([delta_theta[0]/2, delta_theta[1]/2, delta_theta[2]/2, 1.0])
        delta_q /= np.linalg.norm(delta_q)
        self.q = (R.from_quat(self.q) * R.from_quat(delta_q)).as_quat()
        
        self.beta += delta_beta # Update internal bias model
        
        # 6. Update Covariance
        I_KH = np.eye(6) - (K @ H)
        self.P = I_KH @ self.P @ I_KH.T + (K @ R_noise @ K.T)

        return self.q