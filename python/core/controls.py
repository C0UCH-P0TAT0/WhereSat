import numpy as np

def compute_command_torque(
    q_current: np.ndarray, 
    q_target: np.ndarray, 
    omega: np.ndarray, 
    Kp: float, 
    Kd: float
) -> np.ndarray:
    """
    Calculates the 3-axis command torque required to drive the satellite 
    to a target orientation and kill its angular velocity.
    """
    
    # 1. Calculate the Inverse (Conjugate) of the Current Quaternion
    # Reversing the vector components effectively reverses the rotation
    q_inv = np.array([-q_current[0], -q_current[1], -q_current[2], q_current[3]])
    
    # 2. Quaternion Multiplication (Hamilton Product): q_error = q_current_inv ⊗ q_target
    # Breaking out the components [x, y, z, w] for the raw algebraic expansion
    x1, y1, z1, w1 = q_inv
    x2, y2, z2, w2 = q_target
    
    q_error = np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,  # Error X
        w1*y2 - x1*z2 + y1*w2 + z1*x2,  # Error Y
        w1*z2 + x1*y2 - y1*x2 + z1*w2,  # Error Z
        w1*w2 - x1*x2 - y1*y2 - z1*z2   # Error W (Scalar Angle)
    ])
    
    # 3. Extract the Vector and Scalar parts
    q_ev = q_error[0:3]
    q_ew = q_error[3]
    
    # 4. The Shortest Path Trap
    # If the scalar (W) is negative, the math is attempting a >180 degree rotation.
    # We multiply the vector part by -1 to force the hardware to take the shortest route.
    if q_ew < 0.0:
        q_ev = -1.0 * q_ev
        
    # 5. The PD Control Law
    # Proportional (Kp) fights the orientation error (The Spring)
    # Derivative (Kd) fights the angular velocity (The Friction)
    torque_cmd = (Kp * q_ev) - (Kd * omega)
    
    # 6. Hardware Saturation
    # A standard CubeSat reaction wheel maxes out around 0.02 Nm.
    # We strictly clip the request to prevent software from asking for impossible power.
    max_torque = 0.02
    torque_cmd = np.clip(torque_cmd, -max_torque, max_torque)
    
    return torque_cmd