import numpy as np
from scipy.spatial.transform import Rotation as R

def eci_to_body(eci_vector: np.ndarray, quaternion: np.ndarray) -> np.ndarray:
    """
    Rotates a vector from the Earth-Centered Inertial (ECI) frame to the Satellite Body frame.
    
    Args:
        eci_vector: 3D vector in the ECI frame (e.g., star coordinates).
        quaternion: The satellite's attitude quaternion in [x, y, z, w] format.
    
    Returns:
        The 3D vector transformed into the satellite's body frame.
    """
    # R.from_quat STRICTLY expects the [x, y, z, w] convention. 
    # If passed [w, x, y, z] from Astropy, this throws garbage.
    rot = R.from_quat(quaternion)
    
    # The quaternion defines the Body frame's orientation relative to ECI.
    # To bring an external ECI vector INTO the Body frame, we apply the inverse rotation.
    body_vector = rot.inv().apply(eci_vector)
    
    return body_vector

def body_to_eci(body_vector: np.ndarray, quaternion: np.ndarray) -> np.ndarray:
    """
    Rotates a vector from the Satellite Body frame back to the ECI frame.
    
    Args:
        body_vector: 3D vector in the Body frame (e.g., extracted from camera pixels).
        quaternion: The satellite's attitude quaternion in [x, y, z, w] format.
        
    Returns:
        The 3D vector transformed into the ECI frame.
    """
    rot = R.from_quat(quaternion)
    
    # We apply the forward rotation to take the camera's local vector and map it to absolute space.
    eci_vector = rot.apply(body_vector)
    
    return eci_vector