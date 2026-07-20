import numpy as np

def compute_attitude_triad(
    ref_vectors: np.ndarray, 
    body_vectors: np.ndarray
) -> np.ndarray:
    """
    Computes the 3x3 direction cosine matrix (DCM) rotating the ECI frame 
    to the satellite Body frame.
    """
    if len(ref_vectors) < 2 or len(body_vectors) < 2:
        return np.eye(3)

    R1, R2 = ref_vectors[0], ref_vectors[1]
    B1, B2 = body_vectors[0], body_vectors[1]

    # ---------------------------------------------------------
    # 1. Construct the ECI Reference Triad (M_R)
    # ---------------------------------------------------------
    u1 = R1 / np.linalg.norm(R1)
    
    r1_x_r2 = np.cross(R1, R2)
    norm_r = np.linalg.norm(r1_x_r2)
    
    if norm_r < 1e-6:
        return np.eye(3)
        
    u2 = r1_x_r2 / norm_r
    u3 = np.cross(u1, u2)
    
    M_R = np.column_stack((u1, u2, u3))

    # ---------------------------------------------------------
    # 2. Construct the Measured Body Triad (M_V)
    # ---------------------------------------------------------
    v1 = B1 / np.linalg.norm(B1)
    
    b1_x_b2 = np.cross(B1, B2)
    norm_b = np.linalg.norm(b1_x_b2)
    
    if norm_b < 1e-6:
        return np.eye(3)
        
    v2 = b1_x_b2 / norm_b
    v3 = np.cross(v1, v2)
    
    M_V = np.column_stack((v1, v2, v3))

    # ---------------------------------------------------------
    # 3. Calculate the Rotation Matrix (A)
    # ---------------------------------------------------------
    A = M_V @ M_R.T

    # ---------------------------------------------------------
    # 4. Aerospace Validation
    # ---------------------------------------------------------
    is_orthogonal = np.allclose(A @ A.T, np.eye(3), atol=1e-4)
    det_A = np.linalg.det(A)
    is_right_handed = np.isclose(det_A, 1.0, atol=1e-4)

    if not (is_orthogonal and is_right_handed):
        return np.eye(3)

    return A