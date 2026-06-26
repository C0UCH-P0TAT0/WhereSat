import numpy as np

def compute_attitude_quest(ref_vectors: np.ndarray, body_vectors: np.ndarray, weights: np.ndarray = None) -> np.ndarray:
    """
    Solves Wahba's Problem using Davenport's q-Method to calculate the optimal 
    [x, y, z, w] quaternion from a set of noisy vector observations.
    
    Args:
        ref_vectors: Nx3 array of absolute unit vectors in the ECI catalog frame.
        body_vectors: Nx3 array of corresponding unit vectors measured by the sensor.
        weights: Optional Nx1 array of statistical confidence weights per star.
        
    Returns:
        A 1x4 NumPy array representing the optimal quaternion [x, y, z, w].
    """
    num_vectors = len(ref_vectors)
    
    # Mathematical floor: Attitude cannot be resolved with fewer than 2 vectors.
    # We return the identity quaternion (no rotation) to prevent system crashes.
    if num_vectors < 2:
        return np.array([0.0, 0.0, 0.0, 1.0])
        
    # If no confidence weights are provided, we assume all Guide Stars are equally valid.
    if weights is None:
        weights = np.ones(num_vectors) / num_vectors
        
    # 1. The Attitude Profile Matrix (B)
    # This matrix accumulates the outer products of every single star match.
    # It acts as a statistical gravitational well, pulling the math toward the true orientation.
    B = np.zeros((3, 3))
    for i in range(num_vectors):
        # THE HAMILTON FIX:
        # Swapping the outer product forces the K-Matrix to yield a right-handed quaternion
        B += weights[i] * np.outer(ref_vectors[i], body_vectors[i])
        
    # 2. Extract Matrix Components
    S = B + B.T
    sigma = np.trace(B)
    Z = np.array([
        B[1, 2] - B[2, 1], 
        B[2, 0] - B[0, 2], 
        B[0, 1] - B[1, 0]
    ])
    
    # 3. Construct Davenport's K-Matrix (4x4)
    # This transforms the non-linear trig problem into a linear eigenvalue problem.
    K = np.empty((4, 4))
    K[:3, :3] = S - (sigma * np.eye(3))
    K[:3, 3] = Z
    K[3, :3] = Z
    K[3, 3] = sigma
    
    # 4. The Eigenvalue Solution
    # The eigenvector corresponding to the MAXIMUM eigenvalue of the K-Matrix 
    # is mathematically proven to be the optimal rotation quaternion.
    eigenvalues, eigenvectors = np.linalg.eigh(K)
    
    # np.linalg.eigh returns eigenvalues in ascending order. 
    # The last column is the eigenvector for the maximum eigenvalue.
    optimal_quaternion = eigenvectors[:, -1]
    
    # Returns [x, y, z, w] format, ready to be ingested by SciPy or flight controllers
    return optimal_quaternion