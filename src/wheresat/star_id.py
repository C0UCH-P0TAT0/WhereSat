import numpy as np
from scipy.spatial import cKDTree
import itertools

def pixels_to_vectors(centroids: np.ndarray, camera_width: int, camera_fov: float) -> np.ndarray:
    """
    Step 1: Vector Projection
    Reverses the pinhole camera model to convert 2D pixels into 3D Unit Vectors.
    """
    if len(centroids) == 0:
        return np.array([])

    fov_rad = np.radians(camera_fov)
    focal_length = (camera_width / 2) / np.tan(fov_rad / 2)

    # Shift origin to the dead-center of the camera lens
    x_centered = centroids[:, 0] - (camera_width / 2.0)
    y_centered = centroids[:, 1] - (camera_width / 2.0)

    # Create 3D vectors and normalize them to a length of exactly 1
    vectors = np.column_stack((x_centered, y_centered, np.full_like(x_centered, focal_length)))
    magnitudes = np.linalg.norm(vectors, axis=1, keepdims=True)
    
    return vectors / magnitudes

def calculate_triangle_fingerprint(v1: np.ndarray, v2: np.ndarray, v3: np.ndarray) -> np.ndarray:
    """
    Step 2: Triangle Generation
    Calculates the angular distances between three stars and sorts them.
    """
    # Use dot product to find the angle between the vectors. 
    # np.clip prevents floating-point math errors from crashing the arccos function.
    dot_12 = np.clip(np.dot(v1, v2), -1.0, 1.0)
    dot_23 = np.clip(np.dot(v2, v3), -1.0, 1.0)
    dot_31 = np.clip(np.dot(v3, v1), -1.0, 1.0)
    
    theta_12 = np.arccos(dot_12)
    theta_23 = np.arccos(dot_23)
    theta_31 = np.arccos(dot_31)
    
    # Sort the distances (Shortest, Medium, Longest) to make it rotationally invariant
    return np.sort([theta_12, theta_23, theta_31])

def identify_stars(
    centroids: np.ndarray, 
    camera_width: int, 
    camera_fov: float, 
    kd_tree: cKDTree, 
    triangle_id_map: np.ndarray,
    tolerance: float = 1e-4
) -> np.ndarray:
    
    vectors = pixels_to_vectors(centroids, camera_width, camera_fov)
    num_stars = len(vectors)
    
    if num_stars < 3:
        return np.array([]) 
        
    # 1. Initialize an empty set to prevent duplicate ID logging
    identified_unique_stars = set()
        
    for indices in itertools.combinations(range(num_stars), 3):
        v1 = vectors[indices[0]]
        v2 = vectors[indices[1]]
        v3 = vectors[indices[2]]
        
        fingerprint = calculate_triangle_fingerprint(v1, v2, v3)
        distance, tree_idx = kd_tree.query(fingerprint, k=1)
        
        if distance < tolerance:
            matched_ids = triangle_id_map[tree_idx]
            
            # 2. Accumulate the hits. DO NOT RETURN EARLY.
            identified_unique_stars.update(matched_ids)
            
    # 3. Cast the final set back to an array after checking the whole image
    return np.array(list(identified_unique_stars))