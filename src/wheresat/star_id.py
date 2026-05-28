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
    """
    Step 3: The Match Engine
    Takes unknown 2D centroids, builds triangles, and queries the database.
    
    Args:
        centroids: Nx2 array of [X, Y] sub-pixel coordinates.
        camera_width: The width of the sensor in pixels.
        camera_fov: The field of view in degrees.
        kd_tree: The pre-built SciPy KD-Tree containing all catalog triangles.
        triangle_id_map: An array linking KD-Tree indices to absolute Hipparcos IDs.
        tolerance: How strictly the angles must match (radians) to be considered a success.
        
    Returns:
        An array of Hipparcos IDs for the identified stars.
    """
    vectors = pixels_to_vectors(centroids, camera_width, camera_fov)
    num_stars = len(vectors)
    
    if num_stars < 3:
        # A triangle requires 3 stars. The satellite is blind.
        return np.array([]) 
        
    # Generate all possible 3-star combinations from the camera image
    for indices in itertools.combinations(range(num_stars), 3):
        v1 = vectors[indices[0]]
        v2 = vectors[indices[1]]
        v3 = vectors[indices[2]]
        
        # Get the [Short, Medium, Long] fingerprint for this specific triangle
        fingerprint = calculate_triangle_fingerprint(v1, v2, v3)
        
        # Query the K-D Tree to find the 1 absolute closest match in the universe
        distance, tree_idx = kd_tree.query(fingerprint, k=1)
        
        # If the math matches within our strict mathematical tolerance, it is a hit!
        if distance < tolerance:
            # Retrieve the true Hipparcos IDs using the tree index
            matched_ids = triangle_id_map[tree_idx]
            
            # Return the exact IDs of the stars the camera is looking at
            return np.array(matched_ids)
            
    return np.array([]) # No matches found in the entire image

