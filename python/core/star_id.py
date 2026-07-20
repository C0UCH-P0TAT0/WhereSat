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
    dot_12 = np.clip(np.dot(v1, v2), -1.0, 1.0)
    dot_23 = np.clip(np.dot(v2, v3), -1.0, 1.0)
    dot_31 = np.clip(np.dot(v3, v1), -1.0, 1.0)
    
    theta_12 = np.arccos(dot_12)
    theta_23 = np.arccos(dot_23)
    theta_31 = np.arccos(dot_31)
    
    # Sort the distances (Shortest, Medium, Longest)
    return np.sort([theta_12, theta_23, theta_31])

def identify_stars(
    centroids: np.ndarray, 
    camera_width: int, 
    camera_fov: float, 
    kd_tree: cKDTree, 
    triangle_id_map: np.ndarray,
    catalog: np.ndarray,
    tolerance: float = 1e-4
):
    """
    Step 3: Database Matching & Geometric Vertex Alignment
    Returns two perfectly aligned Nx3 arrays for QUEST ingestion.
    """
    body_vectors = pixels_to_vectors(centroids, camera_width, camera_fov)
    num_stars = len(body_vectors)
    
    if num_stars < 3:
        return np.array([]), np.array([])
        
    # We use a dictionary to ensure we map each body vector exactly once, even if it belongs to multiple matching triangles.
    final_mapping = {}
        
    for indices in itertools.combinations(range(num_stars), 3):
        idx1, idx2, idx3 = indices
        v1, v2, v3 = body_vectors[idx1], body_vectors[idx2], body_vectors[idx3]
        
        fingerprint = calculate_triangle_fingerprint(v1, v2, v3)
        distance, tree_idx = kd_tree.query(fingerprint, k=1)
        
        if distance < tolerance:
            cat_ids = triangle_id_map[tree_idx]
            
            # Fetch absolute ECI vectors from the catalog
            e1 = catalog[catalog[:, 0] == cat_ids[0]][0, 1:4]
            e2 = catalog[catalog[:, 0] == cat_ids[1]][0, 1:4]
            e3 = catalog[catalog[:, 0] == cat_ids[2]][0, 1:4]
            
            # Calculate internal edge lengths for the Camera Body Triangle
            b_edges = [
                np.linalg.norm(v2 - v3), # Opposite v1
                np.linalg.norm(v3 - v1), # Opposite v2
                np.linalg.norm(v1 - v2)  # Opposite v3
            ]
            b_sort = np.argsort(b_edges) # [Index of Shortest, Medium, Longest]
            
            # Calculate internal edge lengths for the Catalog ECI Triangle
            e_edges = [
                np.linalg.norm(e2 - e3), # Opposite e1
                np.linalg.norm(e3 - e1), # Opposite e2
                np.linalg.norm(e1 - e2)  # Opposite e3
            ]
            e_sort = np.argsort(e_edges)
            
            # Anchor the vertices based on their opposite edge lengths
            body_triangle = [v1, v2, v3]
            body_indices = [idx1, idx2, idx3]
            eci_triangle = [e1, e2, e3]
            
            for rank in range(3): # Loop through Shortest, Medium, Longest
                b_vertex_idx = body_indices[b_sort[rank]]
                e_vertex = eci_triangle[e_sort[rank]]
                
                # Lock the 1-to-1 mapping in the dictionary
                if b_vertex_idx not in final_mapping:
                    final_mapping[b_vertex_idx] = e_vertex
                    
    # Compile the final arrays
    if len(final_mapping) < 3:
        return np.array([]), np.array([])
        
    aligned_body = []
    aligned_eci = []
    
    # Sort by the dictionary keys (body_indices) to keep the arrays ordered
    for b_idx in sorted(final_mapping.keys()):
        aligned_body.append(body_vectors[b_idx])
        aligned_eci.append(final_mapping[b_idx])
        
    return np.array(aligned_body), np.array(aligned_eci)