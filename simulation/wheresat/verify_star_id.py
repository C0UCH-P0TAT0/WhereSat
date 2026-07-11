import numpy as np
import pickle
import itertools
import os
from scipy.spatial import cKDTree
from star_id import identify_stars, calculate_triangle_fingerprint, pixels_to_vectors

# --- Configuration (Must match export_catalog.py) ---
IMAGE_WIDTH = 1024
CAMERA_FOV = 25.0
MATCH_TOLERANCE = 1e-4

# --- Paths ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
CATALOG_PATH = r"C:\Users\DELL\Desktop\WhereSat\data\optimized_catalog.npy"
DB_TREE_PATH = r"C:\Users\DELL\Desktop\WhereSat\data\triangle_tree.pkl"
DB_MAP_PATH = r"C:\Users\DELL\Desktop\WhereSat\data\triangle_id_map.npy"

# --- Input Centroids (The 14 stars you provided) ---
# Format: [X, Y]
centroids = np.array([
    [944.000, 32.699],   # Star 0
    [226.020, 306.203],  # Star 1
    [656.293, 467.305],  # Star 2
    [535.398, 493.746],  # Star 3
    [251.918, 569.000],  # Star 4
    [731.402, 576.523],  # Star 5
    [775.461, 598.234],  # Star 6
    [495.266, 648.809],  # Star 7
    [80.574, 784.000],   # Star 8
    [409.438, 818.336],  # Star 9
    [40.977, 865.391],   # Star 10
    [232.719, 926.164],  # Star 11
    [780.238, 937.629],  # Star 12
    [30.113, 956.312]    # Star 13
])

def main():
    print("--- 🔍 STAR ID OFFLINE VERIFICATION ---")
    
    # 1. Load the exact database formed by export_catalog.py
    print(f"Loading database from {ROOT_DIR}/data/...")
    if not os.path.exists(DB_TREE_PATH):
        print("Error: triangle_tree.pkl not found. Run export_catalog.py first.")
        return

    with open(DB_TREE_PATH, 'rb') as f:
        kd_tree = pickle.load(f)
    triangle_id_map = np.load(DB_MAP_PATH)
    catalog = np.load(CATALOG_PATH)

    # 2. Convert Centroids to Body Vectors
    print(f"Projecting {len(centroids)} centroids into 3D space...")
    body_vectors = pixels_to_vectors(centroids, IMAGE_WIDTH, CAMERA_FOV)

    # 3. Generate and Print Top 10 Triangle Fingerprints
    print("\n--- TOP 10 GENERATED TRIANGLE FINGERPRINTS ---")
    tri_count = 0
    # itertools.combinations generates triangles in a deterministic order
    for indices in itertools.combinations(range(len(body_vectors)), 3):
        v1, v2, v3 = body_vectors[indices[0]], body_vectors[indices[1]], body_vectors[indices[2]]
        fingerprint = calculate_triangle_fingerprint(v1, v2, v3)
        
        if tri_count < 10:
            print(f"Tri {tri_count} (Stars {indices}): {fingerprint}")
        tri_count += 1
    print(f"Total triangles generated: {tri_count}")

    # 4. Run Identification
    print("\n--- MATCHING AGAINST CATALOG ---")
    aligned_body, aligned_eci = identify_stars(
        centroids, 
        IMAGE_WIDTH, 
        CAMERA_FOV, 
        kd_tree, 
        triangle_id_map, 
        catalog,
        tolerance=MATCH_TOLERANCE
    )

    if len(aligned_body) > 0:
        print(f"✅ SUCCESS: Matched {len(aligned_body)} stars!")
        print("\nMatched Body Vectors (First 3):")
        print(aligned_body[:3])
        print("\nCorresponding ECI Vectors (First 3):")
        print(aligned_eci[:3])
    else:
        print("❌ FAILURE: No stars matched. Possible reasons:")
        print("1. Stars are dimmer than MAX_MAGNITUDE (4.5) in catalog.")
        print("2. MATCH_TOLERANCE is too strict for the noise level.")
        print("3. Camera FOV/Focal Length mismatch between simulation and ID.")

if __name__ == "__main__":
    main()