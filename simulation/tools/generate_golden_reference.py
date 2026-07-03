import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R

# ==========================================
# BULLETPROOF PATH ROUTING
# ==========================================
SCRIPT_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPT_DIR.parent.parent
DATA_DIR = ROOT_DIR / "data"

sys.path.append(str(SCRIPT_DIR.parent))

from wheresat.database import build_star_database
from wheresat.star_id import identify_stars
from wheresat.quest import compute_attitude_quest

def main():
    print("--- GENERATING GOLDEN REFERENCE DATASET FOR STM32 C-PORT ---")
    
    catalog_path = DATA_DIR / "optimized_catalog.npy"
    
    print("Loading database...")
    catalog = np.load(catalog_path)
    database_tree, triangle_ids = build_star_database(str(catalog_path), max_fov_deg=20.0)

    width = 1024
    fov_deg = 20.0
    focal_length = (width / 2) / np.tan(np.radians(fov_deg / 2))
    eci_vectors = catalog[:, 1:4]

    print("Scanning the sky for a dense cluster of bright stars...")
    
    # =====================================================================
    # AUTO-SEARCH: Keep spinning the camera until we find 3+ bright stars!
    # =====================================================================
    np.random.seed(42) # Use a seed so it picks the same stars every time
    
    while True:
        true_q = R.random().as_quat()
        r_body = R.from_quat(true_q)
        
        body_stars = r_body.apply(eci_vectors)
        visible_mask = body_stars[:, 2] > 0
        visible_body = body_stars[visible_mask]
        visible_data = catalog[visible_mask]
        
        pixels_x = (visible_body[:, 0] * focal_length / visible_body[:, 2]) + (width / 2)
        pixels_y = (visible_body[:, 1] * focal_length / visible_body[:, 2]) + (width / 2)
        
        in_frame = (pixels_x >= 0) & (pixels_x < width) & (pixels_y >= 0) & (pixels_y < width)
        valid_indices = np.where(in_frame)[0]
        
        # Only count stars that are Magnitude 2.5 or brighter
        bright_indices = [i for i in valid_indices if visible_data[i, 4] <= 2.5]
        
        if len(bright_indices) >= 3:
            print("Found a great star cluster!")
            break # We found one! Exit the loop.
            
    # Grab the top 5 brightest stars (or however many it found)
    sorted_valid = sorted(bright_indices, key=lambda i: visible_data[i, 4])
    top_stars_idx = sorted_valid[:5]
    
    test_centroids = np.column_stack((pixels_x[top_stars_idx], pixels_y[top_stars_idx]))
    true_hips = visible_data[top_stars_idx, 0].astype(int)

    # 3. Run the Python LIS and QUEST Algorithms
    measured_body, matched_eci = identify_stars(
        centroids=test_centroids, 
        camera_width=width, 
        camera_fov=fov_deg, 
        kd_tree=database_tree, 
        triangle_id_map=triangle_ids, 
        catalog=catalog, 
        tolerance=2e-3
    )
    
    calculated_q = compute_attitude_quest(matched_eci, measured_body)

    # 4. Print the Answer Key for Aditya
    print("\n======================================================")
    print(" GOLDEN REFERENCE ANSWER KEY FOR STM32 VERIFICATION")
    print("======================================================")
    print(f"Camera Config: {width}x{width} pixels, FOV = {fov_deg} deg, Focal Length = {focal_length:.2f} px\n")
    
    print("--- INPUT: RAW CENTROIDS (Feed these into STM32) ---")
    for i, (x, y) in enumerate(test_centroids):
        print(f"Star {i}: X = {x:.3f}, Y = {y:.3f}")
        
    print("\n--- EXPECTED OUTPUT 1: 3D BODY VECTORS ---")
    for i, (x, y) in enumerate(test_centroids):
        vx = x - (width/2)
        vy = y - (width/2)
        vz = focal_length
        mag = np.sqrt(vx**2 + vy**2 + vz**2)
        print(f"Star {i}: [{vx/mag:.6f}, {vy/mag:.6f}, {vz/mag:.6f}]")

    print("\n--- EXPECTED OUTPUT 2: MATCHED HIP IDs ---")
    print(f"Expected HIP IDs: {true_hips.tolist()}")

    print("\n--- EXPECTED OUTPUT 3: FINAL QUEST QUATERNION ---")
    print(f"Q_x = {calculated_q[0]:.6f}")
    print(f"Q_y = {calculated_q[1]:.6f}")
    print(f"Q_z = {calculated_q[2]:.6f}")
    print(f"Q_w = {calculated_q[3]:.6f}")
    print("======================================================\n")

if __name__ == "__main__":
    main()