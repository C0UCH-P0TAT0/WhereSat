import numpy as np
from scipy.spatial.transform import Rotation as R
from pathlib import Path

# Phase 1 & 2: Simulation Environment
from wheresat.coordinates import eci_to_body
from wheresat.camera import generate_image
from wheresat.renderer import render_star_field
from wheresat.sensor import apply_sensor_dirt

# Phase 3: Extraction
from wheresat.centroiding import extract_centroids
from wheresat.gaussian import extract_centroids_gaussian

# Phase 4: Lost-In-Space Identification
from wheresat.database import build_star_database
# pyrefly: ignore [missing-import]
from wheresat.star_id import identify_stars 

def run_closed_loop_test(num_iterations: int = 10):
    print("--- 🚀 INITIATING CLOSED-LOOP LIS BENCHMARK ---")
    
    # Setup Paths & Hardware Config
    data_dir = Path(__file__).resolve().parents[2] / "data"
    catalog_path = str(data_dir / "optimized_catalog.npy")
    
    camera_width = 1024
    camera_fov = 20.0
    noise_sigma = 15.0 # Standard operating noise
    
    # 1. Load the Universe & Build the Database (Your Code)
    catalog = np.load(catalog_path)
    eci_vectors = catalog[:, 1:4]
    database_tree, pair_ids = build_star_database(catalog_path, max_fov_deg=camera_fov)
    
    success_count = 0
    total_stars_tested = 0
    total_stars_identified = 0

    for i in range(num_iterations):
        print(f"\n[Simulation {i+1}/{num_iterations}] Tumbling satellite...")
        
        # Phase A: The True Universe
        sat_quaternion = R.random().as_quat()
        body_vectors = eci_to_body(eci_vectors, sat_quaternion)
        body_data = np.column_stack((catalog[:, 0], body_vectors, catalog[:, 4]))
        
        # Generate mathematical ground truth
        truth_pixels = generate_image(body_data, camera_width, camera_fov)
        if len(truth_pixels) < 3:
            print("   -> Staring at empty space. Skipping.")
            continue
            
        true_ids = set(truth_pixels[:, 0])
        total_stars_tested += len(true_ids)
        
        # Phase B: The Degraded Sensor
        clean_image = render_star_field(truth_pixels, camera_width, sigma=1.5)
        dirty_image = apply_sensor_dirt(clean_image, readout_sigma=noise_sigma, hot_pixel_fraction=0.001)
        
        # Phase C: Centroid Extraction
        dynamic_threshold = int(noise_sigma * 4)
        com_centroids = extract_centroids(dirty_image, threshold=dynamic_threshold)
        gaussian_centroids = extract_centroids_gaussian(dirty_image, com_centroids)
        
        if len(gaussian_centroids) < 3:
            print("   -> [FAIL] Sensor blackout. Could not extract 3+ stars.")
            continue
            
        # Phase D: Yash's Star ID Engine
        # We pass the dirty centroids and your static database
        identified_ids = identify_stars(
            centroids=gaussian_centroids,
            camera_width=camera_width,
            camera_fov=camera_fov,
            kd_tree=database_tree,
            triangle_id_map=pair_ids
        )
        
        # Phase E: The Grading
        matches = set(identified_ids).intersection(true_ids)
        total_stars_identified += len(matches)
        
        if len(matches) >= 3:
            success_count += 1
            print(f"   -> [PASS] Attitude Locked. Identified {len(matches)}/{len(true_ids)} stars.")
        else:
            print(f"   -> [FAIL] Lost in Space. Only identified {len(matches)}/{len(true_ids)} stars.")

    # Final Metrics
    print("\n==========================================")
    print(" 📊 WEEK 4 INTEGRATION RESULTS")
    print("==========================================")
    print(f"Attitude Lock Rate:  {(success_count / num_iterations) * 100:.1f}%")
    print(f"Star ID Efficiency:  {(total_stars_identified / total_stars_tested) * 100:.1f}%")
    if (success_count / num_iterations) > 0.9:
        print("\n[VERDICT] 🟢 FLIGHT READY. Merge to main.")
    else:
        print("\n[VERDICT] 🔴 PIPELINE FAILURE. Check tolerance constraints.")

if __name__ == "__main__":
    run_closed_loop_test(num_iterations=50)