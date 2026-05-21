import os
import numpy as np
from scipy.spatial.transform import Rotation as R

from wheresat.coordinates import eci_to_body
from wheresat.camera import generate_image

# Route to the data folder from the root directory
DATA_DIR = os.path.join(os.path.abspath(os.path.dirname(__file__)), "data")

def run_star_tracker():
    print("--- 🚀 BOOTING STAR TRACKER FLIGHT SOFTWARE ---")
    
    # ---------------------------------------------------------
    # STEP 1: Load the Universe
    # ---------------------------------------------------------
    catalog_path = os.path.join(DATA_DIR, "optimized_catalog.npy")
    try:
        # Loaded format: [ID, X_eci, Y_eci, Z_eci, Magnitude]
        catalog = np.load(catalog_path)
        print(f"[SYSTEM] ECI Database loaded. Total stars: {len(catalog)}")
    except FileNotFoundError:
        print(f"[ERROR] Cannot find {catalog_path}! Run catalog.py first.")
        return

    # ---------------------------------------------------------
    # STEP 2: Simulate Satellite Orientation
    # ---------------------------------------------------------
    # Generate a random attitude quaternion to test the math
    random_rot = R.random()
    sat_quaternion = random_rot.as_quat()
    print(f"[SYSTEM] Satellite orientation (quaternion): {sat_quaternion.round(4)}")
    
    camera_width = 1024
    camera_fov = 12.0
    print(f"[SYSTEM] Camera specs: {camera_width}x{camera_width} px, FOV: {camera_fov}°")

    # ---------------------------------------------------------
    # STEP 3: The Math Crusher (Coordinate Transformation)
    # ---------------------------------------------------------
    print("[SYSTEM] Rotating universe into camera body frame...")
    
    # Extract just the X, Y, Z columns to feed into your math
    eci_vectors = catalog[:, 1:4]
    
    # Push through your math module
    body_vectors = eci_to_body(eci_vectors, sat_quaternion)
    
    # Re-pack the array with IDs and Magnitudes for the camera
    body_data = np.column_stack((
        catalog[:, 0],      # ID
        body_vectors,       # X_body, Y_body, Z_body
        catalog[:, 4]       # Magnitude
    ))

    # ---------------------------------------------------------
    # STEP 4: The Camera Sensor (Projection)
    # ---------------------------------------------------------
    print("[SYSTEM] Projecting 3D body vectors onto 2D sensor...")
    pixels = generate_image(body_data, camera_width, camera_fov)
    
    print(f"[SYSTEM] Projection complete. Captured {len(pixels)} visible stars.")

    # ---------------------------------------------------------
    # STEP 5: Output
    # ---------------------------------------------------------
    if len(pixels) == 0:
        print("\n[SYSTEM] Camera is pointing at deep, empty space. No stars captured.")
    else:
        print("\n--- FINAL CAMERA SENSOR OUTPUT ---")
        print("ID\tX_PIXEL\t\tY_PIXEL\t\tMAGNITUDE")
        print("-" * 55)
        # Just print the first 10 so we don't flood the terminal
        for i in range(min(10, len(pixels))):
            s_id = int(pixels[i, 0])
            x = pixels[i, 1]
            y = pixels[i, 2]
            mag = pixels[i, 3]
            print(f"{s_id}\t{x:.2f}\t\t{y:.2f}\t\t{mag:.2f}")

if __name__ == "__main__":
    run_star_tracker()