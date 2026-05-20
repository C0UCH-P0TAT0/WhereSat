import numpy as np

# Import your tools from the other files!
from subsetter import subset_visible_stars
from projection import project_to_pixels

def run_star_tracker_pipeline():
    print("--- 🚀 BOOTING STAR TRACKER FLIGHT SOFTWARE ---")
    
    # ---------------------------------------------------------
    # STEP 1: Load the Universe (From dataset_builder.py output)
    # ---------------------------------------------------------
    try:
        catalog = np.load("optimized_catalog.npy")
        print(f"[SYSTEM] Database loaded. Total stars: {len(catalog)}")
    except FileNotFoundError:
        print("[ERROR] Cannot find optimized_catalog.npy! Run dataset_builder.py first.")
        return

    # ---------------------------------------------------------
    # STEP 2: Receive Aditya's Output
    # ---------------------------------------------------------
    # In Week 2, Aditya's ECI math will feed these numbers dynamically.
    # For now, we simulate his output pointing at Orion.
    current_ra = 85.0
    current_dec = -1.0
    camera_fov = 12.0
    camera_width = 1024
    print(f"[SYSTEM] Aditya's Math indicates camera pointing at RA {current_ra}°, Dec {current_dec}°")

    # ---------------------------------------------------------
    # STEP 3: The Spatial Filter (subsetter.py)
    # ---------------------------------------------------------
    # We pass the massive catalog in, and catch the tiny list of visible stars
    print("[SYSTEM] Running Spatial Filter...")
    visible_stars = subset_visible_stars(catalog, current_ra, current_dec, camera_fov)
    print(f"[SYSTEM] Filter complete. Found {len(visible_stars)} stars in the camera's view.")
    
    if len(visible_stars) == 0:
        print("[SYSTEM] Camera is pointing at empty space. Shutting down.")
        return

    # ---------------------------------------------------------
    # STEP 4: The Math Crusher (projection.py)
    # ---------------------------------------------------------
    # We pass the output of Step 3 DIRECTLY into the input of Step 4
    print("[SYSTEM] Running Gnomonic Projection Engine...")
    pixel_coordinates = project_to_pixels(visible_stars, current_ra, current_dec, camera_width, camera_fov)
    print("[SYSTEM] Projection complete. 3D coordinates crushed to 2D pixels.")

    # ---------------------------------------------------------
    # STEP 5: Final Output Validation
    # ---------------------------------------------------------
    print("\n--- FINAL CAMERA PIXELS ---")
    print("ID\tX_PIXEL\t\tY_PIXEL\t\tMAGNITUDE")
    print("-" * 50)
    for i in range(len(pixel_coordinates)):
        s_id = int(pixel_coordinates[i, 0])
        x = pixel_coordinates[i, 1]
        y = pixel_coordinates[i, 2]
        mag = pixel_coordinates[i, 3]
        print(f"{s_id}\t{x:.2f}\t\t{y:.2f}\t\t{mag:.2f}")

# Run the software
if __name__ == "__main__":
    run_star_tracker_pipeline()