import sys
from pathlib import Path
import numpy as np
import os
import math
from scipy.spatial.transform import Rotation as R

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPT_DIR.parent.parent
DATA_DIR = ROOT_DIR / "data"

sys.path.append(str(SCRIPT_DIR.parent))
from wheresat.sensor import apply_sensor_dirt

def generate_gauntlet_vector():
    os.makedirs(DATA_DIR, exist_ok=True)
    clean_frame = np.zeros((1024, 1024), dtype=np.uint32)
    ground_truth = []

    # 1. Load the REAL Catalog
    catalog = np.load(DATA_DIR / "optimized_catalog.npy")
    if catalog.dtype.names is not None:
        vecs = np.column_stack((catalog['x'], catalog['y'], catalog['z']))
        mags = catalog['mag'] if 'mag' in catalog.dtype.names else catalog['Vmag']
    else:
        vecs = catalog[:, 1:4]
        mags = catalog[:, 4]

    # Filter to Mag <= 2.5 to match STM32
    mini_vecs = vecs[mags <= 2.5]

    width = 1024
    focal_length = (width / 2) / math.tan(math.radians(20.0 / 2))
    
    # =================================================================
    # THE FIX: Auto-Search for 5 stars (Same as one_click_test.py!)
    # =================================================================
    np.random.seed(42) 
    while True:
        true_q = R.random().as_quat()
        r_body = R.from_quat(true_q) 
        body_stars = r_body.apply(mini_vecs)
        
        visible_mask = body_stars[:, 2] > 0
        visible_body = body_stars[visible_mask]
        
        pixels_x = (visible_body[:, 0] * focal_length / visible_body[:, 2]) + (width / 2)
        pixels_y = (visible_body[:, 1] * focal_length / visible_body[:, 2]) + (width / 2)
        
        in_frame = (pixels_x >= 0) & (pixels_x < width) & (pixels_y >= 0) & (pixels_y < width)
        
        if np.sum(in_frame) >= 5:
            break 
            
    final_x = pixels_x[in_frame][:5]
    final_y = pixels_y[in_frame][:5]

    # 3. Draw the REAL stars
    raw_kernel = np.array([[0.125, 0.250, 0.125], [0.250, 1.000, 0.250], [0.125, 0.250, 0.125]])
    psf_kernel = raw_kernel / np.sum(raw_kernel)

    for i in range(len(final_x)):
        x, y = int(final_x[i]), int(final_y[i])
        total_flux = 50000 # Make them bright
        star_patch = (psf_kernel * total_flux).astype(np.uint16)
        
        if 1 <= x <= 1022 and 1 <= y <= 1022:
            clean_frame[y-1:y+2, x-1:x+2] += star_patch
            ground_truth.append(f"{float(x)} {float(y)} {total_flux}")

    clean_frame = np.clip(clean_frame, 0, 65535).astype(np.uint16)
    dirty_frame = apply_sensor_dirt(clean_frame, readout_sigma=200.0, hot_pixel_fraction=0.00002)
    
    # 4. Save Files
    with open(DATA_DIR / "tb_frame.mem", "w") as f:
        for pixel in dirty_frame.flatten():
            f.write(f"{pixel:04x}\n")
            
    with open(DATA_DIR / "py_centroids.txt", "w") as f:
        for truth in ground_truth:
            f.write(f"{truth}\n")

if __name__ == "__main__":
    generate_gauntlet_vector()