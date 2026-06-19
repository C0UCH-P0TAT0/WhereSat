import numpy as np
import os
import random
import math
from sensor import apply_sensor_dirt

def generate_gauntlet_vector():
    print("--- 🛠️ BUILDING THE GAUNTLET STIMULUS ---")
    os.makedirs("data", exist_ok=True)
    
    clean_frame = np.zeros((1024, 1024), dtype=np.uint32)
    
    ground_truth = []
    stars = []

    # 1. The Boundary Breaker
    # Placed at X=1021 so the 3x3 envelope spans indices exactly 1020, 1021, 1022.
    # This tests the high bounds without clipping the PSF physical structure.
    stars.append((1021, 501, 50000))

    # 2. The Spatial Exclusion Generator
    random.seed(42)
    
    while len(stars) < 26: 
        x = random.randint(10, 1010)
        y = random.randint(10, 1010)
        
        conflict = False
        for (sx, sy, _) in stars:
            if math.hypot(x - sx, y - sy) < 10.0:
                conflict = True
                break
        
        if not conflict:
            intensity = random.randint(40000, 60000)
            stars.append((x, y, intensity))

    # 3. Inject Normalized Gaussian PSF
    # We normalize to 1.0 so total flux (mass) exactly matches the intensity parameter.
    raw_kernel = np.array([
        [0.125, 0.250, 0.125],
        [0.250, 1.000, 0.250],
        [0.125, 0.250, 0.125]
    ])
    psf_kernel = raw_kernel / np.sum(raw_kernel)

    for (x, y, total_flux) in stars:
        star_patch = (psf_kernel * total_flux).astype(np.uint16)
        
        clean_frame[y-1:y+2, x-1:x+2] += star_patch
        
        mass = np.sum(star_patch)
        ground_truth.append(f"{float(x)} {float(y)} {mass}")

    clean_frame = np.clip(clean_frame, 0, 65535).astype(np.uint16)

    # 4. Inject Realistic Noise
    print("Injecting readout noise...")
    dirty_frame = apply_sensor_dirt(clean_frame, readout_sigma=200.0, hot_pixel_fraction=0.00002)
    
    # 5. Write the Hardware Memory File (.mem)
    print("Writing data/tb_frame.mem for Verilog $readmemh...")
    with open("data/tb_frame.mem", "w") as f:
        for pixel in dirty_frame.flatten():
            f.write(f"{pixel:04x}\n")
    
    # 6. Write Ground Truth
    with open("data/py_centroids.txt", "w") as f:
        for truth in ground_truth:
            f.write(f"{truth}\n")

    print(f"[SUCCESS] Gauntlet generated. {len(stars)} distinct stars locked and loaded in data/.")

if __name__ == "__main__":
    generate_gauntlet_vector()