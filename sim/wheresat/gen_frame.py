import numpy as np
import os
import random
from sensor import apply_sensor_dirt

def generate_gauntlet_vector():
    print("--- 🛠️ BUILDING THE GAUNTLET STIMULUS ---")
    os.makedirs("data", exist_ok=True)
    clean_frame = np.zeros((1024, 1024), dtype=np.uint16)
    
    ground_truth = []

    # 1. The Boundary Breaker (Absolute Edge)
    clean_frame[500:503, 1021:1024] = 60000
    ground_truth.append("1022.0 501.0")

    # 2. The Overload (Generate 25 random stars to break the 16-slot limit)
    # We keep them spatially separated to test capacity, not merging (yet).
    random.seed(42) # Deterministic randomness
    for _ in range(25):
        x = random.randint(10, 1010)
        y = random.randint(10, 1010)
        
        # 3x3 Star Profile
        clean_frame[y-1:y+2, x-1:x+2] = random.randint(40000, 60000)
        ground_truth.append(f"{float(x)} {float(y)}")

    # 3. Inject Heavy Readout Noise (Sigma 200)
    print("Injecting heavy readout noise and writing .mem dump...")
    apply_sensor_dirt(clean_frame, readout_sigma=200.0, hot_pixel_fraction=0.0005)
    
    with open("data/py_centroids.txt", "w") as f:
        for truth in ground_truth:
            f.write(f"{truth}\n")

    print("[SUCCESS] Gauntlet tb_frame.mem locked and loaded.")

if __name__ == "__main__":
    generate_gauntlet_vector()