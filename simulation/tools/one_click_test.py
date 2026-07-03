import os, sys, math, itertools
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial import cKDTree

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
DATA_DIR = os.path.join(ROOT_DIR, "data")
INC_DIR = os.path.join(ROOT_DIR, "firmware", "Core", "Inc")

sys.path.append(os.path.join(ROOT_DIR, "simulation"))
from wheresat.quest import compute_attitude_quest # <-- ADDED QUEST IMPORT

def angle_between(v1, v2):
    dot = max(-1.0, min(1.0, np.dot(v1, v2)))
    return math.acos(dot)

def main():
    print("1. Shrinking database to fit in STM32 Flash (512KB limit)...")
    catalog = np.load(os.path.join(DATA_DIR, "optimized_catalog.npy"))
    
    if catalog.dtype.names is not None:
        mags = catalog['mag'] if 'mag' in catalog.dtype.names else catalog['Vmag']
        hips = catalog['id'] if 'id' in catalog.dtype.names else catalog['HIP']
        vecs = np.column_stack((catalog['x'], catalog['y'], catalog['z']))
    else:
        mags = catalog[:, 4]
        hips = catalog[:, 0].astype(int)
        vecs = catalog[:, 1:4]

    mask = mags <= 2.5
    mini_hips = hips[mask]
    mini_vecs = vecs[mask]
    
    triangles = []
    max_fov = math.radians(20.0)
    tree = cKDTree(mini_vecs)
    pairs = tree.query_pairs(2.0 * math.sin(max_fov / 2.0))
    neighbors = {i: set() for i in range(len(mini_hips))}
    for i, j in pairs:
        neighbors[i].add(j)
        neighbors[j].add(i)
        
    for i in range(len(mini_hips)):
        for j in neighbors[i]:
            if j > i:
                common = neighbors[i].intersection(neighbors[j])
                for k in common:
                    if k > j:
                        d12 = angle_between(mini_vecs[i], mini_vecs[j])
                        d23 = angle_between(mini_vecs[j], mini_vecs[k])
                        d13 = angle_between(mini_vecs[i], mini_vecs[k])
                        triangles.append({
                            'angles': sorted([d12, d23, d13]),
                            'hips': [mini_hips[i], mini_hips[j], mini_hips[k]]
                        })
                        
    triangles.sort(key=lambda t: (t['angles'][0], t['angles'][1]))
    
    print("2. Exporting mini-database to Core/Inc...")
    os.makedirs(INC_DIR, exist_ok=True)
    with open(os.path.join(INC_DIR, "catalog_metadata.h"), "w") as f:
        f.write(f"#ifndef CATALOG_METADATA_H\n#define CATALOG_METADATA_H\n\n#define CATALOG_NUM_STARS {len(mini_hips)}\n#define CATALOG_NUM_TRIANGLES {len(triangles)}\n\n#endif\n")
        
    with open(os.path.join(INC_DIR, "catalog.h"), "w") as f:
        f.write("#ifndef CATALOG_H\n#define CATALOG_H\n\n#include <stdint.h>\n\n")
        f.write(f"const float CATALOG_STAR_VECTORS[{len(mini_hips)}][4] = {{\n")
        for i in range(len(mini_hips)):
            f.write(f"    {{{mini_hips[i]}, {mini_vecs[i][0]:.6f}, {mini_vecs[i][1]:.6f}, {mini_vecs[i][2]:.6f}}},\n")
        f.write("};\n\n")
        f.write(f"const float CATALOG_TRIANGLES[{len(triangles)}][6] = {{\n")
        for t in triangles:
            f.write(f"    {{{t['angles'][0]:.6f}, {t['angles'][1]:.6f}, {t['angles'][2]:.6f}, {t['hips'][0]}, {t['hips'][1]}, {t['hips'][2]}}},\n")
        f.write("};\n\n#endif\n")

    print("3. Scanning sky for a cluster of AT LEAST 5 STARS...")
    width = 1024
    focal_length = (width / 2) / math.tan(math.radians(20.0 / 2))
##    np.random.seed(42)
    
    while True:
        true_q = R.random().as_quat()
        r_body = R.from_quat(true_q) 
        body_stars = r_body.apply(mini_vecs)
        visible_mask = body_stars[:, 2] > 0
        visible_body = body_stars[visible_mask]
        visible_hips = mini_hips[visible_mask]
        
        pixels_x = (visible_body[:, 0] * focal_length / visible_body[:, 2]) + (width / 2)
        pixels_y = (visible_body[:, 1] * focal_length / visible_body[:, 2]) + (width / 2)
        
        in_frame = (pixels_x >= 0) & (pixels_x < width) & (pixels_y >= 0) & (pixels_y < width)
        
        if np.sum(in_frame) >= 5:
            break 
            
    final_x = pixels_x[in_frame][:5]
    final_y = pixels_y[in_frame][:5]
    final_hips = visible_hips[in_frame][:5]
    
    # --- NEW: CALCULATE QUEST QUATERNION ---
    # Convert pixels to body vectors
    vx = final_x - (width/2)
    vy = final_y - (width/2)
    vz = np.full_like(vx, focal_length)
    mags = np.sqrt(vx**2 + vy**2 + vz**2)
    measured_body = np.column_stack((vx/mags, vy/mags, vz/mags))
    
    # Get ECI vectors from the catalog
    matched_eci = mini_vecs[visible_mask][in_frame][:5]
    
    # Run Python QUEST
    calculated_q = compute_attitude_quest(matched_eci, measured_body)

    print("\n==================================================")
    print(" COPY AND PASTE THIS INTO mock_fpga.c:")
    print("==================================================\n")
    print(f"    packet->count = {len(final_x)};\n")
    for i in range(len(final_x)):
        print(f"    packet->centroids[{i}].x = {final_x[i]:.3f}f;  packet->centroids[{i}].y = {final_y[i]:.3f}f;")
    
    print("\n==================================================")
    print(f" EXPECTED HIP IDs ON STM32: {final_hips.tolist()}")
    print("--------------------------------------------------")
    print(" EXPECTED QUEST QUATERNION ON STM32:")
    print(f" Qx = {calculated_q[0]:.6f}")
    print(f" Qy = {calculated_q[1]:.6f}")
    print(f" Qz = {calculated_q[2]:.6f}")
    print(f" Qw = {calculated_q[3]:.6f}")
    print("==================================================\n")

if __name__ == "__main__":
    main()