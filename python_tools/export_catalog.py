"""
@file export_catalog.py
@brief Highly optimized Star Catalog Compiler for WhereSat.

This script processes raw Hipparcos data, filters for guide stars, 
generates triangle fingerprints using a symmetric KNN spatial network,
and exports the results for STM32 and Python HIL.

@author Aditya & Yash (WhereSat Team)
"""

import numpy as np
from scipy.spatial import KDTree
from collections import defaultdict
import os
import time
import struct
import pickle
from pathlib import Path

# ==========================================
# CONFIGURATION
# ==========================================
MAX_MAGNITUDE = 4.5              
MAX_FOV_DEGREES = 25.0          
MAX_DIAGONAL_DEG = MAX_FOV_DEGREES * np.sqrt(2)

# Start low. Increase if coverage fails in simulation, decrease if MCU memory blows up.
KNN_NEIGHBORS = 8                

# Path Logic
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))

CATALOG_INPUT = os.path.join(ROOT_DIR, "data", "optimized_catalog.npy")
INC_DIR = os.path.join(ROOT_DIR, "firmware", "Core", "Inc")
DATA_DIR = os.path.join(ROOT_DIR, "data")

def main():
    print("--- 🚀 STARTING CATALOG COMPILATION ---")
    start_time = time.time()

    if not os.path.exists(CATALOG_INPUT):
        print(f"Error: Could not find input catalog at {CATALOG_INPUT}")
        return

    raw_data = np.load(CATALOG_INPUT)
    guide_mask = raw_data[:, 4] <= MAX_MAGNITUDE
    catalog = raw_data[guide_mask]
        
    star_ids = catalog[:, 0].astype(np.uint32)
    vectors = catalog[:, 1:4].astype(np.float32)
       
    num_stars = len(catalog)
    print(f"   -> Guide Stars: {num_stars} (Mag <= {MAX_MAGNITUDE})")

    max_diagonal_rad = np.radians(MAX_DIAGONAL_DEG)
    max_chord = np.sqrt(2 - 2 * np.cos(max_diagonal_rad))

    # 1. Map Spatial Network (KNN Culling)
    print(f"   -> Mapping KNN spatial network (k={KNN_NEIGHBORS})...")
    spatial_tree = KDTree(vectors)
    distances, nbrs = spatial_tree.query(vectors, k=KNN_NEIGHBORS + 1) # +1 to account for self

    # 2. Build Symmetric Adjacency Graph
    print("   -> Building undirected adjacency graph...")
    adj = defaultdict(set)
    for i in range(num_stars):
        # Skip index 0 (self)
        for dist, j in zip(distances[i, 1:], nbrs[i, 1:]):
            if dist <= max_chord:
                adj[i].add(j)
                adj[j].add(i)

    # 3. Generate Valid Triangles
    print("   -> Generating local triangles...")
    triangle_set = set()
    
    for i in range(num_stars):
        neighbors = list(adj[i])
        n_count = len(neighbors)
        
        for idx1 in range(n_count):
            for idx2 in range(idx1 + 1, n_count):
                j = neighbors[idx1]
                k = neighbors[idx2]
                
                # Check third leg validity: j and k must be within max_chord of each other
                if k in adj[j]:
                    # Strict topological sort for uniqueness
                    tri = tuple(sorted([i, j, k]))
                    triangle_set.add(tri)
                
    if len(triangle_set) == 0:
        print("   -> 🚨 FATAL: Zero valid triangles generated. Increase KNN_NEIGHBORS or expand FOV limit.")
        return

    tri_indices = np.array(list(triangle_set))
    num_triangles = len(tri_indices)
    print(f"   -> Found {num_triangles:,} valid triangles.")

    # 3.5 Verify Triangle Participation Profiling
    print("   -> Profiling star participation in triangles...")
    counts = np.bincount(tri_indices.ravel(), minlength=num_stars)
    
    orphans = np.sum(counts == 0)
    weak_nodes = np.sum(counts < 3)
    
    print(f"   -> Orphaned Stars (0 tris): {orphans}")
    print(f"   -> High Risk Stars (<3 tris): {weak_nodes}")
    
    pct = np.percentile(counts, [1, 5, 25, 50, 75, 95, 99])
    print(f"   -> Participation Percentiles:")
    print(f"      [1%]: {pct[0]:.0f} | [5%]: {pct[1]:.0f} | [25%]: {pct[2]:.0f} | [50%]: {pct[3]:.0f}")
    print(f"      [75%]: {pct[4]:.0f} | [95%]: {pct[5]:.0f} | [99%]: {pct[6]:.0f}")

    if orphans > 0 or weak_nodes > 0:
        print("   -> ⚠️ WARNING: Network contains dead/weak zones. Consider tuning KNN_NEIGHBORS.")

    # 4. Vectorized Geometry
    print("   -> Vectorizing geometry calculations...")
    v1, v2, v3 = vectors[tri_indices[:, 0]], vectors[tri_indices[:, 1]], vectors[tri_indices[:, 2]]
        
    d12 = np.clip(np.sum(v1 * v2, axis=1), -1.0, 1.0)
    d23 = np.clip(np.sum(v2 * v3, axis=1), -1.0, 1.0)
    d31 = np.clip(np.sum(v3 * v1, axis=1), -1.0, 1.0)
        
    angles = np.column_stack((np.arccos(d12), np.arccos(d23), np.arccos(d31)))
    angles.sort(axis=1) # [Short, Mid, Long]
        
    hips = np.column_stack((
        star_ids[tri_indices[:, 0]],
        star_ids[tri_indices[:, 1]],
        star_ids[tri_indices[:, 2]]
    ))

    # 5. Sort Database for Binary Search (MCU) and KDTree (Python)
    print("   -> Sorting database by short angle...")
    sort_idx = np.argsort(angles[:, 0])
    sorted_angles = angles[sort_idx]
    sorted_hips = hips[sort_idx]

    # 6. Export to Files
    os.makedirs(INC_DIR, exist_ok=True)
    os.makedirs(DATA_DIR, exist_ok=True)

    # A. catalog_metadata.h
    with open(os.path.join(INC_DIR, "catalog_metadata.h"), "w") as f:
        f.write("/* AUTO-GENERATED */\n#ifndef CATALOG_METADATA_H\n#define CATALOG_METADATA_H\n\n")
        f.write(f"#define CATALOG_NUM_STARS {num_stars}\n")
        f.write(f"#define CATALOG_NUM_TRIANGLES {num_triangles}\n\n")
        f.write("#endif\n")

    # B. catalog.h
    print("   -> Writing C header...")
    with open(os.path.join(INC_DIR, "catalog.h"), "w") as f:
        f.write("/* AUTO-GENERATED */\n#ifndef CATALOG_H\n#define CATALOG_H\n#include <stdint.h>\n\n")
        
        f.write(f"const float CATALOG_STAR_VECTORS[{num_stars}][4] = {{\n")
        for i in range(num_stars):
            f.write(f"    {{{star_ids[i]}, {vectors[i,0]:.6f}f, {vectors[i,1]:.6f}f, {vectors[i,2]:.6f}f}},\n")
        f.write("};\n\n")
        
        f.write(f"const float CATALOG_TRIANGLES[{num_triangles}][3] = {{\n")
        for i in range(num_triangles):
            f.write(f"    {{{sorted_angles[i,0]:.6f}f, {sorted_angles[i,1]:.6f}f, {sorted_angles[i,2]:.6f}f}},\n")
        f.write("};\n\n")
        
        f.write(f"const uint32_t CATALOG_TRIANGLE_IDS[{num_triangles}][3] = {{\n")
        for i in range(num_triangles):
            f.write(f"    {{{sorted_hips[i,0]}, {sorted_hips[i,1]}, {sorted_hips[i,2]}}},\n")
        f.write("};\n\n#endif\n")

    # C. catalog.bin
    print("   -> Writing binary flash dump...")
    with open(os.path.join(DATA_DIR, "catalog.bin"), "wb") as f:
        for i in range(num_stars):
            f.write(struct.pack("<I3f", star_ids[i], *vectors[i]))
        for i in range(num_triangles):
            f.write(struct.pack("<3f3I", *sorted_angles[i], *sorted_hips[i]))

    # D. Python HIL Simulation Files
    print("   -> Exporting Python HIL database files...")
    db_tree = KDTree(sorted_angles)
    with open(os.path.join(DATA_DIR, "triangle_tree.pkl"), "wb") as f:
        pickle.dump(db_tree, f)
        
    np.save(os.path.join(DATA_DIR, "triangle_id_map.npy"), sorted_hips)

    print(f"--- ✅ SUCCESS: Compilation took {time.time() - start_time:.2f}s ---")
    print(f"Files saved to:\n - {INC_DIR}\n - {DATA_DIR}")

if __name__ == "__main__":
    main()