"""
@file export_catalog.py
@brief Highly optimized Star Catalog Compiler for WhereSat.

This script processes raw Hipparcos data, filters for guide stars, 
generates triangle fingerprints using graph-theoretic intersections, 
and exports the results to C headers and binary files for the STM32.

Optimizations:
1. Diagonal FOV Fix: Uses FOV * sqrt(2) for chord length.
2. Graph Intersection: Reduces triangle search from O(N^3) to O(N_pairs).
3. Vectorization: Uses NumPy for parallel geometry calculations.
4. Binary Search Prep: Sorts database by the shortest angle.

@author Aditya & Yash (WhereSat Team)
"""

import numpy as np
from scipy.spatial import KDTree
from collections import defaultdict
import os
import time
import struct
from pathlib import Path

# ==========================================
# CONFIGURATION
# ==========================================
MAX_MAGNITUDE = 4.5             # Standard for reliable Star ID
MAX_FOV_DEGREES = 25.0          # Camera side-length
# The Diagonal Fix: Maximum distance between two stars in a square FOV
MAX_DIAGONAL_DEG = MAX_FOV_DEGREES * np.sqrt(2)

# Path Logic
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
CATALOG_INPUT = os.path.join(ROOT_DIR, "data", "optimized_catalog.npy")
INC_DIR = os.path.join(ROOT_DIR, "Core", "Inc")
BIN_DIR = os.path.join(ROOT_DIR, "data")

def main():
    print("--- 🚀 STARTING CATALOG COMPILATION ---")
    start_time = time.time()

    # 1. Load and Filter Catalog
    if not os.path.exists(CATALOG_INPUT):
        print(f"Error: Could not find {CATALOG_INPUT}")
        return

    raw_data = np.load(CATALOG_INPUT)
    # Filter by magnitude (Column 4)
    guide_mask = raw_data[:, 4] <= MAX_MAGNITUDE
    catalog = raw_data[guide_mask]
    
    star_ids = catalog[:, 0].astype(np.uint32)
    vectors = catalog[:, 1:4].astype(np.float32)
    
    num_stars = len(catalog)
    print(f"   -> Guide Stars: {num_stars} (Mag <= {MAX_MAGNITUDE})")

    # 2. Map Spatial Network (Pairs)
    # Chord length formula: L = sqrt(2 - 2*cos(theta))
    max_diagonal_rad = np.radians(MAX_DIAGONAL_DEG)
    max_chord = np.sqrt(2 - 2 * np.cos(max_diagonal_rad))
    
    print(f"   -> Mapping pairs within {MAX_DIAGONAL_DEG:.2f}° diagonal...")
    tree = KDTree(vectors)
    pairs = tree.query_pairs(r=max_chord)
    
    # 3. Graph Theory Triangle Search
    print("   -> Executing set intersections for triangles...")
    adj = defaultdict(set)
    for i, j in pairs:
        adj[i].add(j) # query_pairs guarantees i < j
        
    tri_indices = []
    for i, j in pairs:
        # Common neighbors of i and j form a triangle
        common = adj[i].intersection(adj[j])
        for k in common:
            tri_indices.append((i, j, k))
            
    tri_indices = np.array(tri_indices)
    num_triangles = len(tri_indices)
    print(f"   -> Found {num_triangles:,} valid triangles.")

    # 4. Vectorized Geometry (Parallel Arccos)
    print("   -> Vectorizing geometry calculations...")
    v1 = vectors[tri_indices[:, 0]]
    v2 = vectors[tri_indices[:, 1]]
    v3 = vectors[tri_indices[:, 2]]
    
    # Dot products
    d12 = np.clip(np.sum(v1 * v2, axis=1), -1.0, 1.0)
    d23 = np.clip(np.sum(v2 * v3, axis=1), -1.0, 1.0)
    d31 = np.clip(np.sum(v3 * v1, axis=1), -1.0, 1.0)
    
    # Convert to angles and sort each row [Short, Mid, Long]
    angles = np.column_stack((np.arccos(d12), np.arccos(d23), np.arccos(d31)))
    angles.sort(axis=1)
    
    # Map back to HIP IDs
    hips = np.column_stack((
        star_ids[tri_indices[:, 0]],
        star_ids[tri_indices[:, 1]],
        star_ids[tri_indices[:, 2]]
    ))

    # 5. Sort Database for Binary Search
    # We sort by the shortest angle (Column 0) so the MCU can search efficiently
    print("   -> Sorting database for Binary Search...")
    sort_idx = np.argsort(angles[:, 0])
    sorted_angles = angles[sort_idx]
    sorted_hips = hips[sort_idx]

    # 6. Export to Files
    os.makedirs(INC_DIR, exist_ok=True)
    os.makedirs(BIN_DIR, exist_ok=True)

    # A. catalog_metadata.h
    with open(os.path.join(INC_DIR, "catalog_metadata.h"), "w") as f:
        f.write("/* AUTO-GENERATED */\n#ifndef CATALOG_METADATA_H\n#define CATALOG_METADATA_H\n\n")
        f.write(f"#define CATALOG_NUM_STARS {num_stars}\n")
        f.write(f"#define CATALOG_NUM_TRIANGLES {num_triangles}\n\n")
        f.write("#endif")

    # B. catalog.h (C Arrays)
    print("   -> Writing C header...")
    with open(os.path.join(INC_DIR, "catalog.h"), "w") as f:
        f.write("/* AUTO-GENERATED */\n#ifndef CATALOG_H\n#define CATALOG_H\n#include <stdint.h>\n\n")
        
        # Star Vectors
        f.write(f"const float CATALOG_STAR_VECTORS[{num_stars}][4] = {{\n")
        for i in range(num_stars):
            f.write(f"    {{{star_ids[i]}, {vectors[i,0]:.6f}f, {vectors[i,1]:.6f}f, {vectors[i,2]:.6f}f}},\n")
        f.write("};\n\n")

        # Triangle Database
        f.write(f"const float CATALOG_TRIANGLES[{num_triangles}][3] = {{\n")
        for i in range(num_triangles):
            f.write(f"    {{{sorted_angles[i,0]:.6f}f, {sorted_angles[i,1]:.6f}f, {sorted_angles[i,2]:.6f}f}},\n")
        f.write("};\n\n")

        # Triangle HIP IDs
        f.write(f"const uint32_t CATALOG_TRIANGLE_IDS[{num_triangles}][3] = {{\n")
        for i in range(num_triangles):
            f.write(f"    {{{sorted_hips[i,0]}, {sorted_hips[i,1]}, {sorted_hips[i,2]}}},\n")
        f.write("};\n\n#endif")

    # C. catalog.bin (Binary for Flash)
    with open(os.path.join(BIN_DIR, "catalog.bin"), "wb") as f:
        for i in range(num_stars):
            f.write(struct.pack("<I3f", star_ids[i], *vectors[i]))
        for i in range(num_triangles):
            f.write(struct.pack("<3f3I", *sorted_angles[i], *sorted_hips[i]))

    print(f"--- ✅ SUCCESS: Compilation took {time.time() - start_time:.2f}s ---")

if __name__ == "__main__":
    main()