import numpy as np
from scipy.spatial import KDTree
from collections import defaultdict
from pathlib import Path
import time

def build_star_database(catalog_path: str, max_fov_deg: float = 20.0):
    """
    Highly optimized graph-theory and vectorized implementation of the 
    LIS Triangle Database. Reduces O(N^3) complexity to runtime seconds.
    """
    print("--- 🚀 SPINNING UP TRIANGLE DATABASE ---")
    start_time = time.time()
    raw_catalog = np.load(catalog_path)
    
    # --- THE GUIDE STAR FILTER ---
    # Slicing the universe to Magnitude 4.5 and brighter.
    # Drops the combination pool from 225 Million to ~1.5 Million triangles.
    guide_mask = raw_catalog[:, 4] <= 4.5
    catalog = raw_catalog[guide_mask]
    
    print(f"   -> Filtered universe from {len(raw_catalog)} to {len(catalog)} Guide Stars.")
    
    hipparcos_ids = catalog[:, 0]
    vectors = catalog[:, 1:4] # 3D Unit Vectors
    
    # 1. The Diagonal Fix (FOV is a square, maximum chord is the diagonal)
    max_diagonal_rad = np.radians(max_fov_deg * np.sqrt(2))
    max_chord_length = np.sqrt(2 - 2 * np.cos(max_diagonal_rad))
    
    # 2. Stage 1: Map the Spatial Network
    print("   -> Mapping spatial network...")
    spatial_tree = KDTree(vectors)
    pairs = spatial_tree.query_pairs(r=max_chord_length)
    
    # 3. Stage 2: Graph Theory Intersections (Lightning Fast)
    print("   -> Executing C-level set intersections...")
    adj = defaultdict(set)
    for i, j in pairs:
        # query_pairs guarantees i < j, creating a directed graph
        adj[i].add(j)
        
    triangles = []
    for i, j in pairs:
        # If 'k' is a neighbor of 'i' and a neighbor of 'j', it forms a triangle.
        common_neighbors = adj[i].intersection(adj[j])
        for k in common_neighbors:
            triangles.append((i, j, k))
            
    triangles = np.array(triangles)
    print(f"   -> Found {len(triangles):,} raw triangles. Vectorizing matrix math...")
    
    # 4. Stage 3: NumPy Vectorization (Parallel Geometry)
    v1 = vectors[triangles[:, 0]]
    v2 = vectors[triangles[:, 1]]
    v3 = vectors[triangles[:, 2]]
    
    # Calculate all dot products across all triangles simultaneously
    dot_12 = np.clip(np.sum(v1 * v2, axis=1), -1.0, 1.0)
    dot_23 = np.clip(np.sum(v2 * v3, axis=1), -1.0, 1.0)
    dot_31 = np.clip(np.sum(v3 * v1, axis=1), -1.0, 1.0)
    
    # Arccos executes in parallel
    angles = np.column_stack((np.arccos(dot_12), np.arccos(dot_23), np.arccos(dot_31)))
    
    # Sort each row horizontally so every fingerprint is strictly [Short, Medium, Long]
    angles.sort(axis=1)
    
    triangle_ids = np.column_stack((
        hipparcos_ids[triangles[:, 0]],
        hipparcos_ids[triangles[:, 1]],
        hipparcos_ids[triangles[:, 2]]
    ))
    
    # 5. Stage 4: Build the Final Memory Bank
    print("   -> Compiling K-D Tree Search Engine...")
    database_tree = KDTree(angles)
    
    print(f"[SYSTEM] Database locked in {time.time() - start_time:.2f} seconds.")
    return database_tree, triangle_ids

import numpy as np
import time

def export_database_to_c(angles: np.ndarray, triangle_ids: np.ndarray, output_filename: str = "catalog.h"):
    """
    Demolishes the dynamic tree and compiles a flat, sorted, bare-metal C array.
    """
    print("--- 💥 INITIATING K-D TREE DEMOLITION ---")
    start_time = time.time()
    
    # 1. The Binary Search Preparation
    # We MUST sort the entire database numerically by the "Shortest Angle" (Column 0).
    print("   -> Sorting database for Binary Search...")
    sort_indices = np.argsort(angles[:, 0])
    sorted_angles = angles[sort_indices]
    sorted_ids = triangle_ids[sort_indices]

    num_triangles = len(sorted_angles)
    print(f"   -> Flattening {num_triangles:,} fingerprints into C syntax...")

    # 2. Write the bare-metal C Header
    with open(output_filename, "w") as f:
        f.write("#ifndef CATALOG_H\n")
        f.write("#define CATALOG_H\n\n")
        f.write("#include <stdint.h>\n")
        
        # PROGMEM Macro: Prevents the array from loading into MCU SRAM
        f.write("#if defined(__AVR__)\n")
        f.write("  #include <avr/pgmspace.h>\n")
        f.write("#else\n")
        f.write("  #define PROGMEM\n")
        f.write("#endif\n\n")
        
        f.write(f"const uint32_t NUM_TRIANGLES = {num_triangles};\n\n")
        
        # Write the Angles Array (The Fingerprints)
        f.write("// Fingerprints: [Shortest, Medium, Longest] in Radians\n")
        f.write("const float triangle_db[][3] PROGMEM = {\n")
        for i, row in enumerate(sorted_angles):
            end_char = ",\n" if i < num_triangles - 1 else "\n"
            f.write(f"    {{{row[0]:.6f}, {row[1]:.6f}, {row[2]:.6f}}}{end_char}")
        f.write("};\n\n")
        
        # Write the Catalog IDs (The Output)
        f.write("// Hipparcos Catalog IDs [Opposite Shortest, Opposite Medium, Opposite Longest]\n")
        f.write("const uint32_t triangle_ids[][3] PROGMEM = {\n")
        for i, row in enumerate(sorted_ids):
            end_char = ",\n" if i < num_triangles - 1 else "\n"
            f.write(f"    {{{row[0]}, {row[1]}, {row[2]}}}{end_char}")
        f.write("};\n\n")
        
        f.write("#endif // CATALOG_H\n")
        
    print(f"[SUCCESS] Flat C database compiled to {output_filename} in {time.time() - start_time:.2f} seconds.")
    print(f"[SYSTEM] MCU is cleared for dynamic-allocation-free operation.")


if __name__ == "__main__":
    data_dir = Path(__file__).resolve().parents[2] / "data"
    catalog_file = str(data_dir / "optimized_catalog.npy")
    build_star_database(catalog_file)