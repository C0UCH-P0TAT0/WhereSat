import math
import struct
import sys
import os
import itertools
import numpy as np
from scipy.spatial import cKDTree

# ==========================================
# CONFIGURATION CONSTANTS & EXACT PATHS
# ==========================================
# Get the directory where this script lives (.../WhereSat/python_tools)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Go up one level to the main WhereSat folder
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))

# Define the exact paths based on your folder structure
CATALOG_FILE = os.path.join(ROOT_DIR, "data", "optimized_catalog.npy")
INC_DIR = os.path.join(ROOT_DIR, "Core", "Inc")
BIN_DIR = os.path.join(ROOT_DIR, "data")

# Production limits for Week 7
MAX_MAGNITUDE = 2.5             
MAX_FOV_DEGREES = 25.0          

def ra_dec_to_vector(ra_deg, dec_deg):
    ra = math.radians(ra_deg)
    dec = math.radians(dec_deg)
    x = math.cos(dec) * math.cos(ra)
    y = math.cos(dec) * math.sin(ra)
    z = math.sin(dec)
    return (x, y, z)

def angle_between(v1, v2):
    dot_product = v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]
    dot_product = max(-1.0, min(1.0, dot_product)) 
    return math.acos(dot_product)

def main():
    print(f"Reading raw catalog from {CATALOG_FILE}...")
    
    star_table = []
    
    try:
        catalog_data = np.load(CATALOG_FILE)
    except FileNotFoundError:
        print(f"Error: Could not find {CATALOG_FILE}.")
        sys.exit(1)

    for row in catalog_data:
        try:
            if catalog_data.dtype.names is not None:
                hip = int(row['id']) if 'id' in catalog_data.dtype.names else int(row['HIP'])
                mag = float(row['mag']) if 'mag' in catalog_data.dtype.names else float(row['Vmag'])
                x, y, z = float(row['x']), float(row['y']), float(row['z'])
                vec = (x, y, z)
            else:
                hip = int(row[0])
                vec = (float(row[1]), float(row[2]), float(row[3]))
                mag = float(row[4])
            
            if mag <= MAX_MAGNITUDE:
                star_table.append({'hip': hip, 'vec': vec})
        except (ValueError, IndexError, KeyError):
            continue

    print(f"Filtered down to {len(star_table)} stars (Magnitude <= {MAX_MAGNITUDE}).")

    # ---------------------------------------------------------
    # STEP 2: Generate Triangle Fingerprints (Memory Safe)
    # ---------------------------------------------------------
    print("Generating triangle combinations... (This will take a few minutes)")
    triangle_table = []
    max_fov_rad = math.radians(MAX_FOV_DEGREES)

    vectors = [s['vec'] for s in star_table]
    tree = cKDTree(vectors)
    chord_length = 2.0 * math.sin(max_fov_rad / 2.0)
    
    total_stars = len(star_table)
    
    for i in range(total_stars):
        # Print progress every 100 stars so you know it's working
        if i % 100 == 0:
            print(f"Processing star {i} of {total_stars}...")
            
        neighbors = tree.query_ball_point(star_table[i]['vec'], chord_length)
        valid_neighbors = [idx for idx in neighbors if idx > i]
        
        for j, k in itertools.combinations(valid_neighbors, 2):
            d23 = angle_between(star_table[j]['vec'], star_table[k]['vec'])
            
            if d23 <= max_fov_rad:
                d12 = angle_between(star_table[i]['vec'], star_table[j]['vec'])
                d13 = angle_between(star_table[i]['vec'], star_table[k]['vec'])
                
                angles = sorted([d12, d23, d13])
                triangle_table.append({
                    'angles': angles,
                    'hips': [star_table[i]['hip'], star_table[j]['hip'], star_table[k]['hip']]
                })

    # ---------------------------------------------------------
    # STEP 3: Sort the Triangle Database
    # ---------------------------------------------------------
    print(f"\nSorting {len(triangle_table)} valid triangles for binary search...")
    triangle_table.sort(key=lambda t: (t['angles'][0], t['angles'][1]))

    # ---------------------------------------------------------
    # STEP 4: Export to C Headers and Binary
    # ---------------------------------------------------------
    print("Exporting files...")
    
    os.makedirs(INC_DIR, exist_ok=True)
    os.makedirs(BIN_DIR, exist_ok=True)

    meta_path = os.path.join(INC_DIR, "catalog_metadata.h")
    with open(meta_path, "w") as f:
        f.write("/* AUTO-GENERATED FILE - DO NOT EDIT */\n")
        f.write("#ifndef CATALOG_METADATA_H\n#define CATALOG_METADATA_H\n\n")
        f.write(f"#define CATALOG_NUM_STARS {len(star_table)}\n")
        f.write(f"#define CATALOG_NUM_TRIANGLES {len(triangle_table)}\n\n")
        f.write("#endif // CATALOG_METADATA_H\n")

    cat_path = os.path.join(INC_DIR, "catalog.h")
    with open(cat_path, "w") as f:
        f.write("/* AUTO-GENERATED FILE - DO NOT EDIT */\n")
        f.write("#ifndef CATALOG_H\n#define CATALOG_H\n\n")
        f.write("#include <stdint.h>\n\n")
        
        f.write("// Format: {HIP_ID, X, Y, Z}\n")
        f.write(f"const float CATALOG_STAR_VECTORS[{len(star_table)}][4] = {{\n")
        for s in star_table:
            f.write(f"    {{{s['hip']}, {s['vec'][0]:.6f}, {s['vec'][1]:.6f}, {s['vec'][2]:.6f}}},\n")
        f.write("};\n\n")

        f.write("// Format: {Small_Angle, Mid_Angle, Large_Angle, HIP1, HIP2, HIP3}\n")
        f.write(f"const float CATALOG_TRIANGLES[{len(triangle_table)}][6] = {{\n")
        for t in triangle_table:
            f.write(f"    {{{t['angles'][0]:.6f}, {t['angles'][1]:.6f}, {t['angles'][2]:.6f}, ")
            f.write(f"{t['hips'][0]}, {t['hips'][1]}, {t['hips'][2]}}},\n")
        f.write("};\n\n")
        
        f.write("#endif // CATALOG_H\n")

    bin_path = os.path.join(BIN_DIR, "catalog.bin")
    with open(bin_path, "wb") as f:
        for s in star_table:
            f.write(struct.pack("<I3f", s['hip'], *s['vec']))
        for t in triangle_table:
            f.write(struct.pack("<3f3I", *t['angles'], *t['hips']))

    print(f"Export complete! Files saved to:\n - {meta_path}\n - {cat_path}\n - {bin_path}")

if __name__ == "__main__":
    main()