import math
import struct
import sys
import os
import itertools
import numpy as np

# ==========================================
# CONFIGURATION CONSTANTS & PATHS
# ==========================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CATALOG_FILE = os.path.join(SCRIPT_DIR, "..", "data", "optimized_catalog.npy")
INC_DIR = os.path.join(SCRIPT_DIR, "..", "Core", "Inc")
BIN_DIR = os.path.join(SCRIPT_DIR, "..", "data")

MAX_FOV_DEGREES = 25.0          
MAX_STARS_FOR_TESTING = 30  # <--- REDUCED TO 30 TO GUARANTEE NO MEMORY ERRORS

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
                hip = int(row['HIP'])
                ra = float(row['RA_deg'])
                dec = float(row['Dec_deg'])
            else:
                hip = int(row[0])
                ra = float(row[1])
                dec = float(row[2])
            
            vec = ra_dec_to_vector(ra, dec)
            star_table.append({'hip': hip, 'vec': vec})
        except (ValueError, IndexError):
            continue

    # --- APPLY THE HARD LIMIT HERE ---
    star_table = star_table[:MAX_STARS_FOR_TESTING]
    print(f"Limited to {len(star_table)} stars to prevent RAM crashes.")

    # ---------------------------------------------------------
    # STEP 2: Generate Triangle Fingerprints
    # ---------------------------------------------------------
    print("Generating triangle combinations... (This will take 0.1 seconds)")
    triangle_table = []
    max_fov_rad = math.radians(MAX_FOV_DEGREES)

    for star1, star2, star3 in itertools.combinations(star_table, 3):
        d12 = angle_between(star1['vec'], star2['vec'])
        if d12 > max_fov_rad: continue
            
        d23 = angle_between(star2['vec'], star3['vec'])
        if d23 > max_fov_rad: continue
            
        d13 = angle_between(star1['vec'], star3['vec'])
        if d13 > max_fov_rad: continue

        angles = sorted([d12, d23, d13])
        triangle_table.append({
            'angles': angles,
            'hips': [star1['hip'], star2['hip'], star3['hip']]
        })

    # ---------------------------------------------------------
    # STEP 3: Sort the Triangle Database
    # ---------------------------------------------------------
    print("Sorting triangle database for binary search...")
    triangle_table.sort(key=lambda t: (t['angles'][0], t['angles'][1]))
    print(f"Generated {len(triangle_table)} valid triangles.")

    # ---------------------------------------------------------
    # STEP 4: Export to C Headers and Binary
    # ---------------------------------------------------------
    print("Exporting files directly to Core/Inc/ ...")
    
    os.makedirs(INC_DIR, exist_ok=True)

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