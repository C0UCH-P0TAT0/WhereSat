import math
import itertools
import struct
import csv
import sys

# ==========================================
# CONFIGURATION CONSTANTS
# ==========================================
CATALOG_FILE = "hipparcos.csv"  # Expected columns: HIP, RA_deg, Dec_deg, Vmag
MAX_MAGNITUDE = 4.5             # Only keep stars brighter than or equal to 4.5
MAX_FOV_DEGREES = 25.0          # Maximum camera Field of View

def ra_dec_to_vector(ra_deg, dec_deg):
    """Converts Right Ascension and Declination to a 3D ECI unit vector."""
    ra = math.radians(ra_deg)
    dec = math.radians(dec_deg)
    x = math.cos(dec) * math.cos(ra)
    y = math.cos(dec) * math.sin(ra)
    z = math.sin(dec)
    return (x, y, z)

def angle_between(v1, v2):
    """Calculates the angular distance (in radians) between two unit vectors."""
    dot_product = v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]
    dot_product = max(-1.0, min(1.0, dot_product)) # Clamp to prevent math domain errors
    return math.acos(dot_product)

def main():
    print(f"Reading raw catalog from {CATALOG_FILE}...")
    
    star_table = []
    
    # ---------------------------------------------------------
    # STEP 1: Read CSV, Filter by Magnitude, Generate Vectors
    # ---------------------------------------------------------
    try:
        with open(CATALOG_FILE, mode='r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                try:
                    hip = int(row['HIP'])
                    ra = float(row['RA_deg'])
                    dec = float(row['Dec_deg'])
                    mag = float(row['Vmag'])
                    
                    if mag <= MAX_MAGNITUDE:
                        vec = ra_dec_to_vector(ra, dec)
                        star_table.append({'hip': hip, 'vec': vec})
                except ValueError:
                    # Skip rows with missing or invalid data
                    continue
    except FileNotFoundError:
        print(f"Error: Could not find {CATALOG_FILE}. Please ensure the file exists.")
        sys.exit(1)

    print(f"Filtered down to {len(star_table)} stars (Magnitude <= {MAX_MAGNITUDE}).")

    # ---------------------------------------------------------
    # STEP 2: Generate Triangle Fingerprints
    # ---------------------------------------------------------
    print("Generating triangle combinations... (This may take a minute)")
    triangle_table = []
    max_fov_rad = math.radians(MAX_FOV_DEGREES)

    for star1, star2, star3 in itertools.combinations(star_table, 3):
        d12 = angle_between(star1['vec'], star2['vec'])
        d23 = angle_between(star2['vec'], star3['vec'])
        d13 = angle_between(star1['vec'], star3['vec'])

        # Skip triangles that cannot fit inside the camera's FOV
        if d12 > max_fov_rad or d23 > max_fov_rad or d13 > max_fov_rad:
            continue

        # Sort the angles: Smallest, Middle, Largest
        angles = sorted([d12, d23, d13])
        
        triangle_table.append({
            'angles': angles,
            'hips': [star1['hip'], star2['hip'], star3['hip']]
        })

    # ---------------------------------------------------------
    # STEP 3: Sort the Triangle Database for Binary Search
    # ---------------------------------------------------------
    print("Sorting triangle database for binary search...")
    triangle_table.sort(key=lambda t: (t['angles'][0], t['angles'][1]))
    print(f"Generated {len(triangle_table)} valid triangles.")

    # ---------------------------------------------------------
    # STEP 4: Export to C Headers and Binary
    # ---------------------------------------------------------
    print("Exporting files...")
    
    # 1. Write catalog_metadata.h
    with open("catalog_metadata.h", "w") as f:
        f.write("/* AUTO-GENERATED FILE - DO NOT EDIT */\n")
        f.write("#ifndef CATALOG_METADATA_H\n#define CATALOG_METADATA_H\n\n")
        f.write(f"#define CATALOG_NUM_STARS {len(star_table)}\n")
        f.write(f"#define CATALOG_NUM_TRIANGLES {len(triangle_table)}\n\n")
        f.write("#endif // CATALOG_METADATA_H\n")

    # 2. Write catalog.h (The flat C arrays)
    with open("catalog.h", "w") as f:
        f.write("/* AUTO-GENERATED FILE - DO NOT EDIT */\n")
        f.write("#ifndef CATALOG_H\n#define CATALOG_H\n\n")
        f.write("#include <stdint.h>\n\n")
        
        # Write Star Vectors Array
        f.write("// Format: {HIP_ID, X, Y, Z}\n")
        f.write(f"const float CATALOG_STAR_VECTORS[{len(star_table)}][4] = {{\n")
        for s in star_table:
            f.write(f"    {{{s['hip']}, {s['vec'][0]:.6f}, {s['vec'][1]:.6f}, {s['vec'][2]:.6f}}},\n")
        f.write("};\n\n")

        # Write Triangle Fingerprints Array
        f.write("// Format: {Small_Angle, Mid_Angle, Large_Angle, HIP1, HIP2, HIP3}\n")
        f.write(f"const float CATALOG_TRIANGLES[{len(triangle_table)}][6] = {{\n")
        for t in triangle_table:
            f.write(f"    {{{t['angles'][0]:.6f}, {t['angles'][1]:.6f}, {t['angles'][2]:.6f}, ")
            f.write(f"{t['hips'][0]}, {t['hips'][1]}, {t['hips'][2]}}},\n")
        f.write("};\n\n")
        
        f.write("#endif // CATALOG_H\n")

    # 3. Write catalog.bin (Raw binary dump)
    with open("catalog.bin", "wb") as f:
        # Pack stars: 1 uint32 (HIP), 3 floats (X,Y,Z)
        for s in star_table:
            f.write(struct.pack("<I3f", s['hip'], *s['vec']))
        # Pack triangles: 3 floats (Angles), 3 uint32 (HIPs)
        for t in triangle_table:
            f.write(struct.pack("<3f3I", *t['angles'], *t['hips']))

    print("Export complete! Generated catalog_metadata.h, catalog.h, and catalog.bin.")

if __name__ == "__main__":
    main()