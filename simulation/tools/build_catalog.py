import os
import numpy as np
from astropy.table import Table

# Dynamically route to the /data folder at the root of your project
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DATA_DIR = os.path.join(ROOT_DIR, "data")

def build_numpy_catalog(raw_csv_filename="hipparcos_catalog.csv", output_npy_filename="optimized_catalog.npy", mag_limit=6.0):
    raw_csv_path = os.path.join(DATA_DIR, raw_csv_filename)
    output_npy_path = os.path.join(DATA_DIR, output_npy_filename)
    
    print(f"Loading raw catalog from {raw_csv_path}...")
    
    # 1. Load the data using Astropy
    dat = Table.read(raw_csv_path, format='csv', encoding='utf-8')
    
    # 2. Aggressive Sanitization (The Native Astropy Way)
    # If a row is missing data, Astropy flags it in the column's .mask array.
    # We drop any row where id, ra, dec, or mag is masked.
    for col in ['id', 'ra', 'dec', 'mag']:
        if hasattr(dat[col], 'mask'):
            dat = dat[~dat[col].mask]
            
    # 3. Filter out phantom/invalid Hipparcos IDs (<= 0)
    dat = dat[dat['id'] > 0]
    
    # 4. Filter by magnitude
    bright_stars = dat[dat['mag'] <= mag_limit]
    
    # 5. Convert spherical to Cartesian ECI unit vectors
    ra_rad = np.radians(bright_stars['ra'] * 15.0)
    dec_rad = np.radians(bright_stars['dec'])
    
    x = np.cos(dec_rad) * np.cos(ra_rad)
    y = np.cos(dec_rad) * np.sin(ra_rad)
    z = np.sin(dec_rad)
    
    # 6. Pack the final math matrix: [ID, X, Y, Z, Magnitude]
    catalog_array = np.column_stack((
        bright_stars['id'].astype(np.float32), # Cast to float here so column_stack doesn't complain
        x, 
        y, 
        z, 
        bright_stars['mag']
    ))
    
    # 7. Save as binary file
    np.save(output_npy_path, catalog_array)
    print(f"Success: Saved {len(catalog_array)} valid stars to {output_npy_path}")
    
    return catalog_array

if __name__ == "__main__":
    os.makedirs(DATA_DIR, exist_ok=True)
    build_numpy_catalog()