import os
import numpy as np
from astropy.table import Table

# Dynamically route to the /data folder at the root of your project
# __file__ is src/wheresat/catalog.py
# 3 dirnames up takes us to the WhereSat root folder
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DATA_DIR = os.path.join(ROOT_DIR, "data")

def build_numpy_catalog(raw_csv_filename="hipparcos_catalog.csv", output_npy_filename="optimized_catalog.npy", mag_limit=6.0):
    raw_csv_path = os.path.join(DATA_DIR, raw_csv_filename)
    output_npy_path = os.path.join(DATA_DIR, output_npy_filename)
    
    print(f"Loading raw catalog from {raw_csv_path}...")
    
    # 1. Load the data using Astropy
    dat = Table.read(raw_csv_path, format='csv', encoding='utf-8')
    
    # 2. Filter by magnitude
    bright_stars = dat[dat['mag'] <= mag_limit]
    
    # 3. Clean corrupt coordinates
    bright_stars = bright_stars[~np.isnan(bright_stars['ra'])]
    bright_stars = bright_stars[~np.isnan(bright_stars['dec'])]
    
    # 4. Convert spherical to Cartesian ECI unit vectors
    # Note: Using Yash's catch that RA is stored in hours (1 hr = 15 degrees)
    ra_rad = np.radians(bright_stars['ra'] * 15.0)
    dec_rad = np.radians(bright_stars['dec'])
    
    x = np.cos(dec_rad) * np.cos(ra_rad)
    y = np.cos(dec_rad) * np.sin(ra_rad)
    z = np.sin(dec_rad)
    
    # 5. Pack the final math matrix: [ID, X, Y, Z, Magnitude]
    catalog_array = np.column_stack((
        bright_stars['id'], 
        x, 
        y, 
        z, 
        bright_stars['mag']
    ))
    
    # 6. Save as binary file in the data/ folder
    np.save(output_npy_path, catalog_array)
    print(f"Success: Saved {len(catalog_array)} stars to {output_npy_path}")
    
    return catalog_array

if __name__ == "__main__":
    # Ensure the data directory exists
    os.makedirs(DATA_DIR, exist_ok=True)
    build_numpy_catalog()