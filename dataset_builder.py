import numpy as np
from astropy.table import Table

def build_numpy_catalog(raw_csv_path, output_npy_path, mag_limit=6.0):
    print(f"Loading raw catalog from {raw_csv_path}...")
    
    # 1. Load the data using Astropy
    dat = Table.read(raw_csv_path, format='csv', encoding='utf-8')
    
    # 2. Filter by magnitude
    bright_stars = dat[dat['mag'] <= mag_limit]
    
    # 3. Clean corrupt coordinates
    bright_stars = bright_stars[~np.isnan(bright_stars['ra'])]
    bright_stars = bright_stars[~np.isnan(bright_stars['dec'])]
    
    # 4. Extract into a pure math matrix: [ID, RA, Dec, Magnitude]
    catalog_array = np.column_stack((
        bright_stars['id'], 
        bright_stars['ra'], 
        bright_stars['dec'], 
        bright_stars['mag']
    ))
    
    # 5. Save as a hyper-fast binary file
    np.save(output_npy_path, catalog_array)
    print(f"Success: Saved {len(catalog_array)} stars to {output_npy_path}")
    
    return catalog_array

# To run it, you would call:
build_numpy_catalog("dataset_star.csv", "optimized_catalog.npy")