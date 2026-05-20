import numpy as np

def subset_visible_stars(catalog_array, boresight_ra_deg, boresight_dec_deg, fov_deg):
    margin = fov_deg / 2.0 
    
    # ASTRONOMY FIX: The database stores RA in HOURS, not degrees!
    # We must divide our target RA by 15.0 so it matches the database.
    ra_min = (boresight_ra_deg - margin) / 15.0
    ra_max = (boresight_ra_deg + margin) / 15.0
    
    # Declination (Dec) is vertical, so it is always stored in degrees. No fix needed!
    dec_min = boresight_dec_deg - margin
    dec_max = boresight_dec_deg + margin
    
    # Run the filter
    mask = (catalog_array[:, 1] >= ra_min) & \
           (catalog_array[:, 1] <= ra_max) & \
           (catalog_array[:, 2] >= dec_min) & \
           (catalog_array[:, 2] <= dec_max)
           
    visible_stars = catalog_array[mask]
    return visible_stars

# ==========================================
# VERIFICATION TEST 
# ==========================================
##
#if __name__ == "__main__":
#    print("--- BOOTING SUBSETTER TEST ---")

#    try:
#        catalog = np.load("optimized_catalog.npy")
#    except FileNotFoundError:
#        print("Error: Could not find optimized_catalog.npy.")
#        exit()

#    test_ra = 85.0     # Orion's Belt RA in degrees
#    test_dec = -1.0    # Orion's Belt Dec in degrees
#    test_fov = 12.0
    
#    print(f"Camera pointing at RA: {test_ra}°, Dec: {test_dec}° (FOV: {test_fov}°)")
    
    # Run the function
#    visible_stars = subset_visible_stars(catalog, test_ra, test_dec, test_fov)
    
#    print("\n--- RESULTS ---")
#    print(f"Total stars in universe: {len(catalog)}")
#    print(f"Stars visible in camera view: {len(visible_stars)}")
    
#    if len(visible_stars) > 0:
#        # We multiply by 15 here to convert the database hours back into degrees for our test!
#        min_ra = np.min(visible_stars[:, 1]) * 15.0
#        max_ra = np.max(visible_stars[:, 1]) * 15.0
#        min_dec = np.min(visible_stars[:, 2])
#        max_dec = np.max(visible_stars[:, 2])
        
#        print("\n--- BOUNDARY CHECK ---")
#        print(f"Lowest RA found:  {min_ra:.2f}°  (Pass: {min_ra >= (test_ra - 6.0)})")
#        print(f"Highest RA found: {max_ra:.2f}° (Pass: {max_ra <= (test_ra + 6.0)})")
#        print(f"Lowest Dec found: {min_dec:.2f}°  (Pass: {min_dec >= (test_dec - 6.0)})")
#        print(f"Highest Dec found: {max_dec:.2f}°  (Pass: {max_dec <= (test_dec + 6.0)})")
        
#        print("\nSTATUS: SUCCESS! The filter caught the stars.")
#    else:
#        print("\nSTATUS: No stars found.")

