import numpy as np

def project_to_pixels(visible_stars, boresight_ra_deg, boresight_dec_deg, image_width_pixels, fov_deg):
    
    # 1. Convert all degrees to Radians
    ra = np.radians(visible_stars[:, 1] * 15.0) 
    dec = np.radians(visible_stars[:, 2])
    
    ra0 = np.radians(boresight_ra_deg)
    dec0 = np.radians(boresight_dec_deg)
    
    # 2. Calculate Camera Focal Length in pixels
    fov_rad = np.radians(fov_deg)
    focal_length = (image_width_pixels / 2.0) / np.tan(fov_rad / 2.0)
    
    # 3. Gnomonic Projection Core Physics
    cos_c = np.sin(dec0) * np.sin(dec) + np.cos(dec0) * np.cos(dec) * np.cos(ra - ra0)
    
    x = focal_length * (np.cos(dec) * np.sin(ra - ra0)) / cos_c
    y = focal_length * (np.cos(dec0) * np.sin(dec) - np.sin(dec0) * np.cos(dec) * np.cos(ra - ra0)) / cos_c
    
    # 4. Shift coordinates from Math Center (0,0) to Computer Top-Left (0,0)
    center_x = image_width_pixels / 2.0
    center_y = image_width_pixels / 2.0
    
    pixel_x = center_x + x
    pixel_y = center_y - y  # Flipped because image Y-axis goes down!
    
    # 5. Pack the final results: [ID, Pixel_X, Pixel_Y, Magnitude]
    projected_stars = np.column_stack((
        visible_stars[:, 0], 
        pixel_x, 
        pixel_y, 
        visible_stars[:, 3]
    ))
    
    return projected_stars

# ==========================================
# VERIFICATION TEST: THE BULLS-EYE
# ==========================================
#if __name__ == "__main__":
#    print("--- BOOTING PROJECTION TEST ---")
#    
   # 1. Define Camera Parameters
#    test_ra = 85.0     # Pointing at Orion
#    test_dec = -1.0    
#   test_fov = 12.0
#    test_width = 1024  # A standard 1024x1024 pixel camera
#    
#    # 2. Create Two Fake Stars to test the math
#    # Format: [ID, RA (in Hours!), Dec (in Degrees), Magnitude]
#    fake_stars = np.array([
#        [999, 85.0 / 15.0, -1.0, 1.0],  # STAR 1: Exactly on the boresight (The Bulls-Eye)
#        [888, 87.0 / 15.0,  1.0, 2.0]   # STAR 2: 2 degrees right, 2 degrees up
#    ])
    
#    print(f"Simulating a {test_width}x{test_width} camera...")
#    print(f"Dead center of the image should be X: 512.0, Y: 512.0\n")
    
    # 3. Run the physics engine
#    pixels = project_to_pixels(fake_stars, test_ra, test_dec, test_width, test_fov)
    
    # 4. Print Results
#    print("--- RAW PIXEL OUTPUT ---")
#    for i in range(len(pixels)):
#        star_id = int(pixels[i, 0])
#        px_x = pixels[i, 1]
#        px_y = pixels[i, 2]
#        print(f"Star {star_id}: X = {px_x:.2f}, Y = {px_y:.2f}")
        
    # 5. Validate the math
#    bullseye_x = pixels[0, 1]
#    bullseye_y = pixels[0, 2]
    
#    if abs(bullseye_x - 512.0) < 0.1 and abs(bullseye_y - 512.0) < 0.1:
#        print("\nSTATUS: SUCCESS! The Bulls-Eye star landed perfectly in the center.")
#    else:
#        print("\nSTATUS: FAILED! The trigonometry is misaligned.")