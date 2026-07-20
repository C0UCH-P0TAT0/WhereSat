# ==========================================
# CAMERA PARAMETERS (Virtual Sensor)
# ==========================================
CAMERA_RES_X = 1024       # Pixel width
CAMERA_RES_Y = 1024       # Pixel height
FOV_X_DEG = 20.0          # Field of View in degrees (Horizontal)
FOV_Y_DEG = 20.0          # Field of View in degrees (Vertical)

# ==========================================
# QUATERNION CONVENTION (DO NOT CHANGE)
# ==========================================
# We strictly use the SciPy scalar-last convention: [x, y, z, w]
# Do NOT use the Astropy scalar-first [w, x, y, z] without explicit conversion.