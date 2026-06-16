import numpy as np

def generate_image(body_data: np.ndarray, width: int, fov_deg: float) -> np.ndarray:
    """
    Projects 3D body vectors onto a 2D camera sensor using the pinhole camera model.
    Filters out any stars that are behind the camera or off the sensor.
    
    Args:
        body_data: N x 5 array [ID, X_body, Y_body, Z_body, Magnitude]
        width: Image width/height in pixels
        fov_deg: Field of view in degrees
        
    Returns:
        M x 4 array of visible stars [ID, Pixel_X, Pixel_Y, Magnitude]
    """
    # 1. Calculate focal length in pixels
    fov_rad = np.radians(fov_deg)
    focal_length = (width / 2.0) / np.tan(fov_rad / 2.0)
    
    cx, cy = width / 2.0, width / 2.0
    
    # 2. Filter 1: Z > 0 (The star must be IN FRONT of the camera, not behind it)
    # This automatically replaces Yash's entire subsetter.py script
    front_mask = body_data[:, 3] > 1e-5
    front_stars = body_data[front_mask]
    
    if len(front_stars) == 0:
        return np.array([]) # Looking at empty space
        
    # 3. Pinhole Projection Core Physics (x = f * x/z)
    # The camera boresight is staring straight down the Z-axis
    px_x = cx + focal_length * (front_stars[:, 1] / front_stars[:, 3])
    px_y = cy - focal_length * (front_stars[:, 2] / front_stars[:, 3]) # Flipped Y for computer graphics
    
    # 4. Filter 2: The star must actually land on the sensor
    sensor_mask = (px_x >= 0) & (px_x <= width) & (px_y >= 0) & (px_y <= width)
    
    # 5. Pack the surviving stars into the final delivery format
    visible_pixels = np.column_stack((
        front_stars[sensor_mask, 0],  # ID
        px_x[sensor_mask],            # X Pixel
        px_y[sensor_mask],            # Y Pixel
        front_stars[sensor_mask, 4]   # Magnitude
    ))
    
    return visible_pixels