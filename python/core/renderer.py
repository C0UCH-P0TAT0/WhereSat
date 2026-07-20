import numpy as np

def render_star_field(visible_stars: np.ndarray, width: int, sigma: float = 1.5, base_flux: float = 50000.0) -> np.ndarray:
    """
    Renders sub-pixel star coordinates into a 16-bit optical image array.
    
    Args:
        visible_stars: Nx4 array [ID, X_Pixel, Y_Pixel, Magnitude]
        width: Image width/height in pixels (e.g., 1024)
        sigma: Spread of the Gaussian blur in pixels
        base_flux: The peak amplitude for a magnitude 0 star
        
    Returns:
        A 2D NumPy array of shape (width, width) of type np.uint16
    """
    # 1. Blank canvas (float64 prevents accumulation overflow before clipping)
    image = np.zeros((width, width), dtype=np.float64)
    
    if len(visible_stars) == 0:
        return image.astype(np.uint16)

    # 2. Extract coordinates and magnitudes
    x_centers = visible_stars[:, 1]
    y_centers = visible_stars[:, 2]
    magnitudes = visible_stars[:, 3]
    
    # 3. Pogson's Equation: Convert Magnitude to Peak Photon Amplitude
    amplitudes = base_flux * (10.0 ** (-0.4 * magnitudes))
    
    # 4. Optimized Bounding Box Rendering
    # Calculate a safe radius (4 standard deviations captures ~99.9% of the Gaussian energy)
    radius = int(np.ceil(4 * sigma))
    
    for x0, y0, A in zip(x_centers, y_centers, amplitudes):
        # Determine the integer pixel bounds for this specific star
        x_min = max(0, int(x0) - radius)
        x_max = min(width, int(x0) + radius + 1)
        y_min = max(0, int(y0) - radius)
        y_max = min(width, int(y0) + radius + 1)
        
        # Skip if the bounding box is completely off-screen
        if x_min >= x_max or y_min >= y_max:
            continue
            
        # Create a tiny localized grid for just this star
        x_grid, y_grid = np.meshgrid(
            np.arange(x_min, x_max),
            np.arange(y_min, y_max)
        )
        
        # Calculate the 2D Gaussian over the localized patch
        gaussian = A * np.exp(-((x_grid - x0)**2 + (y_grid - y0)**2) / (2 * sigma**2))
        
        # Accumulate the light onto the main image
        image[y_min:y_max, x_min:x_max] += gaussian

    # 5. Quantum Photon Noise & Full Well Capacity
    # Poisson simulates the random arrival of photons. 
    # We clip the canvas at 1,000,000 to represent the absolute physical capacity of the pixel bucket. This prevents corrupted "-99 magnitude" ghost stars from crashing the Poisson lambda calculator.
    image = np.clip(image, 0, 1000000.0)
    image = np.random.poisson(image).astype(np.float64)
    
    # 6. Sensor Saturation (16-bit ceiling)
    # Anything above 65535 is clipped (saturated pixels).
    image = np.clip(image, 0, 65535)
    
    return image.astype(np.uint16)