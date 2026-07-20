import numpy as np
from scipy.optimize import curve_fit
from scipy.ndimage import median_filter

def gaussian_2d(coords, A, x0, y0, sigma_x, sigma_y, B):
    x, y = coords
    exponent = -((x - x0)**2 / (2 * sigma_x**2) + (y - y0)**2 / (2 * sigma_y**2))
    g = A * np.exp(exponent) + B
    return g.ravel()

def extract_centroids_gaussian(image_array: np.ndarray, com_centroids: np.ndarray) -> np.ndarray:
    centroids = []
    pad = 5  

    for com in com_centroids:
        cx = int(np.round(com[0]))
        cy = int(np.round(com[1]))
        
        x_min = max(0, cx - pad)
        x_max = min(image_array.shape[1], cx + pad + 1)
        y_min = max(0, cy - pad)
        y_max = min(image_array.shape[0], cy + pad + 1)
        
        raw_patch = image_array[y_min:y_max, x_min:x_max]
        
        if raw_patch.shape[0] < 3 or raw_patch.shape[1] < 3:
            continue

        # 2. Clean the hardware static
        patch = median_filter(raw_patch, size=3)

        x_range = np.arange(x_min, x_max)
        y_range = np.arange(y_min, y_max)
        x_grid, y_grid = np.meshgrid(x_range, y_range)

        coords = (x_grid.ravel(), y_grid.ravel())
        pixel_values = patch.ravel()

        # ---------------------------------------------------------
        # 3. PURE MATHEMATICAL GUESS (100% Independent)
        # ---------------------------------------------------------
        background_guess = np.min(patch)
        amplitude_guess = np.max(patch) - background_guess
        
        # Because the patch is clean, argmax is now safe to use!
        brightest_idx = np.argmax(patch)
        x0_guess = coords[0][brightest_idx]
        y0_guess = coords[1][brightest_idx]
        sigma_guess = 1.5 
        
        p0 = [amplitude_guess, x0_guess, y0_guess, sigma_guess, sigma_guess, background_guess]

        # ---------------------------------------------------------
        # 4. PURE BOUNDARIES
        # We leash the optimizer to the guess.
        # ---------------------------------------------------------
        lower_bounds = [
            0.0,             
            x0_guess - 2.0,  # Bound to the guess
            y0_guess - 2.0,  # Bound to the guess
            0.5,             
            0.5,             
            0.0              
        ]

        upper_bounds = [
            65535.0,         
            x0_guess + 2.0,  # Bound to the guess
            y0_guess + 2.0,  # Bound to the guess
            3.0,             
            3.0,             
            65535.0          
        ]

        try:
            popt, _ = curve_fit(
                gaussian_2d, 
                coords, 
                pixel_values, 
                p0=p0, 
                bounds=(lower_bounds, upper_bounds), 
                maxfev=2000
            )
            centroids.append([popt[1], popt[2]])
        except (RuntimeError, ValueError):
            continue

    return np.array(centroids)