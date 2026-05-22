import numpy as np
from scipy.ndimage import binary_opening, label, center_of_mass

def extract_centroids(image_array: np.ndarray, threshold: int = 200) -> np.ndarray:
    """
    Strips hardware noise and calculates Center of Mass (CoM) for surviving star blobs.
    
    Args:
        image_array: 2D uint16 array (the dirty sensor image)
        threshold: The ADU cutoff to delete readout noise
        
    Returns:
        Nx2 array of [X, Y] sub-pixel centroid coordinates
    """
    # 1. The Global Guillotine (Readout Noise)
    # Nuke the baseline static. Anything below 200 ADU becomes pure black (0).
    thresholded = np.where(image_array > threshold, image_array, 0)
    
    # 2. Morphological Opening (Hot Pixels)
    # We define a 3x3 structuring element. 
    # Hot pixels are isolated 1x1 spikes. The opening operation (erosion followed by dilation)
    # physically deletes anything smaller than 3x3. Hot pixels vanish. Stars survive.
    structure = np.ones((3, 3), dtype=bool)
    mask = thresholded > 0
    clean_mask = binary_opening(mask, structure=structure)
    
    # Re-apply the surviving mask to the actual pixel intensities
    cleaned_image = thresholded * clean_mask
    
    # 3. Blob Detection
    # Group connected lit pixels into distinct, numbered clusters
    labeled_array, num_features = label(clean_mask)
    
    centroids = []
    if num_features > 0:
        # 4. Center of Mass
        # ndimage computes the intensity-weighted center of each isolated blob
        # Returns a list of (Y, X) tuples because arrays are row-major
        coms = center_of_mass(cleaned_image, labeled_array, range(1, num_features + 1))
        
        for (y, x) in coms:
            centroids.append([x, y])  # Swap back to standard Cartesian [X, Y]
            
    return np.array(centroids)