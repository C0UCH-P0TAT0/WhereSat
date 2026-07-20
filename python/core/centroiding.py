import numpy as np
from scipy.ndimage import label, center_of_mass

def extract_centroids(image_array: np.ndarray, threshold: int = 200) -> np.ndarray:
    """
    Strips hardware noise, filters hot pixels by area, and calculates true CoM.
    """
    # 1. The Global Guillotine
    mask = image_array > threshold
    
    # 2. Blob Detection (Group connected pixels)
    labeled_array, num_features = label(mask)
    
    centroids = []
    if num_features > 0:
        # Get the physical pixel area of each detected blob
        # bincount is highly optimized; index corresponds to the blob label ID
        blob_sizes = np.bincount(labeled_array.ravel())
        
        # Subtract the noise floor so CoM only calculates true photon intensity.
        # Cast to float32 to prevent uint16 underflow (e.g., 0 - 200)
        true_intensity = np.where(mask, image_array.astype(np.float32) - threshold, 0)
        
        # 4. Filter and Calculate
        valid_labels = []
        for i in range(1, num_features + 1):
            # The Hot Pixel Filter: Nuke anything that is exactly 1 pixel in size
            if blob_sizes[i] > 1:
                valid_labels.append(i)
                
        if valid_labels:
            # Calculate CoM only on the surviving, background-subtracted stars
            coms = center_of_mass(true_intensity, labeled_array, valid_labels)
            
            # Scipy returns a single tuple if valid_labels has 1 element, otherwise a list of tuples. We force it to a list for safe iteration.
            if not isinstance(coms, list):
                coms = [coms]
                
            for (y, x) in coms:
                centroids.append([x, y])
                
    return np.array(centroids)