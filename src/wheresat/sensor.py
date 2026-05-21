import os
import numpy as np
import matplotlib.pyplot as plt

# ---------------------------------------------------------
# 1. THE NOISE INJECTOR
# ---------------------------------------------------------
def apply_sensor_dirt(clean_image: np.ndarray, readout_sigma: float, hot_pixel_fraction: float) -> np.ndarray:
    """ 
    Adds hardware sensor noise to a clean image array. 
    Args: 
        clean_image: 2D NumPy array containing the rendered stars. 
        readout_sigma: Standard deviation for the Gaussian readout noise floor. 
        hot_pixel_fraction: Percentage of pixels to max out (e.g., 0.001 for 0.1%). 
    Returns: 
        A 2D NumPy array with the noise applied. 
    """ 
    noisy_image = clean_image.astype(np.float64).copy()
    
    # Gaussian Readout Noise
    noise = np.random.normal(loc=0.0, scale=readout_sigma, size=noisy_image.shape)
    noisy_image += noise
    
    # Hot Pixels
    total_pixels = noisy_image.size
    num_hot_pixels = int(total_pixels * hot_pixel_fraction)
    
    if num_hot_pixels > 0:
        y_coords = np.random.randint(0, noisy_image.shape[0], num_hot_pixels)
        x_coords = np.random.randint(0, noisy_image.shape[1], num_hot_pixels)
        noisy_image[y_coords, x_coords] = 255.0
        
    noisy_image = np.clip(noisy_image, 0.0, 255.0)
    
    return noisy_image

# ---------------------------------------------------------
# 2. THE VISUALIZER
# ---------------------------------------------------------
def save_and_visualize(image_array: np.ndarray, ground_truth_pixels: np.ndarray, filename: str = "synthetic_cam.png"): 
    """ 
    Saves the image array and plots it with ground truth overlays. 
    Args: 
        image_array: The final 2D dirty image array. 
        ground_truth_pixels: Nx4 array from camera.py [ID, x, y, mag]. 
        filename: Where to save the output in the data/ folder. 
    """ 
    # Route dynamically to the /data folder
    root_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    data_dir = os.path.join(root_dir, "data")
    os.makedirs(data_dir, exist_ok=True)
    filepath = os.path.join(data_dir, filename)
    
    # Setup the visualizer canvas
    plt.figure(figsize=(10, 10))
    
    # Draw the noisy hardware image. 
    # origin='upper' is REQUIRED so (0,0) is at the top-left!
    plt.imshow(image_array, cmap='gray', origin='upper')
    
    # Overlay the red ground-truth targeting circles
    if len(ground_truth_pixels) > 0:
        truth_x = ground_truth_pixels[:, 1]
        truth_y = ground_truth_pixels[:, 2]
        # facecolors='none' makes the circles hollow so we can see the star inside
        plt.scatter(truth_x, truth_y, edgecolors='red', facecolors='none', 
                    s=100, label='True Star (Math)', linewidths=1.5)
        
    plt.title("Synthetic Star Tracker - Ground Truth Verification")
    plt.legend(loc="upper right")
    
    # Save the file to the hard drive and show it on screen
    plt.savefig(filepath, bbox_inches='tight')
    print(f"[SYSTEM] Hardware simulation complete. Image saved to: {filepath}")
    plt.show()