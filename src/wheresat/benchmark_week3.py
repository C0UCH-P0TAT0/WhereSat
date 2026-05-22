import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist
from scipy.spatial.transform import Rotation as R

# Import your simulation modules
from wheresat.coordinates import eci_to_body
from wheresat.camera import generate_image
from wheresat.renderer import render_star_field
from wheresat.sensor import apply_sensor_dirt

# Import the Week 3 algorithms
# pyrefly: ignore [missing-import]
from wheresat.centroiding import extract_centroids
from wheresat.gaussian import extract_centroids_gaussian

def calculate_centroid_error(calculated_centroids: np.ndarray, truth_pixels: np.ndarray) -> float:
    if len(calculated_centroids) == 0 or len(truth_pixels) == 0:
        return np.nan

    truth_coords = truth_pixels[:, 1:3]
    distances = cdist(calculated_centroids, truth_coords)
    min_distances = np.min(distances, axis=1)

    valid_errors = min_distances[min_distances < 5.0]

    if len(valid_errors) == 0:
        return np.nan

    return float(np.mean(valid_errors))

def plot_snr_vs_error():
    print("--- 🚀 INITIATING ALGORITHM BENCHMARK ---")
    
    catalog_path = os.path.join(os.path.abspath(os.path.dirname(__file__)), "data", "optimized_catalog.npy")
    catalog = np.load(catalog_path)
    eci_vectors = catalog[:, 1:4]
    
    camera_width = 1024
    camera_fov = 12.0
    
    noise_levels = np.linspace(5.0, 50.0, 10)
    gaussian_errors = []
    com_errors = []

    for noise in noise_levels:
        print(f"Testing Camera Noise Level (Sigma): {noise:.1f}...")
        
        sat_quaternion = R.random().as_quat()
        body_vectors = eci_to_body(eci_vectors, sat_quaternion)
        
        body_data = np.column_stack((catalog[:, 0], body_vectors, catalog[:, 4]))
        truth_pixels = generate_image(body_data, camera_width, camera_fov)
        
        if len(truth_pixels) == 0:
            com_errors.append(np.nan)
            gaussian_errors.append(np.nan)
            continue
            
        clean_image = render_star_field(truth_pixels, camera_width, sigma=1.5)
        dirty_image = apply_sensor_dirt(clean_image, readout_sigma=noise, hot_pixel_fraction=0.001)

        com_centroids = extract_centroids(dirty_image, threshold=200)
        gaussian_centroids = extract_centroids_gaussian(dirty_image, com_centroids)

        err_com = calculate_centroid_error(com_centroids, truth_pixels)
        err_gaussian = calculate_centroid_error(gaussian_centroids, truth_pixels)

        com_errors.append(err_com)
        gaussian_errors.append(err_gaussian)

    plt.figure(figsize=(10, 6))
    
    plt.plot(noise_levels, gaussian_errors, marker='o', color='blue', linewidth=2, label='2D Gaussian Fit (Yash)')
    plt.plot(noise_levels, com_errors, marker='x', color='red', linewidth=2, linestyle='--', label='Center of Mass (Malpani)')
    
    plt.title('Star Tracker Algorithm Robustness: Error vs. Camera Noise', fontsize=14, fontweight='bold')
    plt.xlabel('Camera Readout Noise (Sigma)', fontsize=12)
    plt.ylabel('Mean Centroid Error (Pixels)', fontsize=12)
    plt.grid(True, linestyle=':', alpha=0.7)
    plt.legend(fontsize=12)
    plt.ylim(bottom=0)
    
    plt.savefig('algorithm_benchmark_results.png', bbox_inches='tight')
    print("\n[SYSTEM] Benchmark Complete. Graph saved.")

if __name__ == "__main__":
    plot_snr_vs_error()