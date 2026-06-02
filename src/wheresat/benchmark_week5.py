import numpy as np
from scipy.spatial.transform import Rotation as R
from wheresat.quest import compute_attitude_quest

def calculate_quaternion_error(q_true: np.ndarray, q_est: np.ndarray) -> float:
    """
    Calculates the absolute angular error (in degrees) between two quaternions.
    Because q and -q represent the same 3D rotation, we use the absolute dot product.
    """
    dot_product = np.clip(np.abs(np.dot(q_true, q_est)), 0.0, 1.0)
    # The angle between quaternions is theta/2, so we multiply by 2
    error_rad = 2 * np.arccos(dot_product)
    return np.degrees(error_rad)

def run_quest_proving_ground():
    print("--- 🚀 INITIATING QUEST ALGORITHM VALIDATION ---")
    
    # 1. Generate a synthetic Star Catalog (15 random Guide Stars)
    np.random.seed(42) # Lock the seed for deterministic testing
    num_stars = 15
    eci_vectors = np.random.randn(num_stars, 3)
    eci_vectors /= np.linalg.norm(eci_vectors, axis=1, keepdims=True)
    
    # 2. Phase 1: The Identity Test (Zero Rotation)
    # If the camera sees the exact ECI coordinates, the quaternion MUST be [0, 0, 0, 1]
    q_identity = compute_attitude_quest(eci_vectors, eci_vectors)
    print(f"\n[Phase 1: Identity Matrix]")
    print(f"Expected: [0. 0. 0. 1.]")
    print(f"Calculated: {np.round(q_identity, 4)}")
    
    # 3. Phase 2: The Perfect Rotation Test
    # We physically spin the virtual satellite by exactly 45 degrees around the X-axis
    true_rotation = R.from_euler('x', 45, degrees=True)
    true_quat = true_rotation.as_quat() # [x, y, z, w]
    
    # Rotate the ECI vectors to simulate what the clean camera sees
    clean_body_vectors = true_rotation.apply(eci_vectors)
    
    q_clean = compute_attitude_quest(eci_vectors, clean_body_vectors)
    clean_error = calculate_quaternion_error(true_quat, q_clean)
    
    print(f"\n[Phase 2: Perfect Geometry]")
    print(f"Target Quat: {np.round(true_quat, 4)}")
    print(f"QUEST Quat:  {np.round(q_clean, 4)}")
    print(f"Angular Error: {clean_error:.6f} degrees")
    
    # 4. Phase 3: The Radiation Stress Test (Least-Squares Verification)
    # We inject raw Gaussian blur into the body vectors to simulate thermal noise and optical distortion
    # Noise standard deviation of 0.0005 radians (~100 arcseconds of hardware error)
    noisy_body_vectors = clean_body_vectors + np.random.normal(0, 0.0005, clean_body_vectors.shape)
    
    # Re-normalize because real cameras only output unit vectors
    noisy_body_vectors /= np.linalg.norm(noisy_body_vectors, axis=1, keepdims=True)
    
    q_noisy = compute_attitude_quest(eci_vectors, noisy_body_vectors)
    noisy_error = calculate_quaternion_error(true_quat, q_noisy)
    
    print(f"\n[Phase 3: Hardware Noise Injection (15 Stars)]")
    print(f"Target Quat: {np.round(true_quat, 4)}")
    print(f"QUEST Quat:  {np.round(q_noisy, 4)}")
    print(f"Angular Error: {noisy_error:.6f} degrees")
    
    if noisy_error < 0.1:
        print("\n[VERDICT] 🟢 QUEST Algorithm Verified. Least-Squares optimization is crushing the noise.")
    else:
        print("\n[VERDICT] 🔴 QUEST FAILED. Algorithm cannot handle hardware degradation.")

if __name__ == "__main__":
    run_quest_proving_ground()