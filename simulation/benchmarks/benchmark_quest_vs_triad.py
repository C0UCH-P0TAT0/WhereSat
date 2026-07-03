import numpy as np
from scipy.spatial.transform import Rotation as R
from wheresat.quest import compute_attitude_quest
from wheresat.triad import compute_attitude_triad

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
    print("--- 🚀 INITIATING ATTITUDE DETERMINATION VALIDATION ---")
    
    # 1. Generate a synthetic Star Catalog (15 random Guide Stars)
    np.random.seed(42) # Lock the seed for deterministic testing
    num_stars = 15
    eci_vectors = np.random.randn(num_stars, 3)
    eci_vectors /= np.linalg.norm(eci_vectors, axis=1, keepdims=True)
    
    # Slice the top 2 stars for TRIAD's deterministic math
    triad_eci = eci_vectors[:2]
    
    # ---------------------------------------------------------
    # Phase 1: The Identity Test (Zero Rotation)
    # ---------------------------------------------------------
    q_identity_quest = compute_attitude_quest(eci_vectors, eci_vectors)
    
    triad_identity_dcm = compute_attitude_triad(triad_eci, triad_eci)
    q_identity_triad = R.from_matrix(triad_identity_dcm).as_quat()
    
    print(f"\n[Phase 1: Identity Matrix]")
    print(f"Expected:   [0. 0. 0. 1.]")
    print(f"QUEST Calc: {np.round(q_identity_quest, 4)}")
    print(f"TRIAD Calc: {np.round(q_identity_triad, 4)}")
    
    # ---------------------------------------------------------
    # Phase 2: The Perfect Rotation Test
    # ---------------------------------------------------------
    # We physically spin the virtual satellite by exactly 45 degrees around the X-axis
    true_rotation = R.from_euler('x', 45, degrees=True)
    true_quat = true_rotation.as_quat() # [x, y, z, w]
    
    # Rotate the ECI vectors to simulate what the clean camera sees
    clean_body_vectors = true_rotation.apply(eci_vectors)
    triad_clean_body = clean_body_vectors[:2]
    
    q_clean_quest = compute_attitude_quest(eci_vectors, clean_body_vectors)
    clean_error_quest = calculate_quaternion_error(true_quat, q_clean_quest)
    
    triad_clean_dcm = compute_attitude_triad(triad_eci, triad_clean_body)
    q_clean_triad = R.from_matrix(triad_clean_dcm).as_quat()
    clean_error_triad = calculate_quaternion_error(true_quat, q_clean_triad)
    
    print(f"\n[Phase 2: Perfect Geometry]")
    print(f"Target Quat: {np.round(true_quat, 4)}")
    print(f"QUEST Quat:  {np.round(q_clean_quest, 4)} | Error: {clean_error_quest:.6f} deg")
    print(f"TRIAD Quat:  {np.round(q_clean_triad, 4)} | Error: {clean_error_triad:.6f} deg")
    
    # ---------------------------------------------------------
    # Phase 3: The Radiation Stress Test (Hardware Noise Injection)
    # ---------------------------------------------------------
    # Noise standard deviation of 0.0005 radians (~100 arcseconds of hardware error)
    noisy_body_vectors = clean_body_vectors + np.random.normal(0, 0.0005, clean_body_vectors.shape)
    noisy_body_vectors /= np.linalg.norm(noisy_body_vectors, axis=1, keepdims=True)
    triad_noisy_body = noisy_body_vectors[:2]
    
    q_noisy_quest = compute_attitude_quest(eci_vectors, noisy_body_vectors)
    noisy_error_quest = calculate_quaternion_error(true_quat, q_noisy_quest)
    
    triad_noisy_dcm = compute_attitude_triad(triad_eci, triad_noisy_body)
    
    # Check for the identity matrix hallucination in TRIAD
    if np.array_equal(triad_noisy_dcm, np.eye(3)):
        print("\n   -> [FAIL] TRIAD choked and returned an Identity hallucination.")
        noisy_error_triad = 999.0
        q_noisy_triad = np.array([0., 0., 0., 1.])
    else:
        q_noisy_triad = R.from_matrix(triad_noisy_dcm).as_quat()
        noisy_error_triad = calculate_quaternion_error(true_quat, q_noisy_triad)
        
    print(f"\n[Phase 3: Hardware Noise Injection]")
    print(f"Target Quat: {np.round(true_quat, 4)}")
    print(f"QUEST Quat:  {np.round(q_noisy_quest, 4)}")
    print(f"TRIAD Quat:  {np.round(q_noisy_triad, 4)}")
    
    # ---------------------------------------------------------
    # Phase 4: The Showdown
    # ---------------------------------------------------------
    print(f"\n[Phase 4: The Drag Race Summary]")
    print(f"TRIAD Error (2 stars):   {noisy_error_triad:.6f} degrees")
    print(f"QUEST Error (15 stars):  {noisy_error_quest:.6f} degrees")
    
    if noisy_error_quest < noisy_error_triad:
        print("\n[VERDICT] 🟢 QUEST WINS")
    else:
        print("\n[VERDICT] 🔴 TRIAD WINS")

if __name__ == "__main__":
    run_quest_proving_ground()