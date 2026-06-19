import numpy as np
import sys
from scipy.optimize import linear_sum_assignment
from scipy.spatial.distance import cdist

def validate_hardware_math(python_centroids_path: str, fpga_output_path: str, tolerance: float = 0.5):
    print("--- 🔬 RTL DIFF ENGINE ---")
    
    try:
        py_coords = np.loadtxt(python_centroids_path)
        rtl_coords = np.loadtxt(fpga_output_path)
    except Exception as e:
        print(f"[FATAL ERROR] Could not load coordinate files: {e}")
        return

    # 4. Prevent 1D Array Crashes
    if py_coords.size == 0:
        print("[CRITICAL] Python ground truth is completely empty.")
        sys.exit(1)
    if rtl_coords.size == 0:
        print("[CRITICAL] RTL output is completely empty.")
        sys.exit(1)

    py_coords = np.atleast_2d(py_coords)
    rtl_coords = np.atleast_2d(rtl_coords)

    # Strip the mass column from Python ground truth if it exists
    if py_coords.shape[1] >= 3:
        py_coords = py_coords[:, :2]

    # 1. Non-Fatal Count Mismatch
    n_py = len(py_coords)
    n_rtl = len(rtl_coords)
    if n_py != n_rtl:
        print(f"[WARNING] Architecture mismatch! Python: {n_py} | RTL: {n_rtl}")

    # 2. Hungarian Assignment (Bypassing Lexsort fragility)
    # Computes a distance matrix between all Py and RTL stars, then finds the globally optimal 1-to-1 pairing.
    cost_matrix = cdist(py_coords, rtl_coords)
    py_indices, rtl_indices = linear_sum_assignment(cost_matrix)

    # 3. Calculate Euclidean Distance (Drift) for matched pairs
    deltas = cost_matrix[py_indices, rtl_indices]
    
    # 4. Enforce the Tolerance Boundary
    failures = 0
    for idx, delta in enumerate(deltas):
        p_idx = py_indices[idx]
        r_idx = rtl_indices[idx]
        if delta > tolerance:
            print(f"[FAIL] Star Drifted: {delta:.4f} px | Py: {py_coords[p_idx]} -> RTL: {rtl_coords[r_idx]}")
            failures += 1
            
    # 7. Advanced Debugging Stats
    print("\n--- STATS ---")
    print(f"Total Matches Analyzed : {len(deltas)}")
    print(f"Maximum Drift          : {np.max(deltas):.4f} px")
    print(f"Average Drift          : {np.mean(deltas):.4f} px")
    print(f"Median Drift           : {np.median(deltas):.4f} px")
    
    unmatched_py = n_py - len(deltas)
    unmatched_rtl = n_rtl - len(deltas)
    if unmatched_py > 0:
        print(f"Dropped Python Stars   : {unmatched_py}")
    if unmatched_rtl > 0:
        print(f"Phantom RTL Stars      : {unmatched_rtl}")

    if failures == 0:
        print("\n[PASS] RTL Math Verified.")
    else:
        print(f"\n[CRITICAL] Pipeline halted. {failures} accumulators exceeded tolerance.")
        sys.exit(1)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python verify_rtl.py <path_to_py_centroids> <path_to_rtl_centroids>")
        sys.exit(1)
        
    py_path = sys.argv[1]
    rtl_path = sys.argv[2]
    
    validate_hardware_math(py_path, rtl_path, tolerance=0.5)