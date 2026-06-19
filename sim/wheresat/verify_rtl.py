import numpy as np
import sys

def validate_hardware_math(python_centroids_path: str, fpga_output_path: str, tolerance: float = 0.5):
    """
    Loads Python floats and Verilog fixed-point outputs to calculate hardware drift.
    """
    print("--- 🔬 RTL DIFF ENGINE ---")
    
    try:
        # Load the raw text files (assuming space or comma separated [X, Y] rows)
        py_coords = np.loadtxt(python_centroids_path)
        rtl_coords = np.loadtxt(fpga_output_path)
    except Exception as e:
        print(f"[FATAL ERROR] Could not load coordinate files: {e}")
        return

    # 1. Catastrophic Failure Check
    if len(py_coords) != len(rtl_coords):
        print(f"[FAIL] Architecture mismatch! Python found {len(py_coords)} stars, RTL found {len(rtl_coords)}.")
        sys.exit(1)

    # 2. Geometric Alignment
    # We must sort coordinates by X, then Y, to guarantee 1-to-1 matching.
    # Otherwise, we might accidentally compare Python's Star #1 to RTL's Star #4!
    py_coords = py_coords[np.lexsort((py_coords[:, 1], py_coords[:, 0]))]
    rtl_coords = rtl_coords[np.lexsort((rtl_coords[:, 1], rtl_coords[:, 0]))]

    # 3. Calculate Euclidean Distance (Drift)
    deltas = np.linalg.norm(py_coords - rtl_coords, axis=1)
    
    # 4. Enforce the Tolerance Boundary
    failures = 0
    for i, delta in enumerate(deltas):
        if delta > tolerance:
            print(f"[FAIL] Overflow Detected! Star {i} Drifted: {delta:.4f} px | Py: {py_coords[i]} -> RTL: {rtl_coords[i]}")
            failures += 1
            
    if failures == 0:
        print(f"[PASS] RTL Math Verified. Maximum hardware drift: {np.max(deltas):.4f} pixels.")
    else:
        print(f"[CRITICAL] Pipeline halted. {failures} accumulators failed verification.")
        sys.exit(1)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python verify_rtl.py <path_to_py_centroids> <path_to_rtl_centroids>")
        sys.exit(1)
        
    py_path = sys.argv[1]
    rtl_path = sys.argv[2]
    
    validate_hardware_math(py_path, rtl_path, tolerance=0.5)