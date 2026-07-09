import os, sys
import numpy as np

# ==========================================
# PATH ROUTING
# ==========================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
sys.path.append(os.path.join(ROOT_DIR, "simulation"))

from wheresat.mekf import MEKF
from wheresat.controls import compute_command_torque

def main():
    print("==================================================")
    print(" MEKF CONVERGENCE VERIFICATION")
    print("==================================================\n")
    
    dt = 1.0
    gyro_data = np.array([0.01, -0.02, 0.005])
    target_q = np.array([0.0, 0.0, 0.0, 1.0]) 
    quest_q = np.array([0.599655, -0.449512, -0.387009, 0.537193])  # Simulated QUEST output for testing
    
    mekf = MEKF(initial_q=np.array([0.0, 0.0, 0.0, 1.0]))
    
    mekf.P = np.diag([0.1, 0.1, 0.1, 0.01, 0.01, 0.01]) 
    mekf.Q_v = 1e-8                                     
    mekf.Q_u = 3e-6                                     
    r_noise = 0.00000025                                
    
    frame = 1
    previous_bias = np.array([0.0, 0.0, 0.0])
    
    print("Running MEKF until Bias stabilizes (This might take a few hundred frames)...\n")
    
    while True:
        mekf.predict(gyro_data, dt) 
        mekf.update(quest_q, np.eye(3) * r_noise)
        
        clean_omega = gyro_data - mekf.beta
        torque = compute_command_torque(mekf.q, target_q, clean_omega, Kp=0.05, Kd=0.14)
        torque = np.clip(torque, -0.02, 0.02) 
        
        # Print the first 5 frames
        if frame <= 5:
            print(f"--- FRAME {frame} ---")
            print(f"[MEKF] Att [x,y,z,w]: [{mekf.q[0]:.4f}, {mekf.q[1]:.4f}, {mekf.q[2]:.4f}, {mekf.q[3]:.4f}]")
            print(f"[MEKF] Bias [rad/s]:  [{mekf.beta[0]:.5f}, {mekf.beta[1]:.5f}, {mekf.beta[2]:.5f}]")
            print(f"[CTRL] Torque [Nm]:   [{torque[0]:.3f}, {torque[1]:.3f}, {torque[2]:.3f}]\n")

        # Print a progress update every 50 frames so you can watch it learn
        if frame % 50 == 0:
            print(f"... Frame {frame} ... Current Bias: [{mekf.beta[0]:.5f}, {mekf.beta[1]:.5f}, {mekf.beta[2]:.5f}]")

        # Check for Convergence (Has the bias stopped changing?)
        bias_change = np.linalg.norm(mekf.beta - previous_bias)
        
        if bias_change < 0.00001:
            print("\n==================================================")
            print(f" >>> MEKF FULLY CONVERGED AT FRAME {frame}! <<<")
            print("==================================================")
            # ---> ADDED THE ATTITUDE PRINT STATEMENT HERE <---
            print(f"Final Attitude [x,y,z,w]: [{mekf.q[0]:.4f}, {mekf.q[1]:.4f}, {mekf.q[2]:.4f}, {mekf.q[3]:.4f}]")
            print(f"Final Learned Bias:       [{mekf.beta[0]:.5f}, {mekf.beta[1]:.5f}, {mekf.beta[2]:.5f}]")
            print(f"Actual Fake Gyro:         [{gyro_data[0]:.5f}, {gyro_data[1]:.5f}, {gyro_data[2]:.5f}]")
            print(f"Final Torque:             [{torque[0]:.3f}, {torque[1]:.3f}, {torque[2]:.3f}]")
            print("==================================================\n")
            break
            
        # Increased failsafe to 2000 frames
        if frame >= 2000:
            print("\n>>> WARNING: MEKF did not converge after 2000 frames! <<<")
            break
            
        previous_bias = mekf.beta.copy()
        frame += 1

if __name__ == "__main__":
    main()