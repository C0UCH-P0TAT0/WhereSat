import numpy as np
from scipy.spatial.transform import Rotation as R
from wheresat.mekf import MEKF
from wheresat.controls import compute_command_torque

def run_closed_loop_adcs():
    print("--- 🚀 INITIATING ADCS CLOSED-LOOP SIMULATION ---")
    
    # 1. Physical Satellite Properties (10x10x10cm CubeSat)
    inertia_matrix = np.diag([0.1, 0.1, 0.1])  
    inv_inertia = np.linalg.inv(inertia_matrix)
    
    # Target attitude: [0, 0, 0, 1] (Perfectly aligned with ECI zero-point)
    q_target = np.array([0.0, 0.0, 0.0, 1.0])
    
    # 2. Initial Tumbling State (The Physical Reality)
    # The satellite is violently tumbling at ~20 degrees per second
    true_omega = np.array([0.2, -0.2, 0.2]) 
    true_q = R.from_euler('xyz', [45, -30, 60], degrees=True).as_quat()
    
    # 3. Initialize MEKF
    # The flight computer boots up with no idea where it is, assuming [0,0,0,1]
    mekf = MEKF(initial_q=np.array([0.0, 0.0, 0.0, 1.0]), initial_cov=0.1)
    
    # 4. Timing Architecture
    dt_gyro = 0.01          # 100 Hz Gyro / Control Loop
    dt_star_tracker = 0.1   # 10 Hz Star Tracker Loop
    sim_time = 15.0         # Run for 15 seconds
    
    steps = int(sim_time / dt_gyro)
    star_tracker_steps = int(dt_star_tracker / dt_gyro)
    
    print("\n[T=0.0s] Satellite Deployed. Tumbling at ~20 deg/s. Star Tracker blind.")
    print("-------------------------------------------------------------------------")
    
    for step in range(steps):
        time = step * dt_gyro
        
        # --- A. SENSOR MODELS (Hardware Noise Injection) ---
        gyro_noise = np.random.normal(0, 0.001, 3)
        measured_omega = true_omega + gyro_noise
        
        # --- B. MEKF PREDICTION (100 Hz) ---
        mekf.predict(measured_omega, dt_gyro)
        
        # --- C. STAR TRACKER UPDATE (10 Hz) ---
        if step % star_tracker_steps == 0:
            # Simulate QUEST algorithm output + optical blur
            noisy_r = R.from_quat(true_q) * R.from_rotvec(np.random.normal(0, 0.0005, 3))
            measured_q = noisy_r.as_quat()
            
            # Feed the absolute coordinate frame to the Kalman Filter
            R_cam = np.eye(3) * (0.0005 ** 2)
            mekf.update(measured_q, R_cam)
            
        # --- D. YASH'S PD CONTROL LOOP ---
        # We feed Yash the CLEAN estimated state from your MEKF, not the noisy sensors
        command_torque = compute_command_torque(
            q_current=mekf.q, 
            q_target=q_target, 
            omega=measured_omega, 
            Kp=0.05, 
            Kd=0.02
        )
        
        # --- E. PHYSICAL UNIVERSE KINEMATICS ---
        # Apply the electrical torque to the physical chassis (Euler's rigid body dynamics)
        gyroscopic_torque = np.cross(true_omega, inertia_matrix @ true_omega)
        angular_accel = inv_inertia @ (command_torque - gyroscopic_torque)
        
        true_omega += angular_accel * dt_gyro
        
        # Rotate the true satellite attitude forward in time
        dq = R.from_rotvec(true_omega * dt_gyro).as_quat()
        true_q = (R.from_quat(true_q) * R.from_quat(dq)).as_quat()
        
        # --- F. TELEMETRY OUTPUT ---
        if step % 300 == 0 or step == steps - 1:
            # Calculate absolute pointing error
            dot_product = np.clip(np.abs(np.dot(true_q, q_target)), 0, 1)
            error_deg = np.degrees(2 * np.arccos(dot_product))
            spin_rate = np.degrees(np.linalg.norm(true_omega))
            torque_mNm = np.linalg.norm(command_torque) * 1000
            
            print(f"[T={time:04.1f}s] Error: {error_deg:05.2f}° | Spin: {spin_rate:05.2f} deg/s | Torque: {torque_mNm:04.1f} mNm")

    # Final Grading
    final_spin = np.degrees(np.linalg.norm(true_omega))
    if final_spin < 0.5 and error_deg < 1.0:
        print("\n[VERDICT] 🟢 ADCS LOCKED. Tumbling arrested. Pointing stable.")
    else:
        print("\n[VERDICT] 🔴 ADCS FAILED. Satellite is unstable.")

if __name__ == "__main__":
    run_closed_loop_adcs()