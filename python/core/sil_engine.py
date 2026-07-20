import numpy as np
from scipy.spatial.transform import Rotation as R
from mekf import MEKF
from controls import compute_command_torque

def run_flight_grade_adcs():
    print("--- 🚀 INITIATING FLIGHT-GRADE ADCS SIMULATION ---")
    
    # Physical Chassis
    inertia_matrix = np.diag([0.1, 0.1, 0.1])  
    inv_inertia = np.linalg.inv(inertia_matrix)
    q_target = np.array([0.0, 0.0, 0.0, 1.0])
    
    # 1. The Hostile Environment & Broken Hardware
    true_omega = np.array([0.2, -0.2, 0.2]) 
    true_q = R.from_euler('xyz', [45, -30, 60], degrees=True).as_quat()
    
    # Massive Gyro Bias (Hardware is severely degraded)
    true_gyro_bias = np.array([0.05, -0.02, 0.03]) 
    
    # Constant Aerodynamic Drag / Gravity Gradient pulling the satellite
    disturbance_torque = np.array([1.5e-4, -1.0e-4, 0.5e-4]) 
    
    # Initialize 6-State MEKF
    mekf = MEKF(initial_q=np.array([0.0, 0.0, 0.0, 1.0]))
    
    dt_gyro = 0.01          
    dt_star_tracker = 0.1   
    sim_time = 45.0         
    
    steps = int(sim_time / dt_gyro)
    star_tracker_steps = int(dt_star_tracker / dt_gyro)
    
    print("\n[T=0.0s] Environment: High Aerodynamic Drag | Sensor: Severe Thermal Drift")
    print("-------------------------------------------------------------------------")
    
    for step in range(steps):
        time = step * dt_gyro
        
        # --- A. SENSOR READINGS ---
        # The true bias slowly random walks over time
        true_gyro_bias += np.random.normal(0, 1e-6, 3) * dt_gyro
        measured_omega = true_omega + true_gyro_bias + np.random.normal(0, 1e-4, 3)
        
        # --- B. MEKF (6-State Filter) ---
        mekf.predict(measured_omega, dt_gyro)
        
        if step % star_tracker_steps == 0:
            noisy_r = R.from_quat(true_q) * R.from_rotvec(np.random.normal(0, 0.0005, 3))
            mekf.update(noisy_r.as_quat(), np.eye(3) * (0.0005 ** 2))
            
        # --- C. CONTROL LOOP ---
        # We pass the BIAS-CORRECTED omega from the MEKF, not the raw hardware data
        clean_omega = measured_omega - mekf.beta
        command_torque = compute_command_torque(
            q_current=mekf.q, 
            q_target=q_target, 
            omega=clean_omega, 
            Kp=0.05, 
            Kd=0.14  
        )
        # Enforce strict hardware limits
        command_torque = np.clip(command_torque, -0.02, 0.02)
        
        # --- D. KINEMATICS (Applying the physics) ---
        gyroscopic_torque = np.cross(true_omega, inertia_matrix @ true_omega)
        
        # Notice we ADD the disturbance torque pulling against the reaction wheels
        net_torque = command_torque - gyroscopic_torque + disturbance_torque
        
        angular_accel = inv_inertia @ net_torque
        true_omega += angular_accel * dt_gyro
        dq = R.from_rotvec(true_omega * dt_gyro).as_quat()
        true_q = (R.from_quat(true_q) * R.from_quat(dq)).as_quat()
        
        # --- E. TELEMETRY ---
        if step % 500 == 0 or step == steps - 1:
            dot_product = np.clip(np.abs(np.dot(true_q, q_target)), 0, 1)
            error_deg = np.degrees(2 * np.arccos(dot_product))
            spin_rate = np.degrees(np.linalg.norm(true_omega))
            
            # Calculate how well the filter is guessing the invisible bias
            bias_error = np.linalg.norm(true_gyro_bias - mekf.beta)
            
            print(f"[T={time:04.1f}s] Pointing Error: {error_deg:05.2f}° | Spin: {spin_rate:05.2f} deg/s | Bias Est Error: {bias_error:.5f} rad/s")

    if np.degrees(np.linalg.norm(true_omega)) < 0.5 and error_deg < 1.0:
        print("\n[VERDICT] 🟢 FLIGHT-GRADE ADCS LOCKED. Bias eliminated. Drag neutralized.")
    else:
        print("\n[VERDICT] 🔴 ADCS FAILED. Satellite succumbed to the environment.")

if __name__ == "__main__":
    run_flight_grade_adcs()