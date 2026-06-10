import sys
import os
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial import cKDTree

# IAU / Hipparcos common star names keyed by HIP ID.
# Source: Hipparcos ident4.doc / celestialprogramming.com
HIP_NAMES = {
    13847: "Acamar",    7588: "Achernar",   60718: "Acrux",     33579: "Adhara",
    68702: "Agena",     95947: "Albireo",   65477: "Alcor",     17702: "Alcyone",
    21421: "Aldebaran", 105199: "Alderamin", 1067: "Algenib",   50583: "Algieba",
    14576: "Algol",     31681: "Alhena",    62956: "Alioth",    67301: "Alkaid",
    9640:  "Almaak",    109268: "Alnair",   25428: "Alnath",    26311: "Alnilam",
    26727: "Alnitak",   46390: "Alphard",   76267: "Alphekka",  677:   "Alpheratz",
    98036: "Alshain",   97649: "Altair",    2081:  "Ankaa",     80763: "Antares",
    69673: "Arcturus",  25985: "Arneb",     25336: "Bellatrix", 27989: "Betelgeuse",
    30438: "Canopus",   24608: "Capella",   746:   "Caph",      36850: "Castor",
    63125: "Cor Caroli",102098: "Deneb",    57632: "Denebola",  3419:  "Diphda",
    54061: "Dubhe",     107315: "Enif",     87833: "Etamin",    113368: "Fomalhaut",
    9884:  "Hamal",     72105: "Izar",      90185: "Kaus Aus.", 72607: "Kocab",
    113963: "Markab",   59774: "Megrez",    14135: "Menkar",    53910: "Merak",
    25930: "Mintaka",   10826: "Mira",      5447:  "Mirach",    15863: "Mirphak",
    65378: "Mizar",     25606: "Nihal",     92855: "Nunki",     58001: "Phad",
    11767: "Polaris",   37826: "Pollux",    37279: "Procyon",   70890: "Proxima",
    84345: "Rasalgethi",86032: "Rasalhague",49669: "Regulus",   24436: "Rigel",
    71683: "Rigil Kent",92420: "Sheliak",   32349: "Sirius",    65474: "Spica",
    97278: "Tarazed",   68756: "Thuban",    77070: "Unukalhai", 91262: "Vega",
    63608: "Vindemiatrix", 18543: "Zaurak",
}

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(ROOT_DIR, 'src'))

from wheresat.database import build_star_database
from wheresat.renderer import render_star_field
from wheresat.centroiding import extract_centroids
from wheresat.star_id import identify_stars
from wheresat.quest import compute_attitude_quest
from wheresat.mekf import MEKF
from wheresat.controls import compute_command_torque
from wheresat.sensor import apply_sensor_dirt
from wheresat.gaussian import extract_centroids_gaussian

def simulate_camera_optics(catalog, true_q, width=1024, fov_deg=45):
    focal_length = (width / 2) / np.tan(np.radians(fov_deg / 2))
    r_body = R.from_quat(true_q)
    eci_vectors = catalog[:, 1:4]
    body_stars = r_body.apply(eci_vectors)
    
    visible_mask = body_stars[:, 2] > 0
    visible_body = body_stars[visible_mask]
    visible_data = catalog[visible_mask]
    
    if len(visible_body) == 0:
        return np.array([]), focal_length
        
    pixels_x = (visible_body[:, 0] * focal_length / visible_body[:, 2]) + (width / 2)
    pixels_y = (visible_body[:, 1] * focal_length / visible_body[:, 2]) + (width / 2)
    
    in_frame = (pixels_x >= 0) & (pixels_x < width) & (pixels_y >= 0) & (pixels_y < width)
    
    render_data = np.column_stack((
        visible_data[in_frame, 0], pixels_x[in_frame], pixels_y[in_frame], visible_data[in_frame, 4]
    ))
    return render_data, focal_length

class SILEngine:
    def __init__(self):
        print("--- 🌍 IGNITING SIL STATE MACHINE ---")
        catalog_path = os.path.join(ROOT_DIR, "data", "optimized_catalog.npy")
        if not os.path.exists(catalog_path):
            print(f"[FATAL] Cannot find {catalog_path}.")
            sys.exit(1)
            
        self.catalog = np.load(catalog_path)
        self.database_tree, self.triangle_ids = build_star_database(catalog_path, max_fov_deg=45.0)

        # Physical Chassis
        self.inertia_matrix = np.diag([0.1, 0.1, 0.1])  
        self.inv_inertia = np.linalg.inv(self.inertia_matrix)
        self.q_target = np.array([0.0, 0.0, 0.0, 1.0])
        
        # State
        self.true_omega = np.array([0.2, -0.2, 0.2]) 
        self.true_q = R.from_euler('xyz', [45, -30, 60], degrees=True).as_quat()
        self.true_gyro_bias = np.array([0.05, -0.02, 0.03]) 
        
        # Flight Software
        self.mekf = MEKF(initial_q=np.array([0.0, 0.0, 0.0, 1.0]))
        self.adcs_locked = False
        
        # Timing
        self.dt_gyro = 0.01          
        self.dt_camera = 0.1         
        self.width = 1024
        self.step_count = 0
        self.camera_steps = int(self.dt_camera / self.dt_gyro)
        
        # GUI Memory Buffer
        self.last_image = np.zeros((self.width, self.width), dtype=np.uint16)
        self.last_extracted_count = 0
        self.last_extracted_centroids = np.array([])  # [x, y, hip_id] for the 6 used stars
        self.last_extracted_names = []                 # common name (or "HIP N") per centroid

    def step_forward(self):
        time = self.step_count * self.dt_gyro
        
        # --- A. SENSORS & PREDICTION (100Hz) ---
        self.true_gyro_bias += np.random.normal(0, 1e-6, 3) * self.dt_gyro
        measured_omega = self.true_omega + self.true_gyro_bias + np.random.normal(0, 1e-4, 3)
        self.mekf.predict(measured_omega, self.dt_gyro)
        
        # --- B. THE OPTICAL PIPELINE (10Hz) ---
        if self.step_count % self.camera_steps == 0:
            render_data, _ = simulate_camera_optics(self.catalog, self.true_q, width=self.width)
            
            if len(render_data) >= 3:
                self.last_image = render_star_field(render_data, width=self.width, sigma=1.5)
                self.last_image = apply_sensor_dirt(self.last_image, readout_sigma=50.0, hot_pixel_fraction=0.0001)
                raw_centroids = extract_centroids(self.last_image, threshold=300)
                
                if len(raw_centroids) > 6:
                    intensities = np.array([self.last_image[int(c[1]), int(c[0])] for c in raw_centroids])
                    sorted_indices = np.argsort(np.where(intensities >= 60000, 0, intensities))[::-1]
                    extracted_centroids = raw_centroids[sorted_indices][:6]
                else:
                    extracted_centroids = raw_centroids

                # Refine CoM centroids to sub-pixel accuracy via 2D Gaussian fit
                if len(extracted_centroids) > 0:
                    refined = extract_centroids_gaussian(self.last_image, extracted_centroids)
                    if len(refined) > 0:
                        extracted_centroids = refined
                    
                self.last_extracted_count = len(extracted_centroids)

                # Match each centroid back to the nearest projected star to recover HIP ID.
                # render_data columns: [hip_id, x_px, y_px, mag]
                render_tree = cKDTree(render_data[:, 1:3])
                _, idx = render_tree.query(extracted_centroids[:, :2])
                hip_ids = render_data[idx, 0].astype(int)
                names = [HIP_NAMES.get(h, f"HIP {h}") for h in hip_ids]
                # Store as [x, y, hip_id] plus a parallel names list
                self.last_extracted_centroids = np.hstack([
                    extracted_centroids[:, :2],
                    hip_ids.reshape(-1, 1)
                ])
                self.last_extracted_names = names
                
                if len(extracted_centroids) >= 3:
                    measured_body, matched_eci = identify_stars(
                        centroids=extracted_centroids, camera_width=self.width, camera_fov=45.0, 
                        kd_tree=self.database_tree, triangle_id_map=self.triangle_ids, 
                        catalog=self.catalog, tolerance=1e-3
                    )
                    
                    if len(measured_body) >= 3:
                        measured_q = compute_attitude_quest(matched_eci, measured_body)
                        dot_check = np.clip(np.abs(np.dot(self.mekf.q, measured_q)), 0.0, 1.0)
                        q_diff_deg = np.degrees(2 * np.arccos(dot_check))
                        
                        if not self.adcs_locked:
                            self.mekf.update(measured_q, np.eye(3) * (0.0005 ** 2))
                            if q_diff_deg < 10.0:
                                self.adcs_locked = True
                        else:
                            if q_diff_deg < 15.0:
                                self.mekf.update(measured_q, np.eye(3) * (0.0005 ** 2))
            else:
                self.last_image = np.zeros((self.width, self.width), dtype=np.uint16)
                self.last_extracted_count = 0
                self.last_extracted_centroids = np.array([])
                self.last_extracted_names = []
            
        # --- C. CONTROLS (100Hz) ---
        clean_omega = measured_omega - self.mekf.beta
        command_torque = compute_command_torque(self.mekf.q, self.q_target, clean_omega, Kp=0.05, Kd=0.14)
        command_torque = np.clip(command_torque, -0.02, 0.02)
        
        # --- D. KINEMATICS ---
        gyroscopic_torque = np.cross(self.true_omega, self.inertia_matrix @ self.true_omega)
        disturbance_torque = np.array([1.5e-4, -1.0e-4, 0.5e-4]) 
        
        angular_accel = self.inv_inertia @ (command_torque - gyroscopic_torque + disturbance_torque)
        self.true_omega += angular_accel * self.dt_gyro
        dq = R.from_rotvec(self.true_omega * self.dt_gyro).as_quat()
        self.true_q = (R.from_quat(self.true_q) * R.from_quat(dq)).as_quat()
        
        self.step_count += 1
        
        # --- E. TELEMETRY RETURN ---
        dot_product = np.clip(np.abs(np.dot(self.true_q, self.q_target)), 0, 1)
        error_deg = np.degrees(2 * np.arccos(dot_product))
        spin_rate = np.degrees(np.linalg.norm(self.true_omega))
        
        return {
            "time": time,
            "error_deg": error_deg,
            "spin_rate": spin_rate,
            "stars_extracted": self.last_extracted_count,
            "adcs_locked": self.adcs_locked,
            "true_q": self.true_q,
            "image": self.last_image,
            "extracted_centroids": self.last_extracted_centroids,  # [x, y, hip_id], up to 6
            "extracted_names": self.last_extracted_names,            # common name per centroid
            "gyro_bias_est": self.mekf.beta.copy()
        }