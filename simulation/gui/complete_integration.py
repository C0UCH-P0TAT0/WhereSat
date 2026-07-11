import numpy as np
import serial
import subprocess
import os
import struct
import time
import itertools
import pickle
from scipy.spatial.transform import Rotation as R
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg

# Import your modules
from wheresat.renderer import render_star_field
from wheresat.star_id import identify_stars, calculate_triangle_fingerprint, pixels_to_vectors
np.set_printoptions(suppress=True, precision=6, floatmode='fixed')

# --- 0. Control Center Toggles ---
TORQUE_POLARITY = 1.0 

# --- 1. Protocol Constants ---
SOF_HOST              = 0x55  
SOF_TELEM             = 0xAA  
# UPDATED PACKET SIZE: 1 (SOF) + 18*4 (floats) + 2*1 (uint8) + 2 (CRC) = 77 bytes
TELEMETRY_PACKET_SIZE = 77    
MAX_CENTROIDS         = 26    

# --- 2. System & Camera Constants ---
SERIAL_PORT = 'COM12' 
BAUD_RATE   = 115200
HIL_DT      = 0.1 
IMAGE_WIDTH = 1024
CAMERA_FOV  = 25.0
FOCAL_LENGTH    = (IMAGE_WIDTH / 2.0) / np.tan(np.radians(CAMERA_FOV) / 2.0)
PRINCIPAL_POINT = 512.0

# --- 3. Paths ---
XSIM_DIR       = r"C:\Users\DELL\Desktop\WhereSat\fpga\build\WhereSat\WhereSat.sim\sim_1\behav\xsim"
CATALOG_PATH   = r"C:\Users\DELL\Desktop\WhereSat\data\optimized_catalog.npy"
DB_TREE_PATH   = r"C:\Users\DELL\Desktop\WhereSat\data\triangle_tree.pkl" 
DB_MAP_PATH    = r"C:\Users\DELL\Desktop\WhereSat\data\triangle_id_map.npy"
MEM_OUT_PATH   = os.path.join(XSIM_DIR, "tb_frame.mem")
CENTROIDS_PATH = os.path.join(XSIM_DIR, "rtl_centroids.txt")
VIVADO_BIN     = r"C:\AMDDesignTools\2025.2\Vivado\bin"

env = os.environ.copy()
env["PATH"] = VIVADO_BIN + ";" + env["PATH"]

# --- 4. Physics Constants ---
J = np.diag([0.05, 0.05, 0.02])
J_INV = np.linalg.inv(J)

# ==========================================================
# Protocol Helpers
# ==========================================================

def crc16_ccitt(data: bytes):
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000: crc = (crc << 1) ^ 0x1021
            else: crc <<= 1
            crc &= 0xFFFF
    return crc

def build_host_packet(centroids, gyro_vec):
    if centroids.size == 0: centroids = np.empty((0, 3))
    count = min(len(centroids), MAX_CENTROIDS)
    header = struct.pack("<BB", SOF_HOST, count)
    gyro = struct.pack("<3f", *gyro_vec)
    payload = bytearray()
    for i in range(count):
        payload += struct.pack("<3f", float(centroids[i, 0]), float(centroids[i, 1]), float(centroids[i, 2]))
    full_msg = header + gyro + payload
    return full_msg + struct.pack("<H", crc16_ccitt(full_msg))

def unpack_telemetry(data):
    if len(data) != TELEMETRY_PACKET_SIZE: return None
    received_crc = struct.unpack("<H", data[75:77])[0]
    if crc16_ccitt(data[:75]) != received_crc: return None
    
    # Unpack 18 floats and 2 uint8s
    res = struct.unpack("<18fBB", data[1:75])
    
    return {
        'q_est': np.array(res[0:4]), 
        'w_est': np.array(res[4:7]), 
        'torque': np.array(res[7:10]), 
        'q_quest': np.array(res[10:14]),
        'gyro_meas': np.array(res[14:17]),
        'innovation_dot': res[17],
        'locked': res[18], 
        'count': res[19]
    }

# ==========================================================
# Simulation Logic
# ==========================================================

class HILSimulation(QtCore.QThread):
    sig_update = QtCore.pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            print(f"[INIT] Serial Connected: {SERIAL_PORT}")
        except: 
            self.ser = None
            
        self.catalog = np.load(CATALOG_PATH)
        try:
            with open(DB_TREE_PATH, 'rb') as f: self.db_tree = pickle.load(f)
            self.db_map = np.load(DB_MAP_PATH)
        except: self.db_tree = None

        self.q = np.array([0.855121, -0.458259, -0.238438, 0.043744])  # Initial quaternion (45-degree rotation around Y-axis)
        self.w = np.array([0.01, 0.01, 0.0])
        self.running = True
        self.frame_id = 0

    def run(self):
        if not self.ser: return

        while self.running:
            self.frame_id += 1
            print(f"\n[FRAME {self.frame_id}] Starting Cycle...")

            # 1. Project & Render based on TRUE physics state
            visible_stars = self.project_stars()
            
            # --- ADD THIS TO PRINT GROUND TRUTH ---
            true_hip_ids = [int(star[0]) for star in visible_stars]
            print(f"  [TRUTH] Projected HIP IDs: {true_hip_ids}")
            # --------------------------------------

            img = render_star_field(visible_stars, IMAGE_WIDTH)
            print(f"  Step 1: Render OK ({len(visible_stars)} stars projected)")
            
            # 2. Save .mem
            if os.path.exists(CENTROIDS_PATH): os.remove(CENTROIDS_PATH)
            with open(MEM_OUT_PATH, 'w') as f:
                for px in img.flatten(): f.write(f"{px:04X}\n")
                f.flush(); os.fsync(f.fileno())

            # 3. Run FPGA Simulation
            subprocess.run("simulate.bat", cwd=XSIM_DIR, shell=True, env=env, capture_output=True)
            print(f"  Step 2: XSIM OK")
            
            # 4. Parse RTL Centroids
            rtl_centroids = np.empty((0, 3))
            if os.path.exists(CENTROIDS_PATH):
                try:
                    rtl_centroids = np.loadtxt(CENTROIDS_PATH)
                    if rtl_centroids.ndim == 1 and rtl_centroids.size > 0:
                        rtl_centroids = rtl_centroids.reshape(1, -1)
                except: pass
            print(f"  Step 3: RTL Parse OK ({len(rtl_centroids)} centroids found)")

            # 5. Handshake with STM32 (Send TRUE angular velocity as raw gyro)
            self.ser.reset_input_buffer()
            measured_w = self.w
            self.ser.write(build_host_packet(rtl_centroids, measured_w))
            self.ser.flush()
            print(f"  Step 4: Data Sent to STM32")

            # 6. Wait for Telemetry
            start_wait = time.time()
            telem_received = False
            while (time.time() - start_wait) < 60.0:
                byte = self.ser.read(1)
                if byte and byte[0] == SOF_TELEM:
                    raw_telem = byte + self.ser.read(TELEMETRY_PACKET_SIZE - 1)
                    telem = unpack_telemetry(raw_telem)
                    if telem:
                        # Physics Update: Use true plant states
                        applied_torque = telem['torque'] * TORQUE_POLARITY
                        dw = J_INV @ (applied_torque - np.cross(self.w, J @ self.w))
                        self.w += dw * HIL_DT
                        
                        # Integrate true quaternion
                        self.q = (R.from_quat(self.q) * R.from_rotvec(self.w * HIL_DT)).as_quat()
                        self.q /= np.linalg.norm(self.q) 
                        
                        # Bundle true states for UI analysis
                        telem['q_true'] = self.q
                        telem['w_true'] = self.w
                        telem['image'] = img
                        telem['frame_id'] = self.frame_id
                        self.sig_update.emit(telem)
                        telem_received = True
                        print(f"  Step 5: Telemetry OK (Locked: {telem['locked']})")
                    break
            
            if not telem_received:
                print(f"  [ERROR] STM32 Timeout on Frame {self.frame_id}")

    def project_stars(self):
        rot = R.from_quat(self.q).inv()
        body_vectors = rot.apply(self.catalog[:, 1:4])
        mask = body_vectors[:, 2] > 1e-5
        visible_vecs = body_vectors[mask]
        x_pix = PRINCIPAL_POINT + FOCAL_LENGTH * (visible_vecs[:, 0] / visible_vecs[:, 2])
        y_pix = PRINCIPAL_POINT - FOCAL_LENGTH * (visible_vecs[:, 1] / visible_vecs[:, 2])
        in_fov = (x_pix >= 0) & (x_pix < IMAGE_WIDTH) & (y_pix >= 0) & (y_pix < IMAGE_WIDTH)
        return np.column_stack((self.catalog[mask, 0][in_fov], x_pix[in_fov], y_pix[in_fov], self.catalog[mask, 4][in_fov]))

    def stop(self):
        self.running = False
        if self.ser: self.ser.close()

class WhereSatGUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("WhereSat HIL Control Center")
        self.resize(1100, 700)
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)
        self.img_view = pg.ImageView()
        self.img_view.ui.histogram.hide(); self.img_view.ui.roiBtn.hide(); self.img_view.ui.menuBtn.hide()
        layout.addWidget(self.img_view, stretch=2)
        self.telem_text = QtWidgets.QTextEdit()
        self.telem_text.setReadOnly(True); self.telem_text.setFont(QtGui.QFont("Consolas", 10))
        layout.addWidget(self.telem_text, stretch=1)
        self.sim = HILSimulation()
        self.sim.sig_update.connect(self.update_ui)
        self.sim.start()

    def update_ui(self, data):
        self.img_view.setImage(data['image'].T, autoLevels=True)
        
        q_target = np.array([0.855121, -0.458259, -0.238438, 0.043744])
        
        q_true  = data['q_true']
        q_quest = data['q_quest']
        q_est   = data['q_est']

        # Dot Products (use abs because q and -q are identical attitudes)
        dot_target_true  = abs(np.dot(q_target, q_true))
        dot_target_quest = abs(np.dot(q_target, q_quest))
        dot_target_est   = abs(np.dot(q_target, q_est))

        dot_true_quest   = abs(np.dot(q_true, q_quest))
        dot_true_est     = abs(np.dot(q_true, q_est))
        dot_quest_est    = abs(np.dot(q_quest, q_est))

        info = (
            f"================== WHERESAT ADCS ==================\n"
            f"Frame:            {data['frame_id']}\n"
            f"Stars Matched:    {data['count']}\n"
            f"Locked:           {'YES' if data['locked'] else 'NO'}\n\n"

            f"================= QUATERNIONS =====================\n"
            f"Target : {np.round(q_target, 6)}\n"
            f"True   : {np.round(q_true, 6)}\n"
            f"QUEST  : {np.round(q_quest, 6)}\n"
            f"MEKF   : {np.round(q_est, 6)}\n\n"

            f"================= DOT PRODUCTS ===================\n"
            f"Target <-> True  : {dot_target_true:.6f}\n"
            f"Target <-> QUEST : {dot_target_quest:.6f}\n"
            f"Target <-> MEKF  : {dot_target_est:.6f}\n\n"

            f"True   <-> QUEST : {dot_true_quest:.6f}\n"
            f"True   <-> MEKF  : {dot_true_est:.6f}\n"
            f"QUEST  <-> MEKF  : {dot_quest_est:.6f}\n\n"

            f"================ ANGULAR RATES ===================\n"
            f"True Rate    : {np.round(data['w_true'], 6)}\n"
            f"Raw Gyro     : {np.round(data['gyro_meas'], 6)}\n\n"

            f"=================== CONTROL ======================\n"
            f"Torque : {np.round(data['torque'], 6)}\n"
            f"Innovation Dot : {data['innovation_dot']:.6f}"
        )
        
        self.telem_text.setText(info)

    def closeEvent(self, event):
        self.sim.stop(); self.sim.wait(); event.accept()

if __name__ == "__main__":
    app = QtWidgets.QApplication([]); gui = WhereSatGUI(); gui.show(); app.exec_()
