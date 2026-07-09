import numpy as np
import serial
import subprocess
import os
import struct
import time
from scipy.spatial.transform import Rotation as R
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg

# Import your unchanged renderer
from wheresat.renderer import render_star_field

# --- System Constants ---
SERIAL_PORT = 'COM3' 
BAUD_RATE   = 115200
HIL_DT      = 0.1  # 10Hz loop to match STM32

# --- Camera & Image Constants ---
IMAGE_WIDTH      = 1024
FOCAL_LENGTH     = 1024.0
PRINCIPAL_POINT  = 512.0
MAX_CENTROIDS    = 26

# --- Protocol Constants ---
SOF_HOST              = 0x55
SOF_TELEM             = 0xAA
TELEMETRY_PACKET_SIZE = 45  # [SOF][4Q][3W][3T][L][C][CRC16]

# --- Paths ---
XSIM_DIR     = r"C:\Users\a\Desktop\WhereSat\fpga\build\WhereSat\WhereSat.sim\sim_1\behav\xsim"
CATALOG_PATH = r"C:\Users\a\Desktop\WhereSat\data\optimized_catalog.npy"
MEM_OUT_PATH = os.path.join(XSIM_DIR, "tb_frame.mem")
CENTROIDS_PATH = os.path.join(XSIM_DIR, "rtl_centroids.txt")

# --- Physics Constants (3U CubeSat) ---
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
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

def build_host_packet(centroids, gyro_vec):
    """
    Format: [0x55][count][wx, wy, wz][x0, y0, ...][CRC16]
    """
    if centroids.size == 0:
        centroids = np.empty((0, 2))
        
    count = min(len(centroids), MAX_CENTROIDS)
    header = struct.pack("<BB", SOF_HOST, count)
    gyro = struct.pack("<3f", *gyro_vec)
    
    payload = bytearray()
    for i in range(count):
        payload += struct.pack("<HH", int(centroids[i, 0]), int(centroids[i, 1]))
    
    full_msg = header + gyro + payload
    crc = crc16_ccitt(full_msg)
    return full_msg + struct.pack("<H", crc)

def unpack_telemetry(data):
    """
    Format: [0xAA][qx, qy, qz, qw][wx, wy, wz][tx, ty, tz][locked][count][CRC16]
    """
    if len(data) != TELEMETRY_PACKET_SIZE:
        return None
        
    # Verify CRC (all bytes except the last 2)
    received_crc = struct.unpack("<H", data[43:45])[0]
    if crc16_ccitt(data[:43]) != received_crc:
        print("[HIL] Telemetry CRC Error!")
        return None
    
    res = struct.unpack("<4f3f3fBB", data[1:43])
    return {
        'q': np.array(res[0:4]),
        'w': np.array(res[4:7]),
        'torque': np.array(res[7:10]),
        'locked': res[10],
        'count': res[11]
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
        except serial.SerialException as e:
            print(f"Serial Error: {e}")
            self.ser = None
            
        self.catalog = np.load(CATALOG_PATH)
        self.q = np.array([0.0, 0.0, 0.0, 1.0]) 
        self.w = np.array([0.01, 0.01, 0.0])    
        self.running = True

    def project_stars(self):
        """ Projects 3D catalog stars into 2D camera pixels """
        rot = R.from_quat(self.q).inv()
        body_vectors = rot.apply(self.catalog[:, 1:4])
        
        # Only stars in front of camera (z > 0)
        mask = body_vectors[:, 2] > 0
        visible_vecs = body_vectors[mask]
        visible_ids = self.catalog[mask, 0]
        visible_mags = self.catalog[mask, 4]
        
        x_pix = (visible_vecs[:, 0] / visible_vecs[:, 2]) * FOCAL_LENGTH + PRINCIPAL_POINT
        y_pix = (visible_vecs[:, 1] / visible_vecs[:, 2]) * FOCAL_LENGTH + PRINCIPAL_POINT
        
        in_fov = (x_pix >= 0) & (x_pix < IMAGE_WIDTH) & (y_pix >= 0) & (y_pix < IMAGE_WIDTH)
        return np.column_stack((visible_ids[in_fov], x_pix[in_fov], y_pix[in_fov], visible_mags[in_fov]))

    def run(self):
        if not self.ser: return

        while self.running:
            # 1. Render
            visible_stars = self.project_stars()
            img = render_star_field(visible_stars, IMAGE_WIDTH)
            
            # 2. Save .mem and clear old results
            if os.path.exists(CENTROIDS_PATH):
                os.remove(CENTROIDS_PATH)
                
            with open(MEM_OUT_PATH, 'w') as f:
                for px in img.flatten():
                    f.write(f"{px:04X}\n")
                f.flush()
                os.fsync(f.fileno())

            # 3. Run FPGA Simulation
            result = subprocess.run("simulate.bat", cwd=XSIM_DIR, shell=True, 
                                    capture_output=True, text=True)
            if result.returncode != 0:
                print(f"[XSIM ERROR]\n{result.stdout}")
                continue
            
            # 4. Parse Centroids
            centroids = np.empty((0, 2))
            if os.path.exists(CENTROIDS_PATH):
                try:
                    centroids = np.loadtxt(CENTROIDS_PATH)
                    if centroids.ndim == 1 and centroids.size > 0:
                        centroids = centroids.reshape(1, -1)
                except (OSError, ValueError) as e:
                    print(f"[HIL] Centroid parse error: {e}")

            # 5. Send to STM32
            packet = build_host_packet(centroids, self.w)
            self.ser.write(packet)

            # 6. Wait for Telemetry
            start_wait = time.time()
            telem_received = False
            
            while (time.time() - start_wait) < 1.0:
                byte = self.ser.read(1)
                # Use byte[0] for modern Python 3 byte comparison
                if byte and byte[0] == SOF_TELEM:
                    raw_telem = byte + self.ser.read(TELEMETRY_PACKET_SIZE - 1)
                    
                    if len(raw_telem) != TELEMETRY_PACKET_SIZE:
                        print("[HIL] Incomplete telemetry packet received.")
                        continue

                    telem = unpack_telemetry(raw_telem)
                    if telem:
                        # 7. Physics Update
                        self.w = telem['w'] 
                        torque = telem['torque']
                        
                        # Euler Integration
                        dw = J_INV @ (torque - np.cross(self.w, J @ self.w))
                        self.w += dw * HIL_DT
                        
                        dq = R.from_rotvec(self.w * HIL_DT)
                        self.q = (R.from_quat(self.q) * dq).as_quat()
                        self.q /= np.linalg.norm(self.q) 
                        
                        telem['image'] = img
                        self.sig_update.emit(telem)
                        telem_received = True
                    break
            
            if not telem_received:
                print("[HIL] Telemetry Timeout - Check STM32 Connection")

    def stop(self):
        self.running = False
        if self.ser:
            self.ser.close()

# ==========================================================
# GUI Implementation
# ==========================================================

class WhereSatGUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("WhereSat HIL Control Center")
        self.resize(1200, 800)
        
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)
        
        self.img_view = pg.ImageView()
        self.img_view.ui.histogram.hide()
        self.img_view.ui.roiBtn.hide()
        self.img_view.ui.menuBtn.hide()
        layout.addWidget(self.img_view, stretch=2)
        
        stats_layout = QtWidgets.QVBoxLayout()
        layout.addLayout(stats_layout, stretch=1)
        
        self.lbl_status = QtWidgets.QLabel("STATUS: BOOTING")
        self.lbl_status.setStyleSheet("font-size: 18px; font-weight: bold; color: yellow;")
        stats_layout.addWidget(self.lbl_status)

        self.telem_text = QtWidgets.QTextEdit()
        self.telem_text.setReadOnly(True)
        self.telem_text.setFont(QtGui.QFont("Consolas", 10))
        stats_layout.addWidget(self.telem_text)
        
        self.sim = HILSimulation()
        self.sim.sig_update.connect(self.update_ui)
        self.sim.start()

    def update_ui(self, data):
        self.img_view.setImage(data['image'].T, autoLevels=True)
        
        status = "LOCKED" if data['locked'] else "SEARCHING"
        color = "#00FF00" if data['locked'] else "#FF0000"
        self.lbl_status.setText(f"STATUS: {status}")
        self.lbl_status.setStyleSheet(f"font-size: 18px; font-weight: bold; color: {color};")
        
        info = (
            f"--- ADCS TELEMETRY ---\n"
            f"Stars Matched: {data['count']}\n\n"
            f"Quaternion [x,y,z,w]:\n"
            f"[{data['q'][0]:.4f}, {data['q'][1]:.4f},\n"
            f" {data['q'][2]:.4f}, {data['q'][3]:.4f}]\n\n"
            f"Angular Vel (rad/s):\n"
            f"X: {data['w'][0]:.5f}\n"
            f"Y: {data['w'][1]:.5f}\n"
            f"Z: {data['w'][2]:.5f}\n\n"
            f"Torque Cmd (Nm):\n"
            f"X: {data['torque'][0]:.4f}\n"
            f"Y: {data['torque'][1]:.4f}\n"
            f"Z: {data['torque'][2]:.4f}"
        )
        self.telem_text.setText(info)

    def closeEvent(self, event):
        self.sim.stop()
        self.sim.wait()
        event.accept()

if __name__ == "__main__":
    app = QtWidgets.QApplication([])
    gui = WhereSatGUI()
    gui.show()
    app.exec_()