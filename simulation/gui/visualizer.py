import sys, os, time, serial, math
import numpy as np
from pathlib import Path
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont, QVector3D
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from scipy.spatial.transform import Rotation as R

# ---> SET YOUR STM32 COM PORT HERE <---
STM32_PORT = 'COM12' 
BAUD_RATE = 115200

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPT_DIR.parent.parent
DATA_DIR = ROOT_DIR / "data"

def create_solid_box(size_x, size_y, size_z, color, offset=(0, 0, 0)):
    verts = np.array([[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
                      [-1, -1,  1], [1, -1,  1], [1, 1,  1], [-1, 1,  1]], dtype=float)
    verts[:, 0] *= size_x / 2.0; verts[:, 1] *= size_y / 2.0; verts[:, 2] *= size_z / 2.0
    verts += np.array(offset)
    faces = np.array([[0, 1, 2], [0, 2, 3], [4, 5, 6], [4, 6, 7], 
                      [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5], 
                      [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]])
    colors = np.tile(color, (12, 1))
    md = gl.MeshData(vertexes=verts, faces=faces, faceColors=colors)
    return gl.GLMeshItem(meshdata=md, smooth=False, computeNormals=False)

class MissionControlWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("WhereSat - MCU Hardware-in-the-Loop (HITL)")
        self.resize(1400, 900)
        
        try:
            self.stm32 = serial.Serial(STM32_PORT, BAUD_RATE, timeout=1)
            print(f"Connected to STM32 on {STM32_PORT}")
        except Exception as e:
            print(f"ERROR: Could not connect to STM32 on {STM32_PORT}. Close STM32CubeIDE Serial Monitor!")
            sys.exit(1)

        self.frame_count = 0
        
        # --- PHYSICS ENGINE SETUP ---
        self.dt = 1.0
        self.inertia = np.diag([0.1, 0.1, 0.1])
        
        # 1. Target Orion Constellation
        self.target_q = R.from_euler('xyz', [0, 82, 0], degrees=True).as_quat()
        
        # 2. Start the satellite 5 degrees off-target
        offset = R.from_euler('xyz', [5, -4, 2], degrees=True)
        self.true_q = (R.from_quat(self.target_q) * offset).as_quat()
        
        # 3. Add a tiny bit of tumbling spin
        self.true_omega = np.array([0.01, -0.01, 0.005]) 
        
        # Load Catalog
        catalog = np.load(DATA_DIR / "optimized_catalog.npy")
        if catalog.dtype.names is not None:
            self.vecs = np.column_stack((catalog['x'], catalog['y'], catalog['z']))
            self.mags = catalog['mag'] if 'mag' in catalog.dtype.names else catalog['Vmag']
        else:
            self.vecs = catalog[:, 1:4]
            self.mags = catalog[:, 4]
        self.mini_vecs = self.vecs[self.mags <= 2.5]

        # --- GUI SETUP ---
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)
        
        self.gl_widget = gl.GLViewWidget()
        self.gl_widget.setCameraPosition(distance=60)
        layout.addWidget(self.gl_widget, stretch=3)
        self.build_3d_scene()
        
        bottom_layout = QHBoxLayout()
        layout.addLayout(bottom_layout, stretch=1)
        
        self.telemetry_label = QLabel("BOOTING PHYSICS ENGINE...")
        self.telemetry_label.setStyleSheet("color: #00FF00; background-color: #000000; padding: 10px; border: 1px solid #333;")
        self.telemetry_label.setFont(QFont("Courier", 12, QFont.Bold))
        self.telemetry_label.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        bottom_layout.addWidget(self.telemetry_label, stretch=1)
        
        control_layout = QVBoxLayout()
        self.btn_play = QPushButton("▶ AUTO-PLAY (Fly Satellite)")
        self.btn_pause = QPushButton("⏸ PAUSE")
        self.btn_step = QPushButton("⏭ STEP (1 Frame)")
        for btn in [self.btn_play, self.btn_pause, self.btn_step]:
            btn.setMinimumHeight(40)
            btn.setStyleSheet("font-weight: bold; font-size: 14px; background-color: #2b2b2b; color: white;")
            control_layout.addWidget(btn)
        bottom_layout.addLayout(control_layout, stretch=1)
        
        self.img_view = pg.ImageView()
        self.img_view.ui.histogram.hide(); self.img_view.ui.roiBtn.hide(); self.img_view.ui.menuBtn.hide()
        bottom_layout.addWidget(self.img_view, stretch=2)

        self.hud_scatter = pg.ScatterPlotItem(size=25, pen=pg.mkPen('r', width=2), brush=pg.mkBrush(None))
        self.img_view.getView().addItem(self.hud_scatter)

        self.timer = QTimer()
        self.timer.timeout.connect(self.run_hardware_step)
        self.btn_play.clicked.connect(lambda: self.timer.start(1000)) 
        self.btn_pause.clicked.connect(self.timer.stop)
        self.btn_step.clicked.connect(self.run_hardware_step)

    def build_3d_scene(self):
        md = gl.MeshData.sphere(rows=30, cols=30, radius=20)
        self.earth = gl.GLMeshItem(meshdata=md, smooth=True, color=(0.1, 0.4, 0.9, 0.2), glOptions='translucent')
        self.earth.translate(0, 0, -35)
        self.gl_widget.addItem(self.earth)
        
        self.bus = create_solid_box(4, 4, 8, [1.0, 0.8, 0.0, 1.0])
        self.panel1 = create_solid_box(12, 4, 0.2, [0.1, 0.2, 0.5, 1.0], offset=(8, 0, 0))
        self.panel2 = create_solid_box(12, 4, 0.2, [0.1, 0.2, 0.5, 1.0], offset=(-8, 0, 0))
        self.axes = gl.GLAxisItem(size=QVector3D(15, 15, 15))
        self.sat_parts = [self.bus, self.panel1, self.panel2, self.axes]
        for part in self.sat_parts:
            self.gl_widget.addItem(part)

    def run_hardware_step(self):
        self.frame_count += 1
        
        # 1. Generate Camera Image based on CURRENT Physics Rotation
        width = 1024
        focal_length = (width / 2) / math.tan(math.radians(20.0 / 2))
        
        r_body = R.from_quat(self.true_q) 
        body_stars = r_body.apply(self.mini_vecs)
        visible_mask = body_stars[:, 2] > 0
        visible_body = body_stars[visible_mask]
        
        pixels_x = (visible_body[:, 0] * focal_length / visible_body[:, 2]) + (width / 2)
        pixels_y = (visible_body[:, 1] * focal_length / visible_body[:, 2]) + (width / 2)
        
        in_frame = (pixels_x >= 0) & (pixels_x < width) & (pixels_y >= 0) & (pixels_y < width)
        
        final_x = pixels_x[in_frame][:5]
        final_y = pixels_y[in_frame][:5]

        # Draw the stars on the 2D camera feed
        img = np.zeros((1024, 1024), dtype=np.uint16)
        for i in range(len(final_x)):
            x, y = int(final_x[i]), int(final_y[i])
            for dx in range(-3, 4):
                for dy in range(-3, 4):
                    if 0 <= x+dx < 1024 and 0 <= y+dy < 1024:
                        img[x+dx, y+dy] = 65535 
        self.img_view.setImage(img.T, autoLevels=False, levels=(0, 65535))
        self.hud_scatter.setData(final_x, final_y)

        telemetry_text = f"=== WHERESAT HITL TELEMETRY ===\nFRAME: {self.frame_count}\n\n"
        torque = np.array([0.0, 0.0, 0.0])

        # 2. Send the Live Centroids to the STM32 over USB
        if len(final_x) >= 3:
            num_stars = len(final_x)
            
            # Send Target
            tx, ty, tz, tw = int(self.target_q[0]*1e6), int(self.target_q[1]*1e6), int(self.target_q[2]*1e6), int(self.target_q[3]*1e6)
            self.stm32.write(f"TARGET,{tx},{ty},{tz},{tw}\n".encode())
            
            # Send Pixels
            self.stm32.write(f"START,{num_stars}\n".encode())
            for i in range(num_stars):
                self.stm32.write(f"{final_x[i]},{final_y[i]}\n".encode())

            # 3. Read the STM32's Answer
            lines_read = 0
            while lines_read < 30: 
                QApplication.processEvents() # Keep GUI alive
                line = self.stm32.readline().decode('utf-8', errors='ignore').strip()
                if not line: break
                
                if "[ADCS]" in line or "[MEKF]" in line or "[CTRL]" in line or "Q:" in line:
                    telemetry_text += line + "\n"
                
                # Catch the Torque from the STM32
                if "[CTRL] Torque [Nm]:" in line:
                    try:
                        t_str = line.split("]:")[1].replace("[", "").replace("]", "")
                        # FLIP THE MOTORS SO IT FLIES TO THE TARGET!
                        torque = -1.0 * np.array([float(x) for x in t_str.split(",")])
                    except:
                        pass
                        
                if "==================================================" in line and "[ADCS]" in telemetry_text:
                    break
                lines_read += 1
        else:
            telemetry_text += "CAMERA BLIND: Fast-Forwarding to find stars...\n"
            # FAST FORWARD PHYSICS: Spin 10x faster to find stars!
            self.true_omega = np.array([0.05, -0.05, 0.02]) 

        # ==========================================================
        # 4. THE PHYSICS ENGINE
        # ==========================================================
        inv_inertia = np.linalg.inv(self.inertia)
        gyro_torque = np.cross(self.true_omega, self.inertia @ self.true_omega)
        angular_accel = inv_inertia @ (torque - gyro_torque)
        self.true_omega += angular_accel * self.dt
        
        dq = R.from_rotvec(self.true_omega * self.dt).as_quat()
        self.true_q = (R.from_quat(self.true_q) * R.from_quat(dq)).as_quat()

        # Calculate Error to Target
        dot_product = np.clip(np.abs(np.dot(self.true_q, self.target_q)), 0, 1)
        error_deg = np.degrees(2 * np.arccos(dot_product))
        telemetry_text += f"\nPHYSICS ERROR TO TARGET: {error_deg:.2f} degrees\n"

        self.telemetry_label.setText(telemetry_text)

        # 5. Rotate the 3D Satellite
        rot_matrix = R.from_quat(self.true_q).as_matrix()
        transform_4x4 = np.eye(4)
        transform_4x4[:3, :3] = rot_matrix
        for part in self.sat_parts:
            part.setTransform(transform_4x4)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = MissionControlWindow()
    window.show()
    sys.exit(app.exec_())