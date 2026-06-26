import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont, QVector3D
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from scipy.spatial.transform import Rotation as R

from tools.main import SILEngine

def create_solid_box(size_x, size_y, size_z, color, offset=(0, 0, 0)):
    verts = np.array([
        [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
        [-1, -1,  1], [1, -1,  1], [1, 1,  1], [-1, 1,  1]
    ], dtype=float)
    
    verts[:, 0] *= size_x / 2.0
    verts[:, 1] *= size_y / 2.0
    verts[:, 2] *= size_z / 2.0
    verts += np.array(offset)
    
    faces = np.array([
        [0, 1, 2], [0, 2, 3], [4, 5, 6], [4, 6, 7], 
        [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5], 
        [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]  
    ])
    
    colors = np.tile(color, (12, 1))
    md = gl.MeshData(vertexes=verts, faces=faces, faceColors=colors)
    return gl.GLMeshItem(meshdata=md, smooth=False, computeNormals=False)


class MissionControlWindow(QMainWindow):
    def __init__(self, engine):
        super().__init__()
        self.engine = engine
        self.setWindowTitle("WhereSat - ADCS Digital Twin")
        self.resize(1400, 900)
        
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)
        
        self.gl_widget = gl.GLViewWidget()
        self.gl_widget.setCameraPosition(distance=60)
        layout.addWidget(self.gl_widget, stretch=3)
        
        self.build_3d_scene()
        
        bottom_layout = QHBoxLayout()
        layout.addLayout(bottom_layout, stretch=1)
        
        # Telemetry HUD
        self.telemetry_label = QLabel("BOOTING ADCS...")
        self.telemetry_label.setStyleSheet("color: #00FF00; background-color: #000000; padding: 10px; border: 1px solid #333;")
        font = QFont("Courier", 12, QFont.Bold)
        self.telemetry_label.setFont(font)
        self.telemetry_label.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        bottom_layout.addWidget(self.telemetry_label, stretch=1)
        
        # Controls
        control_layout = QVBoxLayout()
        self.btn_play = QPushButton("▶ PLAY (Real-time)")
        self.btn_fast = QPushButton("⏩ FAST FORWARD (Compute Max)")
        self.btn_pause = QPushButton("⏸ PAUSE")
        self.btn_step = QPushButton("⏭ NEXT FRAME (+10ms)")
        
        for btn in [self.btn_play, self.btn_fast, self.btn_pause, self.btn_step]:
            btn.setMinimumHeight(40)
            btn.setStyleSheet("font-weight: bold; font-size: 14px;")
            control_layout.addWidget(btn)
            
        bottom_layout.addLayout(control_layout, stretch=1)
        
        # Star Tracker Camera Feed
        self.img_view = pg.ImageView()
        self.img_view.ui.histogram.hide()
        self.img_view.ui.roiBtn.hide()
        self.img_view.ui.menuBtn.hide()
        bottom_layout.addWidget(self.img_view, stretch=2)

        # Scatter overlay: red circles on the up-to-6 extracted centroids only
        self.hud_scatter = pg.ScatterPlotItem(
            size=25, pen=pg.mkPen('r', width=2), brush=pg.mkBrush(None)
        )
        self.img_view.getView().addItem(self.hud_scatter)
        self.hud_text_items = []  # HIP label per centroid, cleared every camera frame

        self.timer = QTimer()
        self.timer.timeout.connect(self.run_step)
        
        self.btn_play.clicked.connect(lambda: self.timer.start(10)) 
        self.btn_fast.clicked.connect(lambda: self.timer.start(0))  
        self.btn_pause.clicked.connect(self.timer.stop)
        self.btn_step.clicked.connect(self.run_step)
        
        self.run_step()

    def build_3d_scene(self):
        # 1. The Skybox
        sky_radius = 150
        star_coords = self.engine.catalog[:, 1:4] * sky_radius
        
        mags = self.engine.catalog[:, 4]
        alphas = np.clip(1.0 - ((mags - 1.0) / 5.0), 0.1, 1.0)
        colors = np.zeros((len(star_coords), 4))
        colors[:, 0:3] = 1.0 
        colors[:, 3] = alphas
        
        star_scatter = gl.GLScatterPlotItem(pos=star_coords, color=colors, size=4)
        self.gl_widget.addItem(star_scatter)
        
        # 2. The Earth (Restored)
        md = gl.MeshData.sphere(rows=30, cols=30, radius=20)
        self.earth = gl.GLMeshItem(meshdata=md, smooth=True, color=(0.1, 0.4, 0.9, 0.2), glOptions='translucent')
        self.earth.translate(0, 0, -35)
        self.gl_widget.addItem(self.earth)
        
        # 3. The Solid Satellite
        self.bus = create_solid_box(4, 4, 8, [1.0, 0.8, 0.0, 1.0])
        self.panel1 = create_solid_box(12, 4, 0.2, [0.1, 0.2, 0.5, 1.0], offset=(8, 0, 0))
        self.panel2 = create_solid_box(12, 4, 0.2, [0.1, 0.2, 0.5, 1.0], offset=(-8, 0, 0))
        self.axes = gl.GLAxisItem(size=QVector3D(15, 15, 15))
        
        self.sat_parts = [self.bus, self.panel1, self.panel2, self.axes]
        for part in self.sat_parts:
            self.gl_widget.addItem(part)

    def run_step(self):
        telemetry = self.engine.step_forward()
        
        status = "LOCKED 🟢" if telemetry['adcs_locked'] else "TUMBLING 🔴"
        
        hud = (
            f"=== WHERESAT ADCS TELEMETRY ===\n"
            f"MISSION TIME : {telemetry['time']:06.2f} s\n\n"
            f"SYS STATUS   : {status}\n"
            f"POINT ERROR  : {telemetry['error_deg']:05.2f} °\n"
            f"SPIN RATE    : {telemetry['spin_rate']:05.2f} deg/s\n\n"
            f"OPTICAL SENSOR\n"
            f"STARS EXTRACTED : {telemetry['stars_extracted']:02d}\n"
            f"BIAS EST : {telemetry['gyro_bias_est']}\n"
        )
        self.telemetry_label.setText(hud)
        
        # --- CAMERA FEED (original behaviour) ---
        img = telemetry['image'].T
        self.img_view.setImage(img, autoLevels=True)
            
        # --- HUD OVERLAY: circles + star names on extracted centroids only ---
        # Clear previous frame's text labels
        for text_item in self.hud_text_items:
            self.img_view.getView().removeItem(text_item)
        self.hud_text_items.clear()

        centroids = telemetry.get('extracted_centroids', np.array([]))
        names = telemetry.get('extracted_names', [])

        if len(centroids) > 0:
            # centroids: [x, y, hip_id]
            self.hud_scatter.setData(centroids[:, 0], centroids[:, 1])
            for (x, y, _), name in zip(centroids, names):
                label = pg.TextItem(name, color=(255, 50, 50), anchor=(-0.2, 1.2))
                label.setPos(x, y)
                label.setFont(QFont("Arial", 10, QFont.Bold))
                self.img_view.getView().addItem(label)
                self.hud_text_items.append(label)
        else:
            self.hud_scatter.setData([], [])
        
        # --- 3D KINEMATICS ---
        rot_matrix = R.from_quat(telemetry['true_q']).as_matrix()
        transform_4x4 = np.eye(4)
        transform_4x4[:3, :3] = rot_matrix
        
        for part in self.sat_parts:
            part.setTransform(transform_4x4)


if __name__ == "__main__":
    engine = SILEngine()
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = MissionControlWindow(engine)
    window.show()
    sys.exit(app.exec_())