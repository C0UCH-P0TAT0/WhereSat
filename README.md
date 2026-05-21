# Stellar Attitude Determination Pipeline

## 🛰️ Objective
[cite_start]This project implements a complete, closed-loop star tracker simulation pipeline in Python[cite: 89]. [cite_start]Designed as a benchmarking tool to characterize attitude-solving algorithms under realistic hardware conditions, it handles everything from synthetic sensor image generation through to spacecraft attitude output[cite: 90, 91, 92]. 

The current architecture simulates a 3D coordinate universe, projects it onto a 2D sensor array, and strictly models the optical physics and electrical noise of a space-rated analog sensor.

## ⚙️ System Specifications
The simulation engine is hardcoded to the following parameters to accurately replicate edge-hardware constraints:

**Optical & Camera Model**
* **Catalog:** Hipparcos (VizieR)
* **Magnitude Limit:** <= 6.0
* **Camera Resolution:** 1024 x 1024 pixels
* **Field of View (FOV):** 12.0°
* **Point Spread Function (PSF):** 2D Gaussian ($\sigma$ = 1.5 pixels)

**Hardware Sensor Noise Model**
* **Dynamic Range:** 16-bit Unsigned Integer (`np.uint16` / Max ADU: 65535)
* **Quantum Noise:** Poisson photon distribution
* **Readout Noise Floor:** Gaussian ($\sigma$ = 15.0 ADU)
* **Radiation Defect:** Saturated Hot Pixels (0.1% density)

## 📂 Repository Architecture
The project is structured as a modular Python package to separate coordinate mathematics from image processing and hardware simulation.

```text
wheresat/
├── data/                  # Ignored in git. Contains raw CSV, optimized .npy, and output visuals
├── src/
│   └── wheresat/
│       ├── __init__.py
│       ├── catalog.py     # Astropy Hipparcos ingestion and Cartesian (X,Y,Z) vectorization
│       ├── constants.py   # Shared FOV, resolution, and quaternion conventions
│       ├── coordinates.py # Earth-Centered Inertial (ECI) to Body frame quaternion math
│       ├── camera.py      # Pinhole projection (3D to 2D) and spatial filtering
│       ├── renderer.py    # Optical physics engine (Gaussian PSFs + Poisson noise)
│       └── sensor.py      # 16-bit hardware noise injection and Matplotlib validation
├── main.py                # Master flight software entry point
├── requirements.txt       
└── README.md