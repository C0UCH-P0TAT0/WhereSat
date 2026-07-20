# 🚀 WhereSat – FPGA Accelerated Star Tracker & ADCS

## 🎯 Goal
WhereSat is an end-to-end spacecraft **Attitude Determination and Control System (ADCS)** featuring a custom FPGA-accelerated Star Tracker. The project combines computer vision, embedded systems, estimation theory, and spacecraft control to estimate spacecraft orientation and generate reaction wheel control commands. :contentReference[oaicite:0]{index=0}

---

## ✨ Features

- ⭐ Synthetic star field generation
- 📷 Realistic sensor noise simulation
- ⚡ FPGA-based centroid extraction
- 🌌 Lost-in-Space star identification
- 🛰️ QUEST attitude determination
- 📐 Multiplicative Extended Kalman Filter (MEKF)
- 🎯 PD attitude controller
- 🔧 Hardware verification between Python and FPGA RTL

---

## 🏗 System Architecture

```
Synthetic Sky
      │
      ▼
Camera & Sensor Model
      │
      ▼
FPGA Vision Pipeline
(Threshold → Blob Detection → Centroids)
      │
      ▼
MCU Star Identification
(Triangle Matching)
      │
      ▼
QUEST
      │
      ▼
MEKF
      │
      ▼
PD Controller
      │
      ▼
Reaction Wheel Torque
```

---

## ⚙ Technologies

- Python
- NumPy
- SciPy
- Verilog (FPGA)
- Embedded C
- STM32
- Vivado
- Kalman Filtering
- Quaternion Mathematics
- Spacecraft Attitude Dynamics

---

## 📂 Project Modules

- **Simulation** – Synthetic star generation and sensor modelling
- **FPGA** – Real-time image processing and centroid extraction
- **MCU** – Star identification, QUEST, MEKF, and control
- **Verification** – RTL vs Python validation tools

---

## 👥 Team 
Developed by:

- **Yash Dobariya**
- **Aditya Malpani**

**WhereSat Project**
```text
wheresat/
├── data/
├── fpga
│   ├── rtl
│   ├── tb
├── mcu
│   ├── Core
|       ├── Inc
|       ├── Src
│   ├── Drivers
├── python/
│   ├── apps
│   ├── benchmarks
│   ├── core
│   ├── tests
│   ├── tools