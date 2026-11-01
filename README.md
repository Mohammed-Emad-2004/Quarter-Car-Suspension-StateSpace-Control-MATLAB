# 🚗 Quarter-Car Active Suspension — State-Space Control & Observer

> **Mohammed Emad Said Soliman** | UHCI4H  
> Mechatronics Engineering BSc. — University of Debrecen  
> Faculty of Engineering | Department of Electrical Engineering and Mechatronics  
> *Modelling and Simulation Prototype Technologies II — 2025/2026/II. Semester*

---

## 📋 Overview

This project presents the **State-Space modeling, pole-placement controller design, and Luenberger observer** for a **quarter-car active suspension system** subjected to speed bump road disturbances, implemented in **MATLAB and Simulink**.

The goal is to design an active suspension controller that minimizes vertical car body acceleration (passenger comfort) while maintaining road contact (vehicle safety).

---

## 🚙 System Model

### Quarter-Car Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Sprung mass (car body) | mₓ | 408 kg |
| Unsprung mass (wheel+axle) | mₖ | 48.3 kg |
| Suspension spring stiffness | Kᵣ | 30,000 N/m |
| Tire stiffness | Kₖ | 340,000 N/m |
| Damping coefficient | Bᵣ | 1,450 Ns/m |

### State-Space Matrices

**States:** `x = [x_k_dot, x_k, x_c_dot, x_c]`  
**Inputs:** `u = [F_a (actuator force), road disturbance]`  
**Output:** `y = x_c` (car body position)

---

## 🎯 Design Tasks

### 1. Controllability & Observability Analysis
- Computed controllability matrix `Mc` — verified full rank (rank 4)
- Computed observability matrix `Mo` — verified full rank (rank 4)

### 2. Pole Placement Controller
- **Original poles** computed from characteristic equation
- **Desired poles:** `p1=p2=-50`, `p3=p4=-5` (fast settling, no overshoot)
- Gain matrix **K** computed via Ackermann's formula (validated manually)
- **Static error correction** (`Nu`, `Nx`) for zero steady-state error

### 3. Luenberger State Observer
- **Observer poles:** `[-80, -80, -25, -25]` (faster than controller)
- Observer gain **Lc** computed via Ackermann's formula
- Validates that unmeasured states can be estimated from output only

### 4. Speed Bump Disturbance
- Bump parameters: height = 0.2m, width = 0.5m, speed = 5 m/s
- Ramp input modeled in Simulink for realistic road profile

---

## 📁 File Structure

```
├── secondhomework.m          — Main MATLAB script (SS matrices, poles, K, observer)
├── untitled.m                — Supplementary calculations
├── seondhomework.slx         — Simulink model (full closed-loop + observer)
├── UHCI4H Secondhome work.pdf — Full assignment report (PDF)
└── UHCI4H Secondhome work.docx — Full assignment report (Word)
```

---

## 🚀 How to Run

1. Open MATLAB
2. Run `secondhomework.m` to compute all system matrices, gains, and observer
3. Open `seondhomework.slx` in Simulink and run the simulation
4. Observe car body response with/without active control under speed bump input

### Requirements
- MATLAB with Control System Toolbox
- Simulink

---

## 📊 Key Results

| Metric | Uncontrolled | Controlled |
|--------|-------------|-----------|
| Controller poles | System natural | `[-50, -50, -5, -5]` |
| Observer poles | — | `[-80, -80, -25, -25]` |
| Steady-state error | Present | Eliminated (static correction) |
| Speed bump response | Oscillatory | Damped, fast settling |

---

*Mechatronics Engineering BSc. — University of Debrecen, 2026*
