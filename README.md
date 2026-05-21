<div align="center">

```
██╗  ██╗███████╗██╗     ██╗ ██████╗ ███████╗
██║  ██║██╔════╝██║     ██║██╔═══██╗██╔════╝
███████║█████╗  ██║     ██║██║   ██║███████╗
██╔══██║██╔══╝  ██║     ██║██║   ██║╚════██║
██║  ██║███████╗███████╗██║╚██████╔╝███████║
╚═╝  ╚═╝╚══════╝╚══════╝╚═╝ ╚═════╝ ╚══════╝
         / C H I R O N
```

**6-DOF Collaborative Robot Arm — Kinematic, Control & Dynamic Validation**

[![Python](https://img.shields.io/badge/Python-3.9%2B-blue?style=flat-square&logo=python&logoColor=white)](https://python.org)
[![NumPy](https://img.shields.io/badge/NumPy-1.24%2B-013243?style=flat-square&logo=numpy)](https://numpy.org)
[![SciPy](https://img.shields.io/badge/SciPy-1.10%2B-8CAAE6?style=flat-square&logo=scipy)](https://scipy.org)
[![License: MIT](https://img.shields.io/badge/License-MIT-00d4aa?style=flat-square)](LICENSE)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen?style=flat-square)](CONTRIBUTING.md)
[![Stars](https://img.shields.io/github/stars/ibnshafi/helios?style=flat-square&color=yellow)](https://github.com/ibnshafi/helios/stargazers)

[**Quick Start**](#-quick-start) · [**Demo**](#-interactive-dashboard) · [**Architecture**](#-architecture) · [**Contribute**](#-contributing)

</div>

---

## What is this?

HELIOS/CHIRON is a **research-grade, fully-integrated 6-DOF robot arm simulator** built in pure Python. Every module — from DH-parameter kinematics to recursive Newton-Euler dynamics to a live impedance controller — is wired into a single interactive Matplotlib dashboard you can run in under a minute.

Pull it, run it, break it. Then send a PR.

```
Joint sliders  →  FK  →  Jacobian  →  IK (DLS)  →  Impedance Control  →  Live 3D Plot
                                  ↓
                         Gravity Torques (RNE)  →  Torque Sensor Sim  →  Lyapunov Check
```

---

## ✨ Features

| Module | What it does |
|--------|-------------|
| 🦾 **Forward Kinematics** | Full DH-parameter chain, 4×4 homogeneous transforms, link-by-link positions |
| 🎯 **Damped Least Squares IK** | Singularity-robust iterative solver, converges to 1e-6 m in <200 iterations |
| 📐 **Geometric Jacobian** | 6×6 J with condition number — real-time singularity monitoring |
| ⚖️ **Recursive Newton-Euler** | Static gravity torques with full mass, COM, and inertia tensor per link |
| 🌀 **Impedance Control** | Cartesian spring-damper in operational space + Lyapunov stability proof |
| 🗺️ **DIS-RRT\* Cost Field** | Torque + obstacle + conditioning cost with Boltzmann sampling probability |
| 📡 **Torque Sensor Sim** | Flexure strain (µε) and Wheatstone bridge voltage (mV) per joint |
| ⚙️ **Cycloidal Drive Spec** | 35-lobe drive → 35:1 ratio, ~68 Nm output torque |
| 🖥️ **Interactive Dashboard** | Live 3D arm, 6 joint sliders, IK button, full telemetry readout |

---

## 🚀 Quick Start

**Requirements:** Python 3.9+, pip

```bash
# 1. Clone
git clone https://github.com/ibnshafi/helios.git
cd helios

# 2. Install dependencies (no exotic packages)
pip install numpy scipy matplotlib

# 3. Launch the dashboard
python main.py
```

That's it. A 3D arm pops up with sliders, a live IK solver, and real-time torque/sensor readouts.

---

## 🖥️ Interactive Dashboard

The dashboard gives you full live control:

- **6 joint sliders** — drag any joint, the arm updates instantly
- **IK Solve button** — set a Cartesian target (X/Y/Z + RPY), hit solve
- **Live telemetry panel** — EE position, condition number, gravity torques for all 6 joints, sensor voltage, cost field value, Lyapunov V and V̇

```
┌─────────────────────────────┐  ┌──────────────────────────────┐
│                             │  │  q1  ────●──────────  0.00   │
│       3D Arm View           │  │  q2  ──●────────────  -0.79  │
│                             │  │  q3  ──────────●────   1.57  │
│    [live matplotlib plot]   │  │  q4  ──────●────────   0.00  │
│                             │  │  q5  ─────────●─────   0.79  │
│                             │  │  q6  ──────●────────   0.00  │
│                             │  ├──────────────────────────────┤
│      × Target               │  │  [Target X/Y/Z sliders]      │
│                             │  │  [Roll/Pitch/Yaw sliders]    │
└─────────────────────────────┘  │           [ Solve IK ]       │
                                 └──────────────────────────────┘
```

---

## 🏗️ Architecture

### DH Parameters

The arm is defined by 6 joints using standard Denavit-Hartenberg parameters:

```
Joint │   a (m)  │  α (rad)   │   d (m)  │ θ offset
──────┼──────────┼────────────┼──────────┼──────────
  1   │  0.000   │   0.000    │  0.175   │   0.000
  2   │  0.000   │  -π/2      │  0.000   │  -π/2
  3   │  0.350   │   0.000    │  0.000   │   0.000
  4   │  0.050   │  -π/2      │  0.350   │   0.000
  5   │  0.000   │  +π/2      │  0.000   │   0.000
  6   │  0.000   │  -π/2      │  0.100   │   0.000
```

Total reach: ~0.675 m. Base height: 175 mm. Wrist offset: 100 mm.

### Inverse Kinematics — Damped Least Squares

```
q_{k+1} = q_k + J^T (J J^T + λ²I)^{-1} · e(T_des, T_cur)
```

Where `e` is the 6D pose error (position + axis-angle orientation). The damping factor `λ = 0.1` keeps the solve stable near singularities.

### Impedance Control Law

```
F_ee = K_p · e_p + K_d · ė_p
τ_cmd = J^T_v · F_ee

Lyapunov function:  V  = ½ (ė_p^T Λ ė_p + e_p^T K_p e_p)
Stability proof:    V̇  = -ė_p^T K_d ė_p ≤ 0  ✓
```

### Torque Sensor — Wheatstone Bridge

Each joint has a simulated flexure torque sensor:

```
τ → F = τ / (4r) → M = F·L → ε = M·(h/2) / (E·I) → V_out = V_ex · GF · ε · gain
```

Output is strain in µε and bridge voltage in mV, ready for hardware-in-the-loop validation.

---

## 📦 Module Reference

```python
from main import (
    forward_kinematics,     # (q) → T_ee, positions[7×3]
    geometric_jacobian,     # (q) → J[6×6]
    condition_number,       # (J) → float
    ik_dls,                 # (T_des, q0) → q_solution
    newton_euler_gravity,   # (q) → tau_g[6]
    impedance_control_step, # (q, T_des) → tau_cmd[6], e_p[3]
    impedance_lyapunov,     # (e_p) → V, V_dot
    cost_field,             # (q) → scalar cost
    boltzmann_prob,         # (q) → probability ∈ (0,1]
    torque_sensor_output,   # (tau) → strain_ue, V_out_mV
    cycloidal_drive,        # (lobes=35) → ratio, torque_Nm
)
```

### Example — FK + IK + Gravity Torques

```python
import numpy as np
from main import forward_kinematics, ik_dls, newton_euler_gravity

# Start from home configuration
q = np.array([0.0, -np.pi/4, np.pi/2, 0.0, np.pi/4, 0.0])

# Forward kinematics
T_ee, positions = forward_kinematics(q)
print(f"End-effector position: {T_ee[:3, 3].round(3)} m")
# → End-effector position: [0.451 0.    0.525] m

# Solve IK for a target pose
T_target = np.eye(4)
T_target[:3, 3] = [0.5, 0.0, 0.5]
q_sol = ik_dls(T_target, q, max_iter=200, tol=1e-6, damping=0.1)

# Gravity compensation torques at solution
tau_g = newton_euler_gravity(q_sol)
print(f"Gravity torques: {tau_g.round(2)} Nm")
# → Gravity torques: [-0.    8.74  3.21  0.12  0.16 -0.  ] Nm
```

### Example — Torque Sensor Readout

```python
from main import torque_sensor_output

strain_ue, v_out_mv = torque_sensor_output(tau=8.74)  # Joint 2 under gravity
print(f"Strain: {strain_ue * 1e6:.1f} µε")
print(f"Bridge output: {v_out_mv * 1e3:.2f} mV")
```

---

## 🛣️ Roadmap

### Done ✅
- [x] DH forward kinematics + geometric Jacobian
- [x] Damped least squares IK with pose error (position + orientation)
- [x] Recursive Newton-Euler gravity torques
- [x] Cartesian impedance control with Lyapunov stability analysis
- [x] Torque sensor simulation (strain + Wheatstone bridge)
- [x] DIS-RRT* cost field with Boltzmann sampling
- [x] Full interactive Matplotlib dashboard

### In Progress 🔧
- [ ] **Full DIS-RRT\* planner** — connect cost field to a complete tree-based planner with collision avoidance
- [ ] **Dynamic RNE** — extend to full case with angular velocity, acceleration, Coriolis, centrifugal terms

### Open for PRs 🙋
- [ ] **ROS 2 integration** — wrap as nodes, publish `/joint_states`, subscribe to `/target_pose`
- [ ] **Hardware HIL** — SPI/I2C bridge to read real strain gauges, validate sensor model
- [ ] **PyPI package** — `pip install helios`, type hints, ≥90% test coverage
- [ ] **Benchmark suite** — IK convergence speed, RNE timing, conditioning vs. workspace volume
- [ ] **URDF export** — generate robot description for Gazebo / MoveIt
- [ ] **Web viewer** — Three.js port of the dashboard for browser-based demos

---

## 🤝 Contributing

Contributions are what make this project worth forking. Whether it's a bug fix, a new planning algorithm, or a ROS wrapper — all PRs are welcome.

```bash
# Fork → clone your fork
git clone https://github.com/<you>/helios.git

# Create a feature branch
git checkout -b feat/ros2-integration

# Make your changes, then open a PR against main
```

Please keep new modules consistent with the existing pattern:
- Pure functions where possible (no hidden state)
- NumPy arrays in, NumPy arrays out
- A short docstring explaining inputs/outputs and units

If you're unsure where to start, check the [open issues](https://github.com/ibnshafi/helios/issues) — items tagged `good first issue` are ready to pick up.

---

## 📄 License

MIT — do whatever you want, just keep the attribution.

---

<div align="center">

**If this saved you hours of robotics math, consider leaving a ⭐**

Built with ♥ for the open robotics community

</div>