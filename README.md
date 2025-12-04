# 🛰️ Reusable Launch Vehicle (RLV) Flip-and-Land Trajectory Optimization  
### *Cross-Language Implementation in MATLAB, Python (CasADi/IPOPT), and C++ (RK4 Simulation)*  
**Author:** Ross O’Hara

---

## ⭐ Overview  
This project implements a **minimum-energy flip-and-land trajectory** for a reusable launch vehicle (RLV) using a planar rigid-body model with thrust vector control.  

The same modeling and optimization framework is built across **three languages**:

- **MATLAB** → rapid prototyping, ODE modeling, direct shooting  
- **Python (CasADi + IPOPT)** → modern nonlinear optimization and auto-differentiation  
- **C++ (RK4)** → real-time compatible simulation and future on-board autonomy use  

This progression reflects the workflow used in modern GNC organizations:  
prototype → optimize → deploy.

The project demonstrates practical skills in **flight dynamics**, **nonlinear optimal control**, **simulation**, and **software engineering**—all core to landing rockets, autonomous spacecraft, and high-performance robotic systems.

---

## 🚀 Project Motivation  
Reusable launch vehicles rely on precise powered-descent trajectories that balance **attitude**, **thrust**, and **timing** to land safely and efficiently. This project explores a simplified version of that problem:

- How does a rocket flip from horizontal to vertical?  
- How should thrust and gimbal angle be modulated?  
- How do constraints shape the optimal trajectory?  
- How can this be solved using modern numerical optimization tools?

Initially developed in MATLAB, I expanded the project into Python and C++ to better reflect tools used across the industry and to strengthen my fluency across the languages common to GNC teams.

Over time, this repository will grow into a complete, multi-language RLV landing toolbox.

---

## 📐 Technical Summary  
The vehicle is represented as a planar rigid body with:

- **6 states**: x, z, θ, vx, vz, ω  
- **2 controls**: thrust magnitude and gimbal angle  
- **Assumptions**:  
  - constant mass  
  - rigid body  
  - no drag  
  - single gimbaled engine  
  - gravity only  

### **Optimization Objective**  
Minimize:

∫ ( T(t)² + δ(t)² ) dt

subject to:

- nonlinear dynamics  
- actuator limits  
- touchdown constraints (zero velocity, upright attitude)  
- bounded flight envelope  

The result is a 10-second powered descent featuring:

- a **90° flip maneuver**  
- thrust-vector-controlled attitude stabilization  
- smooth deceleration  
- soft landing  

Early plots, animations, and the legacy MATLAB implementation are preserved in the `archive/` folder.

---

## 🗂️ Repository Structure

```
rlv-trajectory-optimization/
  archive/                # preserved early prototype (MATLAB exploration)
  matlab/                 # refactored MATLAB implementation (coming next)
  python/                 # CasADi / IPOPT version under development
  cpp/                    # C++ RK4 simulation under development
  README.md               # this file
```

---

## 🧰 Core Technologies (across languages)

### MATLAB
- ode45  
- nonlinear dynamics modeling  
- PCHIP control interpolation  
- direct shooting with fmincon  
- trajectory plotting and animation  

### Python (planned)
- CasADi symbolic modeling  
- IPOPT nonlinear programming  
- direct collocation  
- Matplotlib visualization  

### C++ (planned)
- RK4 integrator  
- modular dynamics library  
- real-time compatible simulation  
- potential SNOPT/ACADO/CppAD integration  

---

## 🎞️ Preliminary Results  
Early MATLAB results—including the flip maneuver animation, state histories, and control profiles—are located in:

```
archive/matlab_original/
```

This folder preserves the initial exploratory implementation exactly as it was before refactoring.

---

## 🛠️ Planned Extensions  

- variable-mass dynamics  
- atmospheric drag modeling  
- uncertainty propagation  
- robust optimal control  
- full 6-DOF extension  
- onboard-compatible C++ trajectory rollout  
- optional MPC layer  

---

## 📬 Contact  
If you work in **GNC**, **trajectory optimization**, **autonomy**, or **robotics** and want to collaborate or discuss this work, feel free to reach out.

**Email:** rossohara.rlo@gmail.com  
**LinkedIn:** (link to be added)

---
