# Archived Preliminary Implementation  
### Reusable Launch Vehicle (RLV) Flip-and-Land Trajectory Optimization  
**Author:** Ross O'Hara  

This folder contains the **original exploratory implementation** of my reusable launch vehicle (RLV) flip-and-land trajectory optimization project.  
These files represent my earliest working prototype before refactoring the MATLAB code and before expanding the project into **Python (CasADi/IPOPT)** and **C++ (RK4 simulation)**.

Preserving this version highlights the **evolution of the project**, documenting how the initial direct-shooting MATLAB script grew into a complete multi-language trajectory optimization toolchain.

---

## 📁 **Contents**

### **`matlab_original/`**
This folder holds the raw, unrefined MATLAB version exactly as it existed during my first successful tests.

- **`main.m`**  
  The full initial prototype implementing:
  - 6-state nonlinear rigid-body dynamics  
  - PCHIP-interpolated thrust and gimbal controls  
  - A direct-shooting optimal control formulation using `fmincon`  
  - Landing constraints and trajectory visualization  
  - GIF/MP4 animation generation  

- **`res15_initGuess.mat`**  
  Initial guess vector used during early solver experiments. Necessary for reproducing the original optimization behavior.

- **`StatesAndControls.png`**  
  Early visualization of the optimized state trajectories and control profiles produced by this preliminary version.

- **`OptimalControlTVDL.mp4`**  
  Original animation of the flip-and-land maneuver generated directly from the MATLAB prototype.

---

### **`reports/`**
Contains the early technical summary of the project.

- **`Minimum-Energy_Landing_Trajectory_Optimization_for_a_Reusable_Launch_Vehicle.pdf`**  
  A preliminary write-up describing the problem setup, modeling assumptions, optimization strategy, and early results.

---

## 🧭 **Purpose of This Archive**

This archive is intentionally included to demonstrate:

### **1. The iterative nature of engineering work**  
The code in `matlab_original/` is where the project began — a single, monolithic MATLAB script used for exploration, debugging, and rapid prototyping.

### **2. How the project evolved**  
Since this version, the work has expanded into:
- A **clean, modular MATLAB implementation**  
- A **Python trajectory optimizer** using CasADi + IPOPT  
- A **C++ simulation engine** using RK4 (real-time compatible)  

Each refinement improved:
- numerical stability  
- solver performance  
- code clarity  
- extensibility  
- multi-language fluency  

### **3. Transparency and growth**  
Rather than deleting or hiding early work, this archive preserves the engineering lineage of the project.  
My goal is to show not only the final polished implementation, but also the **process** that led there.

This mirrors real aerospace software workflows, where prototypes evolve into production-ready systems.

---

## 🔗 **Where to Find the Refactored Version**

The fully reorganized, modernized MATLAB, Python, and C++ implementations are located in the root project:

```
/matlab/
/python/
/cpp/
```

These contain:
- Clean module separation  
- Updated solver pipelines  
- Improved control parameterization  
- Higher-quality visualizations  
- Cross-language consistency  

---

If you have questions about this archive or would like to discuss the methodology, feel free to reach out.  
This project is part of my ongoing work in **trajectory optimization, flight dynamics, and autonomous systems engineering**.
