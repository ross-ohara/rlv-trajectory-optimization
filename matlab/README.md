# MATLAB Implementation – RLV Flip-and-Land Trajectory Optimization

This folder contains the **MATLAB reference implementation** of the reusable launch vehicle (RLV) flip-and-land trajectory optimization problem.

The MATLAB version serves as the **baseline prototype** for:

- Modeling the planar rigid-body dynamics of a vertically landing rocket
- Formulating and solving a nonlinear optimal control problem using `fmincon`
- Simulating the resulting trajectory
- Visualizing state and control histories
- Generating an animation of the landing maneuver

It is designed to be:

- **Readable** – modular functions with clear responsibilities  
- **Reproducible** – no external `.mat` initial guesses required  
- **Extendable** – a clean base for future enhancements and cross-language ports  

---

## 🔧 Requirements

To run this MATLAB implementation, you’ll need:

- **MATLAB** (R2021a or later recommended)
- **Optimization Toolbox** (for `fmincon`)
- **Parallel Computing Toolbox** *(optional)*  
  - used if `UseParallel` is set to `true` in the `fmincon` options

---

## ▶️ Quick Start

From the repository root:

1. Add the `matlab/` folder to your MATLAB path, or `cd` into it:
   ```matlab
   cd path/to/rlv-trajectory-optimization/matlab
   ```

2. Run the main script:
   ```matlab
   run_rlv_optimization
   ```

This will:

- Define model and optimization parameters
- Build a simple analytic initial guess (no `.mat` file required)
- Run `fmincon` to solve the optimal control problem
- Simulate the optimal trajectory with `ode45`
- Plot the states and controls

To generate an animation:

- Either uncomment the animation call at the end of `run_rlv_optimization.m`, or
- Call it manually after running the optimization:

```matlab
params = rlv_params();
y0     = [-200; 1000; pi/2; 25; -90; 0];  % initial state
% U_opt should be in workspace from run_rlv_optimization
rlv_make_animation(U_opt, params, y0, 'rlv_landing.mp4', 30);
```

---

## 📁 File Overview

### `run_rlv_optimization.m`
**Entry point script** for the MATLAB implementation.

Responsibilities:
- Clears the workspace and sets up the run
- Constructs the parameter struct via `rlv_params`
- Defines the initial state
- Builds a deterministic initial guess for the controls
- Configures `fmincon` options
- Solves the optimal control problem
- Calls:
  - `rlv_simulate_and_plot` to visualize the solution
  - `rlv_make_animation` (optional) to generate a landing animation

---

### `rlv_params.m`
Defines a **parameter struct** `params` used throughout the implementation:

- Physical parameters:
  - `params.m` – vehicle mass (kg)
  - `params.g` – gravity (m/s²)
  - `params.L` – vehicle length (m)
  - `params.r` – thrust moment arm (m)
  - `params.I` – moment of inertia (kg·m²)

- Control discretization:
  - `params.res` – number of control nodes

- Actuator limits:
  - `params.F_max` – maximum thrust (N)
  - `params.gimbal_max` – max gimbal angle (rad)

---

### `rlv_dynamics.m`
Implements the **nonlinear planar dynamics** of the RLV:

- State vector:
  ```
  [x, z, theta, vx, vz, omega]
  ```
- Normalized controls in `Uvect`:
  - thrust nodes in [0,1]
  - gimbal nodes in [0,1]
  - final time tf

Uses PCHIP interpolation and computes:

- Thrust force and direction
- Horizontal & vertical forces
- Moment about the center of mass
- Resulting accelerations

Returns:
```
[ vx, vz, omega, ax, az, alpha ]
```

---

### `rlv_cost.m`
Defines the **objective function** minimized by `fmincon`.

Cost:
\[
J = \int (T_{	ext{norm}}^2 + \delta_{	ext{norm}}^2) \, dt
\]

- Simulates trajectory with `ode45`
- Interpolates normalized controls over time
- Computes smoothness-encouraging squared-integral cost

---

### `rlv_constraints.m`
Implements **nonlinear equality and inequality constraints**.

Equality (`ceq`):
- Final position ≈ 0
- Final velocity ≈ 0
- Final pitch ≈ 0
- Final angular rate ≈ 0
- Final accelerations ≈ 0

Inequality (`c <= 0`):
- Max pitch angle ≤ 90°
- z(t) ≥ 0 (no underground)

---

### `rlv_simulate_and_plot.m`
Simulates the optimal trajectory and produces a **6-panel** visualization:

1. Position vs time  
2. Velocity vs time  
3. Pitch angle vs time  
4. Flight path  
5. Thrust magnitude  
6. Gimbal angle  

---

### `rlv_make_animation.m`
Generates an **MP4 animation** of the flip-and-land maneuver:

- Correct body geometry and rotation convention
- Engine gimbal and thrust vector visualization
- Flight path trace
- Landing pad marker
- Adjustable FPS and filename

Example:
```matlab
rlv_make_animation(U_opt, params, y0, 'rlv_landing.mp4', 30);
```

---

## 🧭 Role in the Overall Project

The MATLAB folder provides the **reference implementation** for:

- Vehicle dynamics  
- Control parameterization  
- Optimization formulation  
- Simulation and visualization  

The upcoming Python/CasADi and C++ versions will mirror this methodology for cross-language consistency.

