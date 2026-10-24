# 3D Smooth Trajectory Generation for UAV Window/Gap Traversal

A MATLAB & Simulink implementation extending the 2D bearing-based gap traversal framework from **"Smooth Trajectory Generation for Gap Traversal by UAVs"** (Singha, Pal, and Hota, *2025 IEEE Space, Aerospace and Defence Conference (SPACE)*) into three-dimensional (3D) space.

---

## Overview

Unmanned Aerial Vehicles (UAVs) operating in cluttered or indoor environments—such as search and rescue missions inside damaged structures—must safely navigate through narrow windows and gaps without colliding with wall edges. Traditional bearing-based traversal methods often generate $C^0$ paths that ignore the vehicle's initial orientation, forcing the UAV to stop and reorient at intermediate waypoints.

This project addresses the immediate future work proposed in the original 2D study by extending the two-stage continuous trajectory generation pipeline into **3D space**:

1. **Initial Pose & Safety Offset Integration:** Accounts for the UAV's initial 3D position and orientation while introducing a safety offset distance ($\delta_{\text{offset}}$) before and after each window midpoint $(x_w, y_w, z_w)$ to prevent circular turn collisions with wall boundaries.
2. **$C^1$ Optimal Reference Path (3D Dubins Path):** Generates a time- and length-optimal $C^1$-continuous reference trajectory connecting sequential gap configurations using CSC (`RSR`, `LSL`, `RSL`, `LSR`) Dubins path geometry.
3. **$C^2$ Smooth Trajectory Tracking (3D Carrot-Chasing Guidance):** Uses the $C^1$ Dubins path as a reference and applies a Virtual Target Point (VTP) Carrot-Chasing guidance law to achieve a near-optimal, $C^2$-continuous trajectory that strictly satisfies prescribed curvature bounds $(-\kappa_{\max}, \kappa_{\max})$ and lateral acceleration limits.

---

## Repository Structure

```text
Matlab-Window-Traversal-3D/
├── init_3dsimulation.m       # Initializes 3D environment, UAV parameters, window coordinates, and Dubins geometry
├── dubins_path3d_model.slx   # Simulink model for 3D Carrot-Chasing guidance and UAV kinematic simulation
├── export_plot3d.m           # Post-processing script to visualize and export 3D trajectories and curvature profiles
└── README.md                 # Project documentation
```

---

## Methodology

### 1. Window Midpoint & Safety Offset Calculation
Given detected 3D window/gap boundary coordinates, the safe passage midpoint $(x_w, y_w, z_w)$ is computed. To ensure the vehicle aligns perpendicular to the gap prior to entry and exits cleanly before initiating the next turn, an offset $\delta_{\text{offset}}$ is applied along the traversal axis to establish the terminal Dubins configuration $(x_f, y_f, z_f, \chi_f)$.

### 2. $C^1$ Reference Path Generation
For each gap segment, the four minimum-turn-radius circle centers ($CL_i, CR_i, CL_f, CR_f$) are evaluated based on the minimum turning radius $r$:
* **Right Turn ($R$):** $a = x + r\sin\chi, \quad b = y - r\cos\chi$
* **Left Turn ($L$):** $a = x - r\sin\chi, \quad b = y + r\cos\chi$

Comparing center-to-center Euclidean distances determines the optimal CSC path type (`RSR`, `LSL`, `RSL`, or `LSR`) and tangent transition points $A$ and $B$, coupled with vertical altitude transitions across the 3D waypoints.

### 3. $C^2$ Continuity via Carrot-Chasing
To eliminate the curvature discontinuities inherent at Dubins tangent junctions, the Carrot-Chasing algorithm projects a Virtual Target Point (VTP) at a look-ahead distance $\delta$ along the reference segments. The UAV adjusts its heading and pitch rates proportionally ($k$) to chase the VTP, yielding a smooth $C^2$ trajectory.

---

## Getting Started

### Prerequisites
* **MATLAB** (R2022b or newer recommended)
* **Simulink**

### Running the Simulation

1. **Clone the repository:**
   ```bash
   git clone git@github.com:Legendparth/Matlab-Window-Traversal-3D.git
   cd Matlab-Window-Traversal-3D
   ```

2. **Initialize simulation parameters:**
   Open MATLAB, navigate to the repository folder, and run the initialization script to load the 3D window edge coordinates, initial UAV pose, turning radius $r$, offset $\delta_{\text{offset}}$, and controller gains into the workspace:
   ```matlab
   run('init_3dsimulation.m')
   ```

3. **Execute the Simulink model:**
   Open and run the Simulink model directly from the command window or Simulink interface:
   ```matlab
   open_system('dubins_path3d_model.slx')
   sim('dubins_path3d_model.slx')
   ```

4. **Generate and export 3D plots:**
   Run the plotting script to render the 3D gap traversal comparison ($C^1$ vs. $C^2$ trajectories) and curvature profiles:
   ```matlab
   run('export_plot3d.m')
   ```

---

## Default Simulation Parameters

The baseline parameters adapted from the reference paper include:

| Parameter | Symbol | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **Airspeed** | $V_a$ | $5\text{ m/s}$ | Constant forward velocity of the UAV |
| **Minimum Turn Radius** | $r$ | $25\text{ m}$ | Minimum curvature radius ($\kappa_{\max} = 0.04\text{ m}^{-1}$) |
| **Safety Offset** | $\delta_{\text{offset}}$ | $30\text{ m}$ | Pre- and post-gap straight-line alignment buffer |
| **Look-Ahead Distance** | $\delta$ | $4.5\text{ m}$ | Virtual Target Point (VTP) distance for Carrot-Chasing |
| **Heading Gain** | $k$ | $0.1$ | Proportional controller gain for guidance tracking |

---

## Reference & Citation

This repository is based on the theoretical framework presented in:

> S. Singha, S. Pal, and S. Hota, "Smooth Trajectory Generation for Gap Traversal by UAVs," *2025 IEEE Space, Aerospace and Defence Conference (SPACE)*, 2025, doi: 10.1109/SPACE65882.2025.11170642.

```bibtex
@INPROCEEDINGS{11170642,
  author={Singha, Sandip and Pal, Sayantan and Hota, Sikha},
  booktitle={2025 IEEE Space, Aerospace and Defence Conference (SPACE)}, 
  title={Smooth Trajectory Generation for Gap Traversal by UAVs}, 
  year={2025},
  doi={10.1109/SPACE65882.2025.11170642}
}
```
