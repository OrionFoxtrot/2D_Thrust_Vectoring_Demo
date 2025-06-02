# 2D Thrust Vectoring Demo

## Overview

This repository contains MATLAB code and data for modeling, linearizing, and simulating a 2D rigid rectangular body with thrust-vectoring control. The goal is to design a feedback controller that commands thrust-vector angles and total thrust magnitude to drive the body’s planar position and orientation to desired setpoints.

Key features include:
- Derivation of the nonlinear equations of motion for a 2D rigid box with thrust‐vectoring.
- Linearization into state‐space form around hover.
- State‐feedback controller design (pole placement).
- Time‐domain simulations that visualize the box’s motion and thrust vectors.
- Example animation (.mp4) demonstrating controlled maneuvers.

## Repository Structure


- **controller_Design.m**  
  MATLAB script that loads the linearized state‐space model, computes state‐feedback gains, and saves the resulting `Controlled_Sys.mat`.

- **state_Space_Linearization.m**  
  Performs symbolic derivation (if needed) and numerical linearization of the nonlinear dynamics around the hover operating point. Outputs `state_space_matricies.mat`.

- **simulations.m**  
  Runs time‐domain simulations of the closed‐loop system (nonlinear M‐file `non_lin_2d_sim.m`). Generates plots of position, attitude, control inputs, and animates the thrust vectors on the rigid box.

- **state_space_matricies.mat**  
  Contains the linearized A, B, C, D matrices saved by `state_Space_Linearization.m`.

- **Controlled_Sys.mat**  
  Contains the state‐space model with state‐feedback gains and any precomputed DC gain scaling.

- **sample_Move_Animation.mp4**  
  Example output of a thrust‐vectoring maneuver run via `simulations.m`. Demonstrates the box moving to a new (x, y, θ) setpoint.


## Installation & Setup

1. **Clone the repository**  
   ```bash
   git clone https://github.com/OrionFoxtrot/2D_Thrust_Vectoring_Demo.git
   cd 2D_Thrust_Vectoring_Demo
