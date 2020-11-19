# Coupled Lateral and Longitudinal MPC for Lateral Stability and Rollover Prevention
This repository contains the code used in the following paper submitted for publication: 
- "COUPLED LATERAL AND LONGITUDINAL CONTROL FOR TRAJECTORY TRACKING, LATERAL STABILITY, AND ROLLOVER PREVENTION USING MINIMUM-TIME PREDICTIVE CONTROL IN AUTOMATED DRIVING"

Directory descriptions are as follows:

1. **src**

   MPC controller, vehicle models, reference trajectory, and other functions as described in the paper.

2. **plots_5.1_verification**

   Code to reproduce plots verifying model accuracy against Carsim for a step input (Section 5.1, Figure 6).

3. **plots_5.2~5.4.1_tracking**

   Code to reproduce plots demonstrating controller tracking ability for reference trajectory with continuously varying curvature (Sections 5.2-5.4.1, Figures 7, 8, 10, 11).

4. **plots_5.4.2_stability**

   Code to reproduce plots showing resulting stability margins and bounded outputs (Section 5.4.2, Figure 12).

## Highlights
- Ready-to-use MATLAB code
- 8-DOF MPC Controller
- 14-DOF Plant Model
- Reproducible Figures

## Installation
1. Clone this repository.
```
git clone https://github.com/uwsbel/LateralRollStabilityMPC.git
```
2. Open MATLAB and navigate to the repository to run files.
- Note that you may have to set your MATLAB working directory to the parent directory of a file you wish to run.

## Quick Start
Use the following steps to reproduce the plots generated in the paper.

#### Model Verification
1. In MATLAB, navigate to the `plots_5.1_verification` directory. 
2. Run `plot_veh_verification.m`. The file will automatically load required data and produce the verification plots (Figure 6).

*Figure 6 (a,b,c,d)*
<p align="left">
  <img src="https://github.com/projectchrono/chrono-web-assets/blob/master/Images/LatLonMPC_Shuping/fig_6a.png" alt="figure 6a" width="180">  
  <img src="https://github.com/projectchrono/chrono-web-assets/blob/master/Images/LatLonMPC_Shuping/fig_6b.png" alt="figure 6b" width="180"> 
  <img src="https://github.com/projectchrono/chrono-web-assets/blob/master/Images/LatLonMPC_Shuping/fig_6c.png" alt="figure 6c" width="180"> 
  <img src="https://github.com/projectchrono/chrono-web-assets/blob/master/Images/LatLonMPC_Shuping/fig_6d.png" alt="figure 6d" width="180">
</p>

#### Controller Tracking Performance
1. In MATLAB, navigate to the `plots_5.2~5.4.1_tracking` directory. 
2. Run `plot_path_tracking.m`. The file will automatically load required data and produce the reference trajectory and curvature (Figure 7), minimum-time speed profile (Figure 8), trajectory tracking results (Figure 10), and control inputs for steering and driving torque (Figure 11).

*Figure 7 (a,b)*
<p align="left">
  <img src="https://github.com/projectchrono/chrono-web-assets/blob/master/Images/LatLonMPC_Shuping/fig_7a.png" alt="figure 7a" width="180">  
  <img src="https://github.com/projectchrono/chrono-web-assets/blob/master/Images/LatLonMPC_Shuping/fig_7b.png" alt="figure 7b" width="180"> 
</p>

*Figure 8*
<p align="left">
  <img src="https://github.com/projectchrono/chrono-web-assets/blob/master/Images/LatLonMPC_Shuping/fig_8.png" alt="figure 8" width="180"> 
</p>

*Figure 10 (a,b,c,d)*
<p align="left">
  <img src="https://github.com/projectchrono/chrono-web-assets/blob/master/Images/LatLonMPC_Shuping/fig_10a.png" alt="figure 10a" width="180"> 
  <img src="https://github.com/projectchrono/chrono-web-assets/blob/master/Images/LatLonMPC_Shuping/fig_10b.png" alt="figure 10b" width="180"> 
  <img src="https://github.com/projectchrono/chrono-web-assets/blob/master/Images/LatLonMPC_Shuping/fig_10c.png" alt="figure 10c" width="180"> 
  <img src="https://github.com/projectchrono/chrono-web-assets/blob/master/Images/LatLonMPC_Shuping/fig_10d.png" alt="figure 10d" width="180"> 
</p>

*Figure 11 (a,b)*
<p align="left">
  <img src="https://github.com/projectchrono/chrono-web-assets/blob/master/Images/LatLonMPC_Shuping/fig_11a.png" alt="figure 11a" width="180"> 
  <img src="https://github.com/projectchrono/chrono-web-assets/blob/master/Images/LatLonMPC_Shuping/fig_11b.png" alt="figure 11b" width="180"> 
</p>

#### Stability and Output Results
1. In MATLAB, navigate to the `plots_5.4.2_stability` directory. 
2. Run `plot_stability.m`. The file will automatically load required data and produce the plots demonstrating stability and bounded output (Figure 12).

*Figure 12 (a,b,c,d,e)*
<p align="left">
  <img src="https://github.com/projectchrono/chrono-web-assets/blob/master/Images/LatLonMPC_Shuping/fig_12a.png" alt="figure 12a" width="180"> 
  <img src="https://github.com/projectchrono/chrono-web-assets/blob/master/Images/LatLonMPC_Shuping/fig_12b.png" alt="figure 12b" width="180"> 
  <img src="https://github.com/projectchrono/chrono-web-assets/blob/master/Images/LatLonMPC_Shuping/fig_12c.png" alt="figure 12c" width="180"> 
  <img src="https://github.com/projectchrono/chrono-web-assets/blob/master/Images/LatLonMPC_Shuping/fig_12d.png" alt="figure 12d" width="180"> 
  <img src="https://github.com/projectchrono/chrono-web-assets/blob/master/Images/LatLonMPC_Shuping/fig_12e.png" alt="figure 12e" width="180"> 
</p>

## Additional Use
Use the files in the `src` directory to experiment with the controller directly. A short description of each file is as follows:
- `controller_8dofC14DOF_PID_6c_Ny3_soft.m` is the MPC controller that uses an 8-DOF model for predictions. Model parameters are configured as described in Table 1 of the paper. 
- `cal_ay.m` is a function that calculates the 8-DOF model as referenced in Figure 1 of the paper.
- `plant_14DOF_vehicle_4.m` is a function that calculates the  14-DOF model as referenced in Figure 2 of the paper.
- `references1.m` is a function that generates the reference trajectory from a given set of path points.
- `pathpoints_arc.xlsx` is a spreadsheet of path points used for generating the reference trajectory.

