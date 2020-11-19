# Coupled Lateral and Longitudinal MPC for Lateral Stability and Rollover Prevention
This repository contains the code used in the following paper submitted for publication: "COUPLED LATERAL AND LONGITUDINAL CONTROL FOR TRAJECTORY TRACKING, LATERAL STABILITY, AND ROLLOVER PREVENTION USING MINIMUM-TIME PREDICTIVE CONTROL IN AUTOMATED DRIVING". 

Folder descriptions are as follows:

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
1. Clone this repository
```
git clone https://github.com/uwsbel/LateralRollStabilityMPC.git
```
2. Open MATLAB and navigate to the repository to run files
- Note that you may have to set your MATLAB working directory to the parent folder of a file you wish to run.

## Quick Start
Use the following steps to reproduce the plots generated in the paper.
1. 
