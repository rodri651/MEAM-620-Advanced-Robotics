## Quadrotor Control, Path Planning, Trajectory generation in static Indoor Enviorenments

As a part of the MEAM 620 Advanced Robotics Course, Matlab code for the following was written
- PD Controller 
- A\* and Dijstra and Path planning Algorithms 
- Minimum acceleration Trajectory optimisation 

## PD Controller

- Change trajectories in Controller/runsim.m to circle or diamond or x\_motion
- Please refer [modelling_quadrotor_dynamics.pdf](modelling_quadrotor_dynamics.pdf) to model the controller of the quadrotor
- Vizualisations [Desired (blue) and Actual (red)]

### Some Results


<img src="Gifs/circle.gif" width="320" title="Spiral"><img src="Gifs/diamond.gif" width="320" title="Diamond">


## Path Planning and Trajectory Generation
- Run the code in plann\_traj/runsim.m
- To load other maps change map in plann\_traj/runsim.m. 
  Additionally change the start and end position accordingly.

#### Minimum Acceleration Trajectory

<p align="centre">
  <img src="Gifs/traj_planner.gif"  width="600">
</p>

-Path planning using A\*
-The path generated is then searched through for shorter paths while simultaneously checking for mid path collisions
-Further a Minimum Acceleration trajectory generator is used over the smoothed path

## Vision based pose and velocity estimates with Extended Kalman Filter

Matlab Code for the following was written
- Vision based 3D pose estimate
- Vision based 3D(linear and angular) velocity estimate
- Extended Kalman Filter

### Some Results
Note: Ground truth estimates were taken from Vicon data(blue)
- Vision based Position and Orientation
<p align="centre">
  <img src="performance_plots/project2_1.PNG" width="950"> 
</p>

- EKF based position, velocity and orientation estimates
<p align="centre">
  <img src="performance_plots/position.jpg" width="560"> 
</p>
<p align="centre">
  <img src="performance_plots/orientation.jpg" width="560"> 
</p>
<p align="centre">
  <img src="performance_plots/velocity.jpg" width="560"> 
</p>




