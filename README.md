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

- Spiral                                           
<p align="centre">
  <img src="Gifs/circle.gif" alt="step" width="320"> 
</p>
- Diamond
<p align="centre">
  <img src="Gifs/diamond.gif" alt="step" width="320">
</p>

## Path Planning and Trajectory Generation
- Run the code in plann\_traj/runsim.m
- To load other maps change map in plann\_traj/runsim.m. 
  Additionally change the start and end position accordingly.

#### Minimum Acceleration Trajectory

<p align="centre">
  <img src="Gifs/astar_on_trj.gif" alt="step" width="320">
</p>
