# wall_model

This directory represents most of the techincal work I did Spring-Summer 2018. Of most interest are the models in the SystemPlumbing and MPC folders.

SystemPlumbing is an in depth Simulink model of the hydraulic components in WALL. It provides detailed predictions of the pressure, water flow,
volume level, etc of the components in WALL. The most updated model is wall_plumbing_V2.slx. To run a simulation:

1: Open wall_plumbing_V2.slx and plumbing_variables.m
2: Set the number of simulation days at the top of plumbing_variables.m
3: Run plumbing_variables.m
4: Set the simulation stop time in the wall_plumbing_V2.slx (simulation days * 24*3600)
5: Run wall_plumbing_V2.slx
6: (optional) Run plot_hourly.m to get hourly plots of variables of interest



MPC is my implementation of a model predictive controller for WALL. This seeks to optimize the use of water over a certain time-span.
Ideally this timespan would be around 24 hours, but I was only able to optimize over a timespan of about 1 hour without the run time blowing up.
To run a simulation: 

1: Open WALL_MPC.m
2: Set simulation time (Tf) at the top of the file.
3: Run the WALL_MPC.m
4: Output will be graphs showing variables of interest over the simulation time.
