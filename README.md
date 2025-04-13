L.M. Arthur, Cambridge, MA, 2025

This code is a work in progress, and is not yet ready for use. It is being developed for research purposes in the MIT Laboratory for Nuclear Security and Policy. 

# Overview
This code is a package for simulating the flight of a ballistic missile. It is designed to be modular, with separate modules for geodesy, atmosphere, vehicle, gravity, stepper, integrator, flight, monte carlo, guidance, and control. The code is written in a combination of C, Python, and bash.

# User Guide
This section provides a brief introduction to using pytrajlib, and will be updated as additional features are rolled out. Run all commands from the pytrajlib directory. To ensure that your version of the code is up-to-date, run 

```git pull```

To compile the code and run the test suite, run 

```bash ./scripts/compile.sh```

And, to run the code, use the 

```bash ./scripts/run.sh```

command. The run parameters can be adjusted in the ```.toml``` files in the ```/input``` directory. The results will be placed in the ```/output``` directory. 

To generate trajectory plots from an existing ```trajectory.txt``` file, run 

```python ./src/traj_plot.py```

To generate a new ```trajectory.txt``` file, run the simulation with ```traj_output = 1``` in the relevant ```.toml``` file. 

## TODO: 
- [X] Set up CMake 
- [X] Write tests for atmosphere module
- [X] Implement exp atmosphere functions in C
- [X] Write tests for vehicle module
- [X] Implement minuteman model in C
- [X] Write tests for newtonian gravity module
- [X] Implement newtonian gravity functions in C
- [X] Write tests for physics module
- [X] Implement physics functions in C
- [X] Write tests for stepper
- [X] Implement stepper in C
- [X] Determine whether to implement integration in C or Python
- [X] Write tests for flight module
- [X] Implement flight function
- [X] Implement ctypes wrapper for C functions 
- [X] Implement output file writer
- [X] Implement plotting functions
- [X] Write scripts for compiling, testing, and running
- [X] Write tests for flight initialization
- [X] Implement flight initialization function
- [X] Write analysis suite to fit distributions to accuracy data
- [X] Implement configuration file and parser
- [X] Write tests for monte carlo function
- [X] Implement monte carlo function in C
- [X] Write tests for imu module
- [X] Implement imu functions in C
- [X] Write tests for gnss module
- [X] Implement gnss functions in C
- [X] Write tests for control module
- [X] Implement control functions in C
- [X] Write tests for Kalman filter
- [X] Implement Kalman filter in C
- [X] Set up .yaml for conda environment
- [X] Write tests for gravity perturbations
- [X] Implement gravity perturbations in C
- [X] Implement atmospheric model with perturbations
- [X] Integrate boost guidance
- [X] Write tests for lift function
- [X] Implement lift function in C
- [X] Integrate proportional navigation for MaRV
- [X] Generate plots
- [X] Reentry update
- [X] Write integration tests
- [X] Add initial rotational perturbation
- [X] Exclude gravity from imu measurements
- [X] Write function for parameter scanning
- [X] Write function to generate sensitivity plots
- [X] Organize runs
- [X] Add non-commercial license
- [X] Set up EG16 atmosphere
- [X] Implement reentry-only simulation
- [ ] Generate system response plots

## BUG REPORTS: 
None
