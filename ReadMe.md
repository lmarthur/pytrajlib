L.M. Arthur, Cambridge, MA, 2024

This code is a work in progress, and is not yet ready for use. It is being developed for research purposes in the MIT Laboratory for Nuclear Security and Policy. 

# Overview
This code is a package for simulating the flight of a ballistic missile. It is designed to be modular, with separate modules for geodesy, atmosphere, vehicle, gravity, stepper, integrator, flight, monte carlo, guidance, control, and Kalman filtering. The code is written in a combination of C, Python, and bash.

# User Guide
This section provides a brief introduction to using the PyTraj tool, and will be updated as additional features are rolled out. Run all commands from the PyTraj directory. To ensure that your version of the code is up-to-date, simply run 

```git pull```

To compile the code and run the test suite, run 

```source ./scripts/compile.sh```

And, to run the code, use the 

```source ./scripts/run.sh```

command. The run parameters can be adjusted in the ```.toml``` files in the ```/input``` directory. The results will be placed in the ```/output``` directory. 

To generate trajectory plots from an existing ```trajectory.txt``` file, run 

```python ./src/traj_plot.py```

To generate a new ```trajectory.txt``` file, run the simulation with ```traj_output = 1``` in the relevant ```.toml``` file. 

## TODO: 
- [ ] Generate plots of miss distance vs response time
- [X] Generate plot of miss distance vs gearing for a range of nav gains
- [ ] Rerun plots with multiple atmospheres
- [ ] Explore noise random walks
- [ ] Explore initial reentry velocity errors
- [ ] Merge branches

## BUG REPORTS: 
None
