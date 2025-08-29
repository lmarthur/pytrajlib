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
- [ ] Determine the optimal values of the navigation gain and the gearing ratio
- [ ] Plots: (Note that all of these should be done with the optimal nav gain and gearing)
    - [ ] Initial position error vs miss distance (or CEP)
    - [ ] Initial velocity error vs miss distance (or CEP)
    - [ ] Pitching mode excitation magnitude vs miss distance (Use a step function to model the pitching mode excitation and note that the expected excitation magnitude and duration is a function of velocity and altitude)
    - [ ] Boundary layer excitation magnitude vs miss distance (Note that this is probably negligible, we may not even want to include a plot)
    - [ ] Anomalous lift coefficient vs miss distance
- [ ] Explore the relationship between reentry speed, reentry angle, and accuracy

## BUG REPORTS: 
None
