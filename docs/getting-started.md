# Getting started

TODO separate docs into Instructions/Examples and API or even Basic API and Advanced API

TODO Installation Instructions
- this means putting on pypi


See https://docs.kidger.site/diffrax 


The easiest way to get started with pytrajlib is with `pytrajlib.run` which simulates
multiple trajectories with randomly perturbed initial conditions.

### Example
```python
run_params = init_default_run_params()
N = 10
ts, states = run(N=N, run_params=run_params, maneuverable=True)
# TODO finish this example by perhaps printing a CEP
cep = ...
cep # 35.5 m
```

TODO make run() accessible using `pytrajlib.run()` not `pytrajlib.run.run()` 

??? Info
    TODO don't use words like "we". Instead I could say "Under the hood, the launch
    position is always parametrized as (x=Earth radius, y=0, z=0), even if a different
    launch point is passed. Keep this in mind if you change `thrust_lat` and `thrust_lon`
    manually. 

     - The launch position is always (Earth radius, 0, 0). Because we treat the
     Earth as a sphere (TODO make notes in gravity about the correction effects),
    we can always define launch point as an arbitrary latitude, longitude on the
    real Earth.
    - The thrust angles (`theta_lat` and `theta_lon`) are parametrized to be the
    angles (in radians) from the x-y plane and x-y planes, respectively. After an
    initial 5-second vertical thrust (TODO link to the thrust page) the thrust points
    in the same Cartesian direction for the duration of the boost phase.

    TODO provide options for giving a particular aim point (and perhaps launch point if we want the nice map plotting)
    - this would require doing the Minimum Energy Trajectory Optimization of fuel and thrust angles


# Learning more
TODO see our paper LINK TO PAPER(s)

or read the documentation, starting with the forces because at the most basic level, the trajectory is based on integrating the acceleration caused by four forces:

1. [Drag](API/forces/drag)
2. [Gravity](API/forces/gravity)
3. [Lift](API/forces/lift)
4. [Thrust](API/forces/lift)
