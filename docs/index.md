# Get started
`pytrajlib` is a library written in a combination of C and Python whose purpose is to simulate trajectories of missiles with ballistic and maneuvering reentry vehicles and determine their accuracy. Accuracy limits arise from errors that can be broadly categorized into guidance errors or control limitations. `pytrajlib`'s key output is the distribution of miss distances. Samples of this distribution are obtained by Monte-Carlo sampling of error factors and trajectories. 

!!! warning
    This code is a work in progress, and is not yet ready for use. It is being developed for research purposes in the MIT Laboratory for Nuclear Security and Policy.


## Installation
`pytrajlib` is pip-installable, though we recommend using a packaging manager such as [`uv`](https://docs.astral.sh/uv/). To install, run
```bash
pip install pytrajlib
```

## Quickstart
`pytrajlib` can be run as a command-line tool to facilitate use in larger scripts or other programming languages. For a list of commands, try
```bash
pytrajlib --help
```

To quickly get started, run
```bash
pytrajlib --num-runs 10 --num-processes 2
```

The primary outputs of interest are the impact data and the impact scatter plot, which shows the impact locations with downrange and crossrange components. These plots are created by default and saved by default to `output/rv-maneuv/impact_plot.png` along with detailed trajectory and reentry guidance information about the first run in CSV files.

To examine the first trajectory's characteristics, use the `--plot-trajectory` flag
```bash
pytrajlib --num-runs 10 --num-processes 2 --plot-trajectory
```

You can also use `pytrajlib` like any other Python package from a Python file or notebook:
```python
import pytrajlib as ptl

impact_df = ptl.run(num_runs=10, num_processes=2, plot_trajectory=True)
print(impact_df)
```

Pytrajlib supports two primary reentry vehicle (RV) types: ballistic and maneuverable.

Maneuverable RVs are the default. A ballistic RV can be specified with the `--rv-maneuv 0` flag:
```python
# maneuverable (default)
pytrajlib --rv-maneuv 1


# ballistic without accurate positioning updates
pytrajlib --rv-maneuv 0 --gnss-nav 0
```


## Detailed usage
All simulation and vehicle configs can be changed using a configuration .json file.
Run the modified simulation with
```bash
pytrajlib --config path-to-your-config.json
```

The default configuration is
```json
{
  # run parameters
  "run": {
    "run_name": "rv-maneuv", # Run identifier used for output folders and artifacts.
    "num_runs": 200, # Number of simulation runs to execute.
    "num_runs_optimizer": 50, # Number of Monte Carlo runs used by the boost and reentry optimizers.
    "num_trials_optimizer": 100, # Number of optimization trials per optimizer run.
    "time_step_boost": 0.001, # Time step used during the boost phase, in seconds.
    "time_step_lambert": 0.0001, # Time step used during Lambert maneuver, in seconds.
    "time_step_midcourse": 1.0, # Time step used during the midcourse phase, in seconds.
    "time_step_reentry": 0.0001, # Time step used during the reentry phase, in seconds.
    "traj_output": 1, # Write trajectory output logs for the first run with 1, and disable with 0.
    "range": 10000000, # Downrange distance in meters; supersedes the aimpoint.
    "x_aim": null, # Target aimpoint x-coordinate in meters.
    "y_aim": null, # Target aimpoint y-coordinate in meters.
    "z_aim": null, # Target aimpoint z-coordinate in meters.
    "integrator": 1, # Integrator selection; 0 is modified Euler-Maruyama, 1 is SRA3.
    "random_seed": -1, # Random seed used to initialize stochastic simulation inputs.
    "atm_path": null, # Path to the atmospheric profiles file used by the simulation.
    "optimize_boost": 0, # Optimize t_des_final and theta_long when set to 1.
    "optimize_reentry": 0 # Optimize reentry maneuver parameters (max_deflection_angle, gearing_ratio, nav_gain_0, nav_gain_1, K_q, K_pp, K_delta_p, K_delta_d) when set to 1.
  },
  # flight parameters
  "flight": {
    "grav_error": 1, # Enable the gravitational error model.
    "ballistic_drag": 0, # Use simplified drag; 1 enables it and 0 disables it.
    "atm_model": 2, # Atmospheric model selection; 0 is exponential, 1 adds perturbations, 2 is EarthGram, 3 is mean EarthGram.
    "gnss_nav": 1, # Enable GNSS position updates during exoatmospheric flight.
    "rv_maneuv": 1, # Reentry vehicle maneuverability mode; 1 uses realistic maneuverability, 2 uses idealized maneuverability.
    "perfect_boost": 0, # Set to 1 for a perfect boost phase and 0 for a realistic boost phase.
    "t_vert_boost": 10, # Vertical boost time, in seconds.
    "deflection_time": 0.02, # Actuator deflection time, in seconds.
    "actuator_force": 100, # Maximum actuator force in kN.
    "actuator_resolution": 0.01 # Actuator resolution in degrees.
  },
  # optimized parameters
  "optimized": {
    "theta_long": 0.6715065960788051, # Thrust angle from x axis in x-y plane.
    "theta_lat": 0.0, # Thrust angle above x-y plane.
    "t_des_final": 2986.6467267274857, # Desired final time for the boost phase, in seconds.
    "gearing_ratio": 18.902565748694002, # Actuator gearing ratio. Higher gearing ratios correspond to increased max force and decreased max speed.
    "max_deflection_angle": 5.005158800904358, # Maximum deflection angle allowed for the reentry vehicle in degrees.
    "nav_gain_0": 17.16392199274926, # Navigation gain at surface used by the reentry guidance law.
    "nav_gain_1": 1.8450079089106168, # Navigation gain at reentry used by the reentry guidance law.
    "K_q": 9.93839564932045, # Pitch-rate feedback gain.
    "K_pp": 12.011089458530302, # Proportional restoring angle of attack gain.
    "K_delta_p": 0.2397109503235244, # Proportional deflection gain.
    "K_delta_d": 12.81326407331284 # Derivative deflection gain.
  },
  # error parameters
  "error": {
    "initial_x_error": 0.0, # Initial x-position error.
    "initial_pos_error": 0.1, # Initial position error magnitude.
    "initial_vel_error": 0.001, # Initial velocity error magnitude.
    "initial_angle_error": 1e-06, # Initial angle error magnitude.
    "acc_scale_stability": 1e-06, # Accelerometer scale-factor stability.
    "gyro_bias_stability": 1e-08, # Gyroscope bias stability.
    "gyro_noise": 1e-08, # Gyroscope noise level.
    "gnss_noise": 0.1, # GNSS measurement noise level.
    "gnss_freq": 1.0, # GNSS update frequency in Hz.
    "roll_gyro_error_factor": 0.0, # Roll gyroscope error scaling factor.
    "burn_time_error": 0.1 # Burn time error magnitude in seconds.
  },
  # The vehicle parameters must be set in the json config. There is no command line support for modifying them.
  "vehicle": {
    # The default booster is based on the MMIII booster with three stages and a 
    # total burn time of 188 seconds.
    "booster": {
      "name": "MMIII",
      "area": 2.2698,
      "bus_mass": 100.0,
      "c_d_0": 0.15, # drag coefficient
      "stages": [
        {
          "wet_mass": 23230.0, # total mass of stage including fuel
          "fuel_mass": 20780.0,
          "isp0": 2619.27, # Isp * 9.81
          "burn_time": 61.0
        },
        {
          "wet_mass": 7270.0,
          "fuel_mass": 6240.0,
          "isp0": 2815.47,
          "burn_time": 66.0
        },
        {
          "wet_mass": 3710.0,
          "fuel_mass": 3306.0,
          "isp0": 2795.85,
          "burn_time": 61.0
        }
      ]
    },
    # The default reentry vehicle is based on SWERVE
    "rv": {
      "name": "SWERVE",
      "maneuverability_flag": 1,
      "rv_mass": 450.0,
      # Reference length is the tip to base length (m)
      "rv_length": 2.75,
      # Reference radius is the base radius (m)
      "rv_radius": 0.277,
      "half_angle": 0.0916, # Cone half angle in radians (5.5 degrees)
      "flap_area": 0.04, # flap area in square meters
      "x_flap": -2.65, # x-coordinate of the flap hinge in meters
      "x_com": -1.65, # x-coordinate of the center of mass in meters
      "Iyy": 290.0, # Moment of inertia around pitch/yaw axis
      "aerodynamics": {
        # If tabulated aerodynamic coefficients are present, then they will be used
        # to calculate the true state of the vehicle and the linear approximations will be 
        # used to calculate the guidance computer's estimated state of the vehicle.
        "c_d_0": 0.018, # Drag coefficient at zero angle of attack
        "c_d_alpha": 0.487, # Drag coefficient derivative per radian angle of attack
        "c_l_alpha": 1.988, # Lift coefficient derivative per radian angle of attack
        "c_m_alpha": -0.111, # Moment coefficient derivative per radian angle of attack
        "c_m_q": -0.429, # Moment coefficient derivative per radian/s angular velocity
        "c_m_delta": 0.059, # Moment coefficient derivative per radian flap deflection extent
        # Tabulated aerodynamic coefficients with angle of attack
        "alpha_deg_table": [
            0.0,0.2,0.4,0.6,0.8,1.0,1.2,1.4,1.6,1.8,2.0,2.2,2.4,2.6,2.8,3.0,3.2,3.4,3.6,3.8,4.0,4.2,4.4,4.6,4.8,5.0,5.2,5.4,5.6,5.8,6.0,6.2,6.4,6.6,6.8,7.0,7.2,7.4,7.6,7.8,8.0,8.2,8.4,8.6,8.8,9.0,9.2,9.4,9.6,9.8,10.0
        ],
        "c_d_table": [
            0.018,0.0181,0.0182,0.0183,0.0185,0.0188,0.0192,0.0196,0.0201,0.0205,0.0212,0.0219,0.0226,0.0234,0.0243,0.0252,0.0262,0.0273,0.0284,0.0296,0.0309,0.0322,0.0337,0.0351,0.0367,0.0383,0.0399,0.0417,0.0436,0.0455,0.0474,0.0494,0.0515,0.0538,0.0559,0.0583,0.0607,0.0632,0.0657,0.0684,0.0711,0.074,0.0769,0.0797,0.0828,0.0861,0.0893,0.0925,0.096,0.0994,0.1031
        ],
        "c_l_table": [
            0.0,0.006,0.012,0.018,0.024,0.03,0.036,0.042,0.048,0.055,0.061,0.067,0.073,0.08,0.086,0.092,0.099,0.105,0.112,0.118,0.125,0.131,0.138,0.144,0.151,0.157,0.164,0.171,0.177,0.184,0.191,0.198,0.205,0.212,0.219,0.226,0.234,0.241,0.249,0.256,0.264,0.272,0.28,0.288,0.296,0.304,0.313,0.321,0.329,0.338,0.347
        ],
        "c_m_table": [
            0.0,-0.0002,-0.0003,-0.0005,-0.0008,-0.001,-0.0012,-0.0014,-0.0016,-0.0018,-0.0021,-0.0023,-0.0026,-0.0029,-0.0032,-0.0035,-0.0038,-0.0041,-0.0043,-0.0047,-0.005,-0.0054,-0.0057,-0.0061,-0.0065,-0.0069,-0.0072,-0.0076,-0.0079,-0.0084,-0.0088,-0.0093,-0.0098,-0.0101,-0.0106,-0.011,-0.0115,-0.012,-0.0126,-0.013,-0.0135,-0.014,-0.0146,-0.0152,-0.0157,-0.0164,-0.0169,-0.0175,-0.0181,-0.0187,-0.0194
        ],
        "c_m_q_table": [
            -0.118,-0.119,-0.12,-0.121,-0.121,-0.122,-0.123,-0.124,-0.125,-0.125,-0.126,-0.127,-0.128,-0.129,-0.129,-0.13,-0.131,-0.131,-0.132,-0.133,-0.134,-0.135,-0.136,-0.136,-0.137,-0.138,-0.138,-0.14,-0.141,-0.143,-0.145,-0.147,-0.149,-0.151,-0.153,-0.155,-0.158,-0.16,-0.163,-0.165,-0.167,-0.17,-0.172,-0.175,-0.177,-0.18,-0.182,-0.185,-0.187,-0.19,-0.193
        ]
      }
    }
  }
}

```