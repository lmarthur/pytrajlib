#ifndef RUNPARAMS_H
#define RUNPARAMS_H

typedef struct runparams {
  char *run_name;         // name of the run
  char *output_path;      // path to the output directory
  char *trajectory_path;  // path to the trajectory data file
  char *atm_path;         // path to "atmprofiles.txt"
  char *mean_atm_path;    // path to "mean_atm.txt"
  int num_runs;           // number of Monte Carlo runs
  int num_runs_optimizer; // number of runs to use during trajectory
                          // optimization
  double
      time_step_lambert; // time step in seconds during Lambert guidance phase
  double time_step_midcourse; // time step in seconds during midcourse flight
  double time_step_atm; // time step in seconds inside atmosphere (reentry and
                        // first part of boost)
  int traj_output;      // flag to output trajectory data
  double range;         // downrange distance along equator in meters
  double x_aim;         // target x-coordinate in meters
  double y_aim;         // target y-coordinate in meters
  double z_aim;         // target z-coordinate in meters
  double theta_long;    // thrust angle in the longitudinal direction in radians
  double theta_lat;     // thrust angle in the latitudinal direction in radians
  int integrator;   // flag for which numerical integration method to use (0 for
                    // modified Euler-Maruyama, 1 for SRA3)
  long random_seed; // RNG seed (-1 = auto-seed)

  int grav_error;      // flag to include gravitational perturbations
  int include_drag;    // flag to include aerodynamic drag force
  int atm_model;       // flag to select the atmospheric model
  int gnss_nav;        // flag to include GNSS navigation
  int rv_maneuv;       // flag to include guidance during the reentry phase
  double reentry_vel;  // reentry velocity in meters per second
  int perfect_boost;   // 1 perfect boost, 0 realistic
  int optimize_boost;  // 1 optimize t_des_final/theta_long, 0 use provided
  int optimize_maneuv; // 1 optimize tau_deflect, 0 use provided
  double t_des_final;  // desired flight time (optimized by code)
  double t_vert_boost; // Duration of vertical boost (optimized by code)

  int rv_type; // reentry vehicle type (0: ballistic, 1: maneuverable)
  double deflection_time; // time to make full flap deflection in seconds, used
                          // for maneuverability
  double actuator_force;  // actuator max force in kilonewtons, used for
                          // maneuverability
  double gearing_ratio;   // gearing ratio of the control surfaces, used for
                          // maneuverability
  double actuator_resolution;  // actuator angular resolution in degrees
  double max_deflection_angle; // maximum flap deflection angle in radians
  double nav_gain;    // navigation gain for proportional navigation guidance
  double flap_gain;   // Gain for approaching the commanded flap position when
                      // slower than max rate.
  double Glp;         // low-pass filter gain used in maneuverability control
  double tau_deflect; // Time constant for deflection control

  double initial_x_error;     // initial x-error in meters
  double initial_pos_error;   // initial position error in meters
  double initial_vel_error;   // initial velocity error in meters per second
  double initial_angle_error; // initial angle error in radians
  double acc_scale_stability; // accelerometer scale stability in ppm
  double gyro_bias_stability; // gyro bias stability in rad/s
  double gyro_noise;          // gyro noise in rad/s/sqrt(s)
  double gnss_noise;          // GNSS error in meters
  double gnss_freq;           // GNSS update frequency in Hz
  double cl_pert;             // Coefficient of lift perturbation
  double step_acc_mag;        // Step acceleration perturbation magnitude
  double step_acc_hgt; // Step acceleration perturbation height (altitude) in
                       // meters
  double step_acc_dur; // Step acceleration perturbation duration in seconds

  // RV parameter overrides (-1 = use vehicle type default)
  double rv_mass;      // RV mass in kg
  double rv_length;    // RV length in m
  double rv_radius;    // RV base radius in m
  double rv_c_d_0;     // RV zero-lift drag coefficient
  double rv_c_d_alpha; // RV drag coefficient derivative (per radian)

  // Booster parameter overrides (-1 = use vehicle type default)
  double booster_area;     // Booster reference area in m^2
  double booster_maxdiam;  // Booster maximum diameter in m
  double booster_c_d_0;    // Booster zero-lift drag coefficient
  double booster_bus_mass; // Bus/payload carrier mass in kg

} runparams;

#endif