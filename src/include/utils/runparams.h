#ifndef RUNPARAMS_H
#define RUNPARAMS_H

typedef struct runparams {
  char *run_name;           // name of the run
  char *output_path;        // path to the output directory
  char *trajectory_path;    // path to the trajectory data file
  char *atm_path;           // path to "atmprofiles.csv"
  char *mean_atm_path;      // path to "mean_atm.txt"
  int num_runs;             // number of Monte Carlo runs
  int num_runs_optimizer;   // number of runs to use during trajectory
                            // optimization
  int num_trials_optimizer; // number of Optuna trials to use during
                            // optimization
  double time_step_boost; // time step in seconds during atmospheric boost phase
  double time_step_lambert;   // time step in seconds during exoatmospheric
                              // Lambert guidance phase
  double time_step_midcourse; // time step in seconds during midcourse flight
  double time_step_reentry;   // time step in seconds during reentry
  int traj_output;            // flag to output trajectory data
  double range;               // downrange distance along equator in meters
  double x_aim;               // target x-coordinate in meters
  double y_aim;               // target y-coordinate in meters
  double z_aim;               // target z-coordinate in meters
  double theta_long; // thrust angle in the longitudinal direction in radians
  double theta_lat;  // thrust angle in the latitudinal direction in radians
  int integrator;   // flag for which numerical integration method to use (0 for
                    // modified Euler-Maruyama, 1 for SRA3)
  long random_seed; // RNG seed (-1 = auto-seed)

  int grav_error;       // flag to include gravitational perturbations
  int ballistic_drag;   // flag to use simplified reentry drag calculation
  int atm_model;        // flag to select the atmospheric model
  int gnss_nav;         // flag to include GNSS navigation
  int rv_maneuv;        // flag to include guidance during the reentry phase
  int perfect_boost;    // 1 perfect boost, 0 realistic
  int optimize_boost;   // 1 optimize t_des_final/theta_long/lambert_v_offset,
                        // 0 use provided
  int optimize_reentry; // 1 optimize maneuver params (max_deflection_angle,
                        // nav_gain_0, nav_gain_1, K_q, K_pp, K_delta_p,
                        // K_delta_d), 0 use provided
  double t_des_final;   // desired flight time (optimized)
  double lambert_v_offset; // Lambert velocity offset (optimized)
  double t_vert_boost;     // Duration of vertical boost

  double deflection_time; // time to make full flap deflection in seconds, used
                          // for maneuverability
  double actuator_force;  // actuator max force in kilonewtons, used for
                          // maneuverability
  double gearing_ratio;   // gearing ratio of the control surfaces, used for
                          // maneuverability
  double actuator_resolution;  // actuator angular resolution in degrees
  double max_deflection_angle; // maximum flap deflection angle in degrees
                               // (optimized)
  double nav_gain_0;
  double nav_gain_1;
  double K_q;
  double K_pp;
  double K_delta_p;
  double K_delta_d;

  double initial_x_error;        // initial x-error in meters
  double initial_pos_error;      // initial position error in meters
  double initial_vel_error;      // initial velocity error in meters per second
  double initial_angle_error;    // initial angle error in radians
  double acc_scale_stability;    // accelerometer scale stability in ppm
  double gyro_bias_stability;    // gyro bias stability in rad/s
  double gyro_noise;             // gyro noise in rad/s/sqrt(s)
  double gnss_noise;             // GNSS error in meters
  double gnss_freq;              // GNSS update frequency in Hz
  double roll_gyro_error_factor; // Multiplicative error factor for roll gyro
  // bias error
  double
      geoid_height_error; // standard deviation of geoid height error in meters

  // Booster burn time error standard deviation
  double burn_time_error; // Applied independently to each stage in seconds

} runparams;

#endif