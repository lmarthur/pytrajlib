#ifndef RUN_PARAMS_H
#define RUN_PARAMS_H

#include "math/linalg.h"
#include "rng/rng.h"
#include <math.h>
typedef struct runparams {
    // char *run_name; // name of the run
    // int run_type; // 0 for full trajectory, 1 for reentry only
    // char *impact_data_path; // path to the impact data file
    // char *trajectory_path; // path to the trajectory data file
    // char *atm_profile_path; // path to the atmospheric profile file
    // char *mean_atm_profile_path; // path to the mean atmospheric profile file
    // int num_runs; // number of Monte Carlo runs
    // double time_step_main; // time step in seconds during boost and outside
    // the atmosphere double time_step_reentry; // time step in seconds during
    // reentry int traj_output; // flag to output trajectory data int
    // impact_output; // flag to output impact data double x_launch; // launch
    // x-coordinate in meters double y_launch; // launch y-coordinate in meters
    // double z_launch; // launch z-coordinate in meters
    cartvec aimpoint;  // target coordinates in meters
    double theta_long; // thrust angle in the longitudinal direction in radians
    double theta_lat;  // thrust angle in the latitudinal direction in radians

    int grav_error; // flag to include gravitational perturbations
    int atm_model;  // atmospheric model: 0=exponential, 1=exponential+wind,
                    // 2=EarthGRAM, 3=mean EarthGRAM
    int gnss_nav;   // flag to include GNSS navigation
    int ins_nav;    // flag to include INS navigation
    int rv_maneuv;  // If set to 1, enables RV proportional navigation w/
                    // realistic maneuverability, if set to 2, idealized
                    // maneuverability
    int rv_type;    // type of reentry vehicle (0: ballistic, 1: maneuverable)
    // double vx_reentry; // reentry x-velocity in meters per second
    // double vy_reentry; // reentry y-velocity in meters per second
    // double vz_reentry; // reentry z-velocity in meters per second
    // double x_reentry; // reentry x-position in meters
    // double y_reentry; // reentry y-position in meters
    // double z_reentry; // reentry z-position in meters

    // int booster_type; // type of booster (0: MMIII, 1: SCUD, 2: SCUD-ER, 3:
    // GBSD, 4: D5, 5: Mock)
    double deflection_time; // time to make full flap deflection in seconds,
                            // used for maneuverability
    double actuator_force;  // actuator max force in kilonewtons, used for
                            // maneuverability
    double gearing_ratio;   // gearing ratio of the control surfaces, used for
                            // maneuverability
    double nav_gain;  // navigation gain for proportional navigation guidance
    double flap_gain; // Gain for approaching the commanded flap position when
                      // slower than max rate.

    // double initial_x_error; // initial x-error in meters

    // Error parameters
    double initial_pos_error;   // initial position error in meters
    double initial_vel_error;   // initial velocity error in meters per second
    double initial_angle_error; // initial angle error in radians
    double acc_scale_stability; // accelerometer scale stability in ppm
    double gyro_bias_stability; // gyro bias stability in rad/s
    double gyro_noise;          // gyro noise in rad/s/sqrt(s)
    double gnss_noise;          // GNSS error in meters
    // double cl_pert; // Coefficient of lift perturbation
    // double step_acc_mag; // Step acceleration perturbation magnitude
    // double step_acc_hgt; // Step acceleration perturbation height (altitude)
    // in meters double step_acc_dur; // Step acceleration perturbation duration
    // in seconds double step_acc_angle; // Step acceleration perturbation angle
    // in radians. 0 = along lift vector, -1 is single random direction for
    // duration of run

    double boost_dt;
    double midcourse_dt;
    double reentry_dt;

    double init_thrust_lat_pert; // Not set by user. Derived from other params
    double init_thrust_lon_pert; // Not set by user. Derived from other params

} runparams;

runparams init_base_run_params() {
    runparams run_params;

    run_params.theta_lat = 0.0;
    run_params.theta_long = 1.04719755;

    // Set time steps for each phase
    run_params.boost_dt = 1.0;
    run_params.midcourse_dt = 1.0;
    run_params.reentry_dt = 0.01;

    return run_params;
}

runparams init_base_error_run_params() {
    runparams run_params = init_base_run_params();

    run_params.atm_model = 1; // exponential + wind

    // Set error params
    run_params.grav_error = 1;
    run_params.initial_pos_error = 1e-1;
    run_params.initial_vel_error = 1e-3;
    run_params.initial_angle_error = 1e-6;
    run_params.acc_scale_stability = 1e-6;
    run_params.gyro_bias_stability = 1e-8;
    run_params.gyro_noise = 1e-8;
    run_params.gnss_noise = 1e-1;

    double initial_rot_pert = run_params.initial_angle_error * ran_gaussian(1);
    run_params.init_thrust_lat_pert =
        run_params.initial_angle_error * ran_gaussian(1) +
        run_params.theta_long * initial_rot_pert -
        fabs(run_params.theta_lat * initial_rot_pert);
    run_params.init_thrust_lon_pert =
        run_params.initial_angle_error * ran_gaussian(1) -
        run_params.theta_lat * initial_rot_pert -
        fabs(run_params.theta_long * initial_rot_pert);

    return run_params;
}

runparams init_aimpoint_run_params(int rv_type, int atm_model) {
    runparams run_params = init_base_run_params();

    run_params.ins_nav = 0;
    run_params.gnss_nav = 0;

    run_params.rv_maneuv = 0;
    run_params.rv_type = rv_type;

    // Use the mean EarthGRAM model instead of a specific one for aimpoint
    // calculations if any EarthGRAM model is used
    if (atm_model >= 2) {
        run_params.atm_model = 3;
    } else {
        run_params.atm_model = 0;
    }

    // Set error params to zero
    run_params.grav_error = 0;
    run_params.initial_pos_error = 0.0;
    run_params.initial_vel_error = 0.0;
    run_params.initial_angle_error = 0.0;
    run_params.acc_scale_stability = 0.0;
    run_params.gyro_bias_stability = 0.0;
    run_params.gyro_noise = 0.0;

    return run_params;
}

runparams init_ballistic_run_params(cartvec aimpoint) {
    runparams run_params = init_base_error_run_params();
    run_params.aimpoint = aimpoint;

    run_params.rv_maneuv = 0; // no maneuverability
    run_params.ins_nav = 0;
    run_params.gnss_nav = 0;

    return run_params;
}

runparams init_ins_run_params(cartvec aimpoint) {
    runparams run_params = init_base_error_run_params();
    run_params.aimpoint = aimpoint;

    run_params.rv_maneuv = 2; // perfect maneuverability
    run_params.ins_nav = 1;
    run_params.gnss_nav = 0;

    return run_params;
}

runparams init_ins_gnss_run_params(cartvec aimpoint) {
    runparams run_params = init_base_error_run_params();
    run_params.aimpoint = aimpoint;

    run_params.rv_maneuv = 2; // perfect maneuverability
    run_params.ins_nav = 1;
    run_params.gnss_nav = 1;

    return run_params;
}

// run_params.flap_gain = 100;

#endif