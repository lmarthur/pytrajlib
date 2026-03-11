#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "forces/drag.h"
#include "forces/gravity.h"
#include "forces/lift.h"
#include "forces/thrust.h"
#include "integrator.h"
#include "models/atmosphere.h"
#include "models/grav.h"
#include "models/sensors.h"
#include "models/vehicle.h"
#include "rng/rng.h"
#include "utils.h"

// Define a constant upper limit for the number of Monte Carlo runs
#define MAX_RUNS 1000

// Define a struct to store impact data
typedef struct impact_data {
  // Impact data
  state impact_states[MAX_RUNS];
  double impact_times[MAX_RUNS];

} impact_data;

/**
 * Interpolate between two states to estimate impact crossing at altitude 0.
 *
 * @param state_0 Pointer to pre-impact state
 * @param state_1 Pointer to post-impact state
 * @return Interpolated impact state
 */
state impact_linterp(state *state_0, state *state_1, double t0, double t1,
                     double *impact_t) {

  // Calculate the interpolation factor
  double altitude_0 = norm(state_0->position) - EARTH_RADIUS_M;
  double altitude_1 = norm(state_1->position) - EARTH_RADIUS_M;
  double interp_factor = altitude_0 / (altitude_0 - altitude_1);

  // Perform the interpolation
  state impact_state = *state_0;
  *impact_t = t0 + interp_factor * (t1 - t0);
  impact_state.position = add(
      state_0->position,
      smultiply(subtract(state_1->position, state_0->position), interp_factor));
  impact_state.velocity = add(
      state_0->velocity,
      smultiply(subtract(state_1->velocity, state_0->velocity), interp_factor));
  impact_state.a_lift =
      add(state_0->a_lift,
          smultiply(subtract(state_1->a_lift, state_0->a_lift), interp_factor));

  return impact_state;
}

/**
 * Computes interpolated impact states and applies coriolis/aimpoint correction.
 *
 * @param old_true_state Pointer to pre-impact true state
 * @param true_state Pointer to post-impact true state
 * @param old_true_t Pre-impact true time
 * @param true_t Post-impact true time
 * @param old_est_state Pointer to pre-impact estimated state
 * @param est_state Pointer to post-impact estimated state
 * @param old_est_t Pre-impact estimated time
 * @param est_t Post-impact estimated time
 * @param run_params Pointer to run configuration parameters
 * @param true_final_t Output interpolated true impact time
 * @param est_final_state Output interpolated estimated impact state
 * @return Corrected true impact state
 */
state impact_with_coriolis(state *old_true_state, state *true_state,
                           double old_true_t, double true_t,
                           state *old_est_state, state *est_state,
                           double old_est_t, double est_t,
                           runparams *run_params, double *true_final_t,
                           state *est_final_state) {
  double est_final_t;
  state true_final_state = impact_linterp(old_true_state, true_state,
                                          old_true_t, true_t, true_final_t);
  *est_final_state =
      impact_linterp(old_est_state, est_state, old_est_t, est_t, &est_final_t);

  // Add coriolis effect based on the latitude and the impact time error
  double lat = ran_flat(-M_PI / 2, M_PI / 2);
  double lon = ran_flat(-M_PI, M_PI);
  double time_error = *true_final_t - est_final_t;
  double rot_speed = 464 * cos(lat);
  double coriolis = rot_speed * time_error;

  // based on the coriolis effect, update the final state x and y
  // This might seem like a bug, but I promise it's just clever
  // This replicates flying in a random direction, not just along the
  // equator
  true_final_state.position.x =
      true_final_state.position.x - coriolis * sin(lon) * cos(lat);
  true_final_state.position.y =
      true_final_state.position.y + coriolis * cos(lon) * cos(lat);
  true_final_state.position.z =
      true_final_state.position.z + coriolis * sin(lat);
  if (run_params->rv_maneuv == 2) {
    // If perfect rv maneuver, update the final position
    true_final_state.position =
        subtract(true_final_state.position, est_final_state->position);
  }

  return true_final_state;
}

/**
 * Write impact state data for all Monte Carlo runs to a file stream.
 *
 * @param impact_file Output file stream
 * @param impact_data Pointer to impact data container
 * @param num_runs Number of Monte Carlo runs to write
 */
void output_impact(FILE *impact_file, impact_data *impact_data, int num_runs) {

  // Iterate through the number of runs and output the impact data
  for (int i = 0; i < num_runs; i++) {
    fprintf(impact_file, "%f, %f, %f, %f, %f, %f, %f\n",
            impact_data->impact_times[i],
            impact_data->impact_states[i].position.x,
            impact_data->impact_states[i].position.y,
            impact_data->impact_states[i].position.z,
            impact_data->impact_states[i].velocity.x,
            impact_data->impact_states[i].velocity.y,
            impact_data->impact_states[i].velocity.z);
  }

  // Close the impact file
  fclose(impact_file);
}

/**
 * Simulate a single trajectory until impact and return final impact state.
 *
 * @param run_params Pointer to run configuration parameters
 * @param initial_state Pointer to initial true state
 * @param vehicle Pointer to vehicle model/state
 * @return Final impact state
 */
state fly(runparams *run_params, state *initial_state, vehicle *vehicle,
          double *impact_time) {

  // Initialize the variables and structures
  int max_steps = 10000000;

  grav true_grav = init_grav(run_params);
  grav est_grav = init_grav(run_params);

  atm_model exp_atm_model = init_exp_atm(run_params);

  // Initialize either a randomly chosen EarthGRAM profile or the average
  // EarthGRAM profile
  eg16_profile atm_profile;
  if (run_params->atm_model == 2) {
    int atm_profile_num = (int)ran_flat(0, 100);
    atm_profile = parse_atm(run_params->atm_path, atm_profile_num);
  } else if (run_params->atm_model == 3) {
    atm_profile = parse_atm(run_params->mean_atm_path, -1);
  }

  state true_state = *initial_state;
  double true_t = 0;

  state est_state = init_est_state(run_params, true_state);
  double est_t = 0;

  int traj_output = run_params->traj_output;
  double time_step;
  // Initialize the IMU
  imu imu = imu_init(run_params, initial_state);

  // Initialize the GNSS
  gnss gnss = gnss_init(run_params);

  // Create a .txt file to store the trajectory data
  FILE *traj_file;
  if (traj_output == 1) {
    traj_file = fopen(run_params->trajectory_path, "w");
    fprintf(
        traj_file,
        "t, current_mass, x, y, z, vx, vy, vz, "
        "a_lift, ax_total, ay_total, az_total, est_x, est_y, est_z, est_vx, "
        "est_vy, est_vz, est_ax_total, est_ay_total, est_az_total, "
        "true_a_lift_x, true_a_lift_y, "
        "true_a_lift_z, est_a_lift_x, est_a_lift_y, est_a_lift_z \n");
    // Write the initial state to the trajectory file
    write_trajectory_state(traj_file, true_t, get_vehicle_mass(vehicle, true_t),
                           &true_state, &est_state);
  }

  // Variables for step function anomaly (only used for run_type = 1)
  double step_timer = 0;       // time since step function was activated
  int sampled_new_profile = 0; // flag to indicate whether a new profile has
                               // been sampled after boost phase

  // Begin the integration loop
  for (int i = 0; i < max_steps; i++) {
    // At the end of boost phase, sample a new atm profile for EarthGram
    // so boost and reentry don't use the same profile
    if ((run_params->atm_model == 2) &&
        (true_t > vehicle->booster.total_burn_time) &&
        (sampled_new_profile == 0)) {
      int atm_profile_num = (int)ran_flat(0, 100);
      atm_profile = parse_atm("input/atmprofiles.txt", atm_profile_num);
      sampled_new_profile = 1;
    }

    // Get the atmospheric conditions
    double old_altitude = get_altitude(true_state.position);

    atm_cond true_atm_cond =
        get_atm_cond(old_altitude, &exp_atm_model, run_params, &atm_profile);
    atm_cond est_atm_cond = get_exp_atm_cond(old_altitude, &exp_atm_model);
    // Use the boost timestep for Lambert Guidance, midcourse timestep for
    // post-boost & before reentry And reentry timestep just before reentry to
    // capture the transition accurately
    if (true_t <= vehicle->booster.total_burn_time) {
      if (old_altitude < 100e3) {
        time_step = run_params->time_step_atm;
      } else {
        time_step = run_params->time_step_lambert;
      }
    } else {
      double angle_v_grav =
          acos(dot(true_state.velocity, smultiply(true_state.position, -1)) /
               (norm(true_state.velocity) * norm(true_state.position)));

      if (fabs(angle_v_grav < M_PI_2) && (old_altitude < 1.2e5)) {
        time_step = run_params->time_step_atm;
      } else {
        time_step = run_params->time_step_midcourse;
      }
    }

    // GNSS Measurement
    gnss.time_since_last_update += time_step;
    if ((run_params->gnss_nav == 1) &&
        (get_altitude(true_state.position) > 100e3) &&
        (gnss.time_since_last_update >= 1.0 / run_params->gnss_freq)) {
      gnss_measurement(&gnss, &true_state, &est_state);
      gnss.time_since_last_update = 0.0;
    }

    state old_true_state = true_state;
    state old_est_state = est_state;
    double old_true_t = true_t;
    double old_est_t = est_t;

    // Perform an integration step
    int success;
    if (run_params->integrator == 0) {
      success = euler_maruyama_step(
          run_params, &imu, vehicle, &true_grav, &est_grav, &true_atm_cond,
          &est_atm_cond, &true_state, &est_state, &true_t, &est_t, time_step);
    } else {
      success = sra3_step(run_params, &imu, vehicle, &true_grav, &est_grav,
                          &true_atm_cond, &est_atm_cond, &true_state,
                          &est_state, &true_t, &est_t, time_step);
    }

    if (!success) {
      return true_state;
    }

    // Check if the vehicle has impacted the Earth
    double new_altitude = get_altitude(true_state.position);
    if (new_altitude < 0) {
      double true_final_t;
      state est_final_state;
      state true_final_state =
          impact_with_coriolis(&old_true_state, &true_state, old_true_t, true_t,
                               &old_est_state, &est_state, old_est_t, est_t,
                               run_params, &true_final_t, &est_final_state);
      if (traj_output == 1) {
        // Write the final state to the trajectory file
        write_trajectory_state(traj_file, true_final_t,
                               get_vehicle_mass(vehicle, true_final_t),
                               &true_final_state, &est_final_state);
        fclose(traj_file);
      }

      *impact_time = true_final_t;

      // Only save full trajectory on the first run.
      run_params->traj_output = 0;

      return true_final_state;
    }

    // output the trajectory data
    if (traj_output == 1) {
      write_trajectory_state(traj_file, true_t,
                             get_vehicle_mass(vehicle, true_t), &true_state,
                             &est_state);
    }
  }

  printf("Warning: Maximum number of steps reached with no impact\n");
  // Close the trajectory file
  if (traj_output == 1) {
    fclose(traj_file);
  }

  // Only save full trajectory on the first run.
  run_params->traj_output = 0;
  *impact_time = true_t;
  return true_state;
}

/**
 * Run a Monte Carlo trajectory simulation.
 *
 * @param run_params Run configuration parameters
 * @return Impact states for all runs
 */
impact_data mc_run(runparams run_params) {
  // Initialize the variables
  int num_runs = run_params.num_runs;
  if (num_runs > MAX_RUNS) {
    printf("Error: Number of runs exceeds the maximum limit. Increase MAX_RUNS "
           "in src/include/trajectory.h and recompile. \n");
    printf("num_runs: %d, MAX_RUNS: %d\n", num_runs, MAX_RUNS);
    exit(1);
  }
  impact_data impact_data;

#ifdef FROM_PYTHON
  update_loading_bar(0, num_runs);
#endif

  for (int i = 0; i < num_runs; i++) {
    vehicle vehicle;
    if (run_params.run_type == 0) {
      if (run_params.rv_type == 0) {
        vehicle = init_mmiii_ballistic();
      } else if (run_params.rv_type == 1) {
        vehicle = init_mmiii_swerve();
      } else {
        printf("Error: Invalid RV type\n");
        exit(1);
      }
    } else if (run_params.run_type == 1) {
      vehicle = init_reentry_only();
    } else {
      printf("Error: Invalid run type\n");
      exit(1);
    }

    state initial_true_state = init_true_state(&run_params);

    impact_data.impact_states[i] = fly(&run_params, &initial_true_state,
                                       &vehicle, &impact_data.impact_times[i]);

#ifdef FROM_PYTHON
    int five_percent = (int)(num_runs / 20);
    five_percent = (int)clip(five_percent, 1, 100);
    if ((i + 1) % five_percent == 0) {
      update_loading_bar(i + 1, num_runs);
    }
    if (i == num_runs - 1) {
      update_loading_bar(num_runs, num_runs);
    }
#endif
  }

  // Output the impact data
  return impact_data;
}

#endif