#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#ifdef _WIN32
#include <process.h>
#define getpid _getpid
#else
#include <unistd.h>
#endif

#include "derivatives.h"
#include "integrator.h"
#include "models/atmosphere.h"
#include "models/grav.h"
#include "models/sensors.h"
#include "models/vehicle.h"
#include "physics/drag.h"
#include "physics/gravity.h"
#include "physics/thrust.h"
#include "rng/rng.h"
#include "utils/run_logging.h"
#include "utils/utils.h"

// Define a constant upper limit for the number of Monte Carlo runs
#define MAX_RUNS 1000

// Define a struct to store impact data
typedef struct impact_data {
  // Impact data
  state impact_states[MAX_RUNS];
  double impact_times[MAX_RUNS];
  double burnout_speed[MAX_RUNS];
  double burnout_altitude[MAX_RUNS];
  double burnout_angle[MAX_RUNS];
  double apogee[MAX_RUNS];
  double reentry_speed[MAX_RUNS];
  double reentry_angle[MAX_RUNS];
} impact_data;

/**
 * Calculate flight path angle relative to local horizon.
 *
 * @param position Position vector
 * @param velocity Velocity vector
 * @return Flight path angle in radians
 */
static inline double flight_path_angle(cartvec position, cartvec velocity) {
  // Get angle between velocity and local "up" vector
  double v_norm = norm(velocity);
  double p_norm = norm(position);
  if (v_norm == 0.0 || p_norm == 0.0) {
    // Degenerate case: direction is undefined when either vector has zero
    // magnitude
    return 0.0;
  }
  double angle = M_PI / 2 - acos(dot(velocity, position) / (v_norm * p_norm));
  return angle;
}

/**
 * Target angle = initial angle + rotation
 *
 * Assuming the entry angle = -burnout angle (not entirely true; burnout is
 * ~170km, entry is 100km) Initial angle = 90 - burnout flight path angle Target
 * angle = 90 - entry angle + central angle between position and aimpoint
 * (simplifying assumption that reentry is close to the aimpoint)
 * -->
 * rotation = burnout flight path angle - entry angle + central angle
 * See Regan 6.7 "Deployment Attitudes"
 */
static inline void set_entry_angle(state *true_state, state *est_state,
                                   runparams *run_params, vehicle *vehicle,
                                   grav *grav) {
  cartvec aimpoint = {run_params->x_aim, run_params->y_aim, run_params->z_aim};
  double est_burnout_angle =
      flight_path_angle(est_state->position, est_state->velocity);

  double current_est_speed = norm(est_state->velocity);
  double current_r = norm(est_state->position);
  double reentry_r = 100e3 + grav->earth_radius;
  state reentry_est_state = (state){0};
  reentry_est_state.position =
      (cartvec){reentry_r, 0, 0}; // actual position does not matter for the
                                  // gravity estimation, only the magnitude

  double current_kinetic =
      0.5 * vehicle->rv.rv_mass * current_est_speed * current_est_speed;
  double current_potential = current_r * norm(get_gravity_acc(grav, est_state));
  double reentry_potential =
      reentry_r * norm(get_gravity_acc(grav, &reentry_est_state));

  double reentry_est_speed =
      sqrt((current_kinetic + current_potential - reentry_potential) * 2 /
           vehicle->rv.rv_mass);
  double reentry_angle =
      acos(current_r * current_est_speed * cos(est_burnout_angle) /
           (reentry_r * reentry_est_speed));
  double entry_angle = -reentry_angle;
  double central_angle = acos(dot(aimpoint, est_state->position) /
                              (norm(aimpoint) * norm(est_state->position)));
  double rot_angle = est_burnout_angle - (entry_angle - central_angle);

  cartvec rot_axis_E = cross(est_state->position, aimpoint);
  cartvec hat_rot_axis_E = sdivide(rot_axis_E, norm(rot_axis_E));

  cartvec est_goal_entry_vector =
      rotate(est_state->velocity, hat_rot_axis_E, rot_angle);
  est_goal_entry_vector =
      sdivide(est_goal_entry_vector, norm(est_goal_entry_vector));

  cartvec true_goal_entry_vector =
      rotate(true_state->velocity, hat_rot_axis_E, rot_angle);
  true_goal_entry_vector =
      sdivide(true_goal_entry_vector, norm(true_goal_entry_vector));

  true_state->q_EB = align_roll_axis_with_vector(true_goal_entry_vector);
  est_state->q_EB = align_roll_axis_with_vector(est_goal_entry_vector);
}

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
  impact_state.delta_1 =
      state_0->delta_1 + interp_factor * (state_1->delta_1 - state_0->delta_1);
  impact_state.delta_2 =
      state_0->delta_2 + interp_factor * (state_1->delta_2 - state_0->delta_2);

  return impact_state;
}

/**
 * In an ECI frame, assuming boost and reentry guidance account for Earth
 * rotation, the discrepancy between true and estimated trajectories due to
 * Coriolis appears through the difference between true and estimated impact
 * times. For random aimpoints with fixed great-circle range, rotation speed is
 * adjusted from equatorial surface speed to local latitude:
 * $$
 * \begin{align}
 * v_\text{rot} = 464 \cos(\text{lat}).
 * \end{align}
 * $$
 * With interpolated impact-time error
 * $$
 * \begin{align}
 * \Delta t = t_\text{est} - t_\text{true},
 * \end{align}
 * $$
 * the Coriolis offset is
 * $$
 * \begin{align}
 * c = v_\text{rot} \Delta t.
 * \end{align}
 * $$
 * In the local east direction, Cartesian offsets are
 * $$
 * \begin{align}
 * \Delta x &= -c \sin(\text{lon}) \\
 * \Delta y &= c \cos(\text{lon}).
 * \end{align}
 * $$
 * The corrected impact position is $(x + \Delta x, y + \Delta y, z)$.
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
  double time_error = est_final_t - *true_final_t;
  double rot_speed = 464 * cos(lat);
  double coriolis = rot_speed * time_error;

  // based on the coriolis effect, update the final state x and y
  // This might seem like a bug, but I promise it's just clever
  // This replicates flying in a random direction, not just along the
  // equator
  true_final_state.position.x -= coriolis * sin(lon);
  true_final_state.position.y += coriolis * cos(lon);
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
 * @param burnout_vel_mag Output burnout speed in m/s
 * @param burnout_alt Output burnout altitude in m
 * @param burnout_ang Output burnout flight-path angle in radians
 * @param apogee_alt Output maximum altitude (apogee) in m
 * @param reentry_vel Output speed at 120 km reentry crossing in m/s
 * @param reentry_ang Output flight-path angle at reentry crossing in radians
 * @return Final impact state
 */
state fly(runparams *run_params, state *initial_state, vehicle *vehicle,
          double *impact_time, double *burnout_vel_mag, double *burnout_alt,
          double *burnout_ang, double *apogee_alt, double *reentry_vel,
          double *reentry_ang) {

  int max_steps = 100000000;

  // Initialize time step to the "inside atmosphere" time step
  double time_step = run_params->time_step_atm;

  // Init structs
  grav true_grav = init_grav(run_params);
  grav est_grav = init_grav(run_params);
  atm_model exp_atm_model = init_exp_atm(run_params);
  imu imu = imu_init(run_params, initial_state);
  gnss gnss = gnss_init(run_params);

  // Initialize either a randomly chosen EarthGRAM profile or the average
  // EarthGRAM profile
  eg16_profile atm_profile;
  if (run_params->atm_model == 2) {
    int atm_profile_num = (int)ran_flat(0, 100);
    atm_profile = parse_atm(run_params->atm_path, atm_profile_num);
  } else if (run_params->atm_model == 3) {
    atm_profile = parse_atm(run_params->mean_atm_path, -1);
  }

  // Initalize true and estimated states
  state true_state = *initial_state;
  state est_state = init_est_state(run_params);
  double true_t = 0;
  double est_t = 0;

  // Initialize holders for prev state for impact interpolation
  state prev_true_state = true_state;
  state prev_est_state = est_state;
  double prev_true_t = true_t;
  double prev_est_t = est_t;

  // Initialize run logging files.
  if (run_params->traj_output == 1) {
    init_run_logging(run_params->trajectory_path);
    double initial_altitude = get_altitude(true_state.position);
    atm_cond initial_true_atm_cond = get_atm_cond(
        initial_altitude, &exp_atm_model, run_params, &atm_profile);
    // Write the initial state to the trajectory file
    write_trajectory_log_row(true_t, get_vehicle_mass(vehicle, true_t),
                             &true_state, &est_state, &initial_true_atm_cond);
  }

  int sampled_new_profile = 0; // flag to indicate whether a new profile has
                               // been sampled after boost phase

  // Initialize flight event tracking
  double max_altitude = 0.0;
  int exit_atmosphere_captured = 0;
  int burnout_captured = 0;
  int reentry_captured = 0;
  *burnout_vel_mag = 0.0;
  *burnout_alt = 0.0;
  *burnout_ang = 0.0;
  *apogee_alt = 0.0;
  *reentry_vel = 0.0;
  *reentry_ang = 0.0;

  // Begin the integration loop
  for (int i = 0; i < max_steps; i++) {
    // Track apogee
    double altitude = get_altitude(true_state.position);
    if (altitude > max_altitude) {
      max_altitude = altitude;
    }

    // Check exit atmosphere event
    if (!exit_atmosphere_captured && altitude > 100e3) {
      exit_atmosphere_captured = 1;
      time_step = run_params->time_step_lambert;
    }

    // Check burnout event
    if (!burnout_captured && true_t > vehicle->booster.total_burn_time) {
      burnout_captured = 1;
      time_step = run_params->time_step_midcourse;

      *burnout_vel_mag = norm(true_state.velocity);
      *burnout_alt = altitude;
      *burnout_ang =
          flight_path_angle(true_state.position, true_state.velocity);

      // At the end of boost phase, sample a new atm profile for EarthGram
      // so boost and reentry don't use the same profile
      if ((run_params->atm_model == 2) &&
          (true_t > vehicle->booster.total_burn_time) &&
          (sampled_new_profile == 0)) {
        int atm_profile_num = (int)ran_flat(0, 100);
        atm_profile = parse_atm("input/atmprofiles.txt", atm_profile_num);
        sampled_new_profile = 1;
      }
      // Post-boost attitude maneuver
      set_entry_angle(&true_state, &est_state, run_params, vehicle, &est_grav);
      // Assume the post-boost attitude-setting maneuver results in zero angular
      // velocity
      true_state.angular_vel_B = zeros();
      est_state.angular_vel_B = zeros();
    }

    // Check reentry event. Use altitude of 120km rather than 100km so the
    // smaller time step captures all atmospheric events below 100km
    int descending = altitude < max_altitude;
    if (!reentry_captured && descending && altitude < 1.2e5 &&
        true_t > run_params->t_vert_boost) {
      reentry_captured = 1;
      time_step = run_params->time_step_atm;
      *reentry_vel = norm(true_state.velocity);
      *reentry_ang =
          flight_path_angle(true_state.position, true_state.velocity);
    }

    // Check impact event
    if (get_altitude(true_state.position) < 0 &&
        true_t > run_params->t_vert_boost) {
      double true_final_t;
      state est_final_state;
      state true_final_state = impact_with_coriolis(
          &prev_true_state, &true_state, prev_true_t, true_t, &prev_est_state,
          &est_state, prev_est_t, est_t, run_params, &true_final_t,
          &est_final_state);
      if (run_params->traj_output == 1) {
        // Write the final state to the trajectory file
        double final_altitude = get_altitude(true_final_state.position);
        atm_cond true_final_atm_cond = get_atm_cond(
            final_altitude, &exp_atm_model, run_params, &atm_profile);
        write_trajectory_log_row(
            true_final_t, get_vehicle_mass(vehicle, true_final_t),
            &true_final_state, &est_final_state, &true_final_atm_cond);
        close_run_logging();
      }

      *impact_time = true_final_t;
      *apogee_alt = max_altitude;

      // Only save full trajectory on the first run.
      run_params->traj_output = 0;

      return true_final_state;
    }

    // Get the atmospheric conditions
    atm_cond true_atm_cond =
        get_atm_cond(altitude, &exp_atm_model, run_params, &atm_profile);
    atm_cond est_atm_cond = get_exp_atm_cond(altitude, &exp_atm_model);

    // Write trajectory data to file
    if (run_params->traj_output == 1) {
      write_trajectory_log_row(true_t, get_vehicle_mass(vehicle, true_t),
                               &true_state, &est_state, &true_atm_cond);
    }

    // GNSS Measurement
    gnss.time_since_last_update += time_step;
    if ((run_params->gnss_nav == 1) &&
        (get_altitude(true_state.position) > 100e3) &&
        (gnss.time_since_last_update >= 1.0 / run_params->gnss_freq)) {
      gnss_measurement(&gnss, &true_state, &est_state);
      gnss.time_since_last_update = 0.0;
    }

    // Convert resolution from degrees to radians
    double resolution = run_params->actuator_resolution * M_PI / 180;
    double max_extent = run_params->max_deflection_angle * M_PI / 180;
    double clipped_delta_1 =
        clip(fmod(true_state.delta_1, 2 * M_PI), -max_extent, max_extent);
    double clipped_delta_2 =
        clip(fmod(true_state.delta_2, 2 * M_PI), -max_extent, max_extent);

    true_state.delta_1 = round(clipped_delta_1 / resolution) * resolution;
    true_state.delta_2 = round(clipped_delta_2 / resolution) * resolution;
    est_state.delta_1 = true_state.delta_1;
    est_state.delta_2 = true_state.delta_2;

    // Perform an integration step
    prev_true_state = true_state;
    prev_est_state = est_state;
    prev_true_t = true_t;
    prev_est_t = est_t;
    int success;
    if (run_params->integrator == 0) {
      success = euler_maruyama_step(run_params, &imu, vehicle, &true_grav,
                                    &est_grav, &true_atm_cond, &est_atm_cond,
                                    &true_state, &est_state, &true_t, &est_t,
                                    time_step, drift, diffusion);
    } else {
      success =
          sra3_step(run_params, &imu, vehicle, &true_grav, &est_grav,
                    &true_atm_cond, &est_atm_cond, &true_state, &est_state,
                    &true_t, &est_t, time_step, drift, diffusion);
    }

    if (!success) {
      printf("WARNING, integrator error, returning early\n");
      if (run_params->traj_output == 1) {
        close_run_logging();
      }
      return true_state;
    }
  }

  printf("Warning: Maximum number of steps reached with no impact\n");
  // Close the trajectory file
  if (run_params->traj_output == 1) {
    close_run_logging();
  }

  // Only save full trajectory on the first run.
  run_params->traj_output = 0;
  *apogee_alt = max_altitude;
  *impact_time = true_t;
  return true_state;
}

/**
 * Run a Monte Carlo trajectory simulation.
 *
 * @param run_params Run configuration parameters
 * @return Impact states for a single run
 */
impact_data mc_run(runparams run_params) {
  uint64_t seed;
  if (run_params.random_seed >= 0) {
    seed = (uint64_t)run_params.random_seed;
  } else {
    seed =
        (uint64_t)time(NULL) ^ ((uint64_t)clock() << 32) ^ (uint64_t)getpid();
  }
  init_genrand64(seed);
  reset_ran_gaussian();

  // Initialize the variables
  int num_runs = run_params.num_runs;
  if (num_runs > MAX_RUNS) {
    printf("Error: Number of runs exceeds the maximum limit. Increase MAX_RUNS "
           "in src/include/trajectory.h and recompile. \n");
    printf("num_runs: %d, MAX_RUNS: %d\n", num_runs, MAX_RUNS);
    exit(1);
  }
  impact_data impact_data;

  vehicle vehicle;

  if (run_params.rv_type == 0) {
    vehicle = init_mmiii_ballistic(&run_params);
  } else if (run_params.rv_type == 1) {
    vehicle = init_mmiii_swerve(&run_params);
  } else {
    printf("Error: Invalid RV type\n");
    exit(1);
  }

  state initial_true_state = init_true_state(&run_params);

  impact_data.impact_states[0] = fly(
      &run_params, &initial_true_state, &vehicle, &impact_data.impact_times[0],
      &impact_data.burnout_speed[0], &impact_data.burnout_altitude[0],
      &impact_data.burnout_angle[0], &impact_data.apogee[0],
      &impact_data.reentry_speed[0], &impact_data.reentry_angle[0]);

  // Output the impact data
  return impact_data;
}

#endif