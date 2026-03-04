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

} impact_data;

/**
 * Interpolate between two states to estimate impact crossing at altitude 0.
 *
 * @param state_0 Pointer to pre-impact state
 * @param state_1 Pointer to post-impact state
 * @return Interpolated impact state
 */
state impact_linterp(state *state_0, state *state_1) {

  // Calculate the interpolation factor
  double altitude_0 = norm(state_0->position) - EARTH_RADIUS_M;
  double altitude_1 = norm(state_1->position) - EARTH_RADIUS_M;
  double interp_factor = altitude_0 / (altitude_0 - altitude_1);

  // Perform the interpolation
  state impact_state = *state_0;
  impact_state.t = state_0->t + interp_factor * (state_1->t - state_0->t);
  impact_state.position = add(
      state_0->position,
      smultiply(subtract(state_1->position, state_0->position), interp_factor));
  impact_state.velocity = add(
      state_0->velocity,
      smultiply(subtract(state_1->velocity, state_0->velocity), interp_factor));
  impact_state.a_drag =
      add(state_0->a_drag,
          smultiply(subtract(state_1->a_drag, state_0->a_drag), interp_factor));
  impact_state.a_lift =
      add(state_0->a_lift,
          smultiply(subtract(state_1->a_lift, state_0->a_lift), interp_factor));
  impact_state.a_thrust = add(
      state_0->a_thrust,
      smultiply(subtract(state_1->a_thrust, state_0->a_thrust), interp_factor));
  impact_state.a_total = add(
      state_0->a_total,
      smultiply(subtract(state_1->a_total, state_0->a_total), interp_factor));

  return impact_state;
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
            impact_data->impact_states[i].t,
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
state fly(runparams *run_params, state *initial_state, vehicle *vehicle) {

  // Initialize the variables and structures
  int max_steps = 10000000;

  grav true_grav = init_grav(run_params);
  grav est_grav = init_grav(run_params);

  atm_model exp_atm_model = init_exp_atm(run_params);

  double a_command_total = 0;
  double a_lift_total = 0;

  // Initialize either a randomly chosen EarthGRAM profile or the average
  // EarthGRAM profile
  eg16_profile atm_profile;
  if (run_params->atm_model == 2) {
    int atm_profile_num = (int)ran_flat(0, 100);
    atm_profile = parse_atm(run_params->atm_path, atm_profile_num);
  } else if (run_params->atm_model == 3) {
    atm_profile = parse_atm(run_params->mean_atm_path, -1);
  }

  state old_true_state = *initial_state;
  state new_true_state = *initial_state;

  state old_est_state = init_est_state(run_params);
  state new_est_state = init_est_state(run_params);

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
        "ax_drag, ay_drag, az_drag, a_command, a_lift, ax_thrust, ay_thrust, "
        "az_thrust, ax_total, ay_total, az_total, est_x, est_y, est_z, est_vx, "
        "est_vy, est_vz, est_ax_total, est_ay_total, est_az_total, "
        "est_ax_drag, est_ay_drag, est_az_drag, true_a_lift_x, true_a_lift_y, "
        "true_a_lift_z, est_a_lift_x, est_a_lift_y, est_a_lift_z, roll \n");
    // Write the initial state to the trajectory file
    fprintf(
        traj_file,
        "%g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, "
        "%g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, "
        "%g, %g, %g, %g\n",
        old_true_state.t, vehicle->current_mass, old_true_state.position.x,
        old_true_state.position.y, old_true_state.position.z,
        old_true_state.velocity.x, old_true_state.velocity.y,
        old_true_state.velocity.z, old_true_state.a_drag.x,
        old_true_state.a_drag.y, old_true_state.a_drag.z, a_command_total,
        a_lift_total, old_true_state.a_thrust.x, old_true_state.a_thrust.y,
        old_true_state.a_thrust.z, old_true_state.a_total.x,
        old_true_state.a_total.y, old_true_state.a_total.z,
        old_est_state.position.x, old_est_state.position.y,
        old_est_state.position.z, old_est_state.velocity.x,
        old_est_state.velocity.y, old_est_state.velocity.z,
        old_est_state.a_total.x, old_est_state.a_total.y,
        old_est_state.a_total.z, old_est_state.a_drag.x, old_est_state.a_drag.y,
        old_est_state.a_drag.z, old_true_state.a_lift.x,
        old_true_state.a_lift.y, old_true_state.a_lift.z,
        old_est_state.a_lift.x, old_est_state.a_lift.y, old_est_state.a_lift.z,
        old_true_state.roll);
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
        (old_true_state.t > vehicle->booster.total_burn_time) &&
        (sampled_new_profile == 0)) {
      int atm_profile_num = (int)ran_flat(0, 100);
      atm_profile = parse_atm("input/atmprofiles.txt", atm_profile_num);
      sampled_new_profile = 1;
    }

    // Get the atmospheric conditions
    double old_altitude = get_altitude(old_true_state.position);

    atm_cond true_atm_cond =
        get_atm_cond(old_altitude, &exp_atm_model, run_params, &atm_profile);
    // printf("true_atm_cond: %f, %f, %f\n", true_atm_cond.density,
    // true_atm_cond.meridional_wind, true_atm_cond.zonal_wind);
    atm_cond est_atm_cond = get_exp_atm_cond(old_altitude, &exp_atm_model);
    // if during boost or outside atmosphere, dt = main time step, else dt =
    // reentry time step go a bit above 100km to 2e5 to ensure accuracy at very
    // close to 100km
    double angle_v_grav = acos(
        dot(old_true_state.velocity, smultiply(old_true_state.position, -1)) /
        (norm(old_true_state.velocity) * norm(old_true_state.position)));

    if (((angle_v_grav > 0) && ((angle_v_grav < M_PI_2)) &&
         (old_altitude < 1.2e5)) ||
        (old_true_state.t <= vehicle->booster.total_burn_time)) {
      time_step = run_params->time_step_reentry;
    } else {
      time_step = run_params->time_step_main;
    }

    // Update the thrust of the vehicle
    if (run_params->perfect_boost) {
      update_thrust(&new_true_state, vehicle, run_params, &true_grav);
      new_est_state.a_thrust = new_true_state.a_thrust;
    } else {
      update_thrust(&new_est_state, vehicle, run_params, &est_grav);
      new_true_state.a_thrust = new_est_state.a_thrust;
    }
    if (isnan(new_true_state.a_thrust.x)) {
      return new_true_state;
    }
    // Update the gravity acceleration components
    cartvec a_grav_true = update_gravity(&true_grav, &new_true_state);
    cartvec a_grav_est = update_gravity(&est_grav, &new_est_state);

    // Update the drag acceleration components
    update_drag(run_params, vehicle, &true_atm_cond, &new_true_state,
                &step_timer);
    update_drag(run_params, vehicle, &est_atm_cond, &new_est_state,
                &step_timer);

    // If maneuverable RV, use proportional navigation during reentry
    if (run_params->rv_maneuv == 1 && (angle_v_grav > 0) &&
        (angle_v_grav < M_PI_2) && (old_altitude < 1e5)) {
      update_lift(&new_true_state, &new_est_state, run_params, &true_atm_cond,
                  &est_atm_cond, vehicle, time_step);
      a_lift_total = norm(new_true_state.a_lift);
    }

    // Calculate the total acceleration components
    new_true_state.a_total =
        add(add(a_grav_true, new_true_state.a_drag),
            add(new_true_state.a_lift, new_true_state.a_thrust));
    new_est_state.a_total =
        add(add(a_grav_est, new_est_state.a_drag),
            add(new_est_state.a_lift, new_est_state.a_thrust));

    if (run_params->ins_nav == 1) {
      // INS Measurement
      cartvec a_total_est =
          imu_measurement(&imu, &new_true_state, &new_est_state, vehicle,
                          a_grav_true, a_grav_est);
      new_est_state.a_total = a_total_est;
      update_imu(&imu, time_step);
    }

    if ((run_params->gnss_nav == 1) && (old_altitude > 100e3)) {
      // GNSS Measurement
      gnss_measurement(&gnss, &new_true_state, &new_est_state);
    }

    // Perform a Runge-Kutta step
    euler_maruyama_step(&new_true_state, time_step);
    euler_maruyama_step(&new_est_state, time_step);
    // Update the mass of the vehicle
    update_mass(vehicle, new_true_state.t);

    // Check if the vehicle has impacted the Earth
    double new_altitude = get_altitude(new_true_state.position);
    if (new_altitude < 0) {
      state true_final_state = impact_linterp(&old_true_state, &new_true_state);
      state est_final_state = impact_linterp(&old_est_state, &new_est_state);

      // Add coriolis effect based on the latitude and the impact time error
      double lat = ran_flat(-M_PI / 2, M_PI / 2);
      double lon = ran_flat(-M_PI, M_PI);
      double time_error = true_final_state.t - est_final_state.t;
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
            subtract(true_final_state.position, est_final_state.position);
      }
      if (traj_output == 1) {
        // Write the final state to the trajectory file
        fprintf(traj_file,
                "%g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, "
                "%g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, "
                "%g, %g, %g, %g, %g, %g, %g, %g\n",
                true_final_state.t, vehicle->current_mass,
                true_final_state.position.x, true_final_state.position.y,
                true_final_state.position.z, true_final_state.velocity.x,
                true_final_state.velocity.y, true_final_state.velocity.z,
                true_final_state.a_drag.x, true_final_state.a_drag.y,
                true_final_state.a_drag.z, a_command_total, a_lift_total,
                true_final_state.a_thrust.x, true_final_state.a_thrust.y,
                true_final_state.a_thrust.z, true_final_state.a_total.x,
                true_final_state.a_total.y, true_final_state.a_total.z,
                est_final_state.position.x, est_final_state.position.y,
                est_final_state.position.z, est_final_state.velocity.x,
                est_final_state.velocity.y, est_final_state.velocity.z,
                est_final_state.a_total.x, est_final_state.a_total.y,
                est_final_state.a_total.z, est_final_state.a_drag.x,
                est_final_state.a_drag.y, est_final_state.a_drag.z,
                old_true_state.a_lift.x, old_true_state.a_lift.y,
                old_true_state.a_lift.z, old_est_state.a_lift.x,
                old_est_state.a_lift.y, old_est_state.a_lift.z,
                true_final_state.roll);
        fclose(traj_file);
      }

      return true_final_state;
    }

    // output the trajectory data
    if (traj_output == 1) {
      fprintf(
          traj_file,
          "%g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, "
          "%g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, "
          "%g, %g, %g, %g\n",
          new_true_state.t, vehicle->current_mass, new_true_state.position.x,
          new_true_state.position.y, new_true_state.position.z,
          new_true_state.velocity.x, new_true_state.velocity.y,
          new_true_state.velocity.z, new_true_state.a_drag.x,
          new_true_state.a_drag.y, new_true_state.a_drag.z, a_command_total,
          a_lift_total, new_true_state.a_thrust.x, new_true_state.a_thrust.y,
          new_true_state.a_thrust.z, new_true_state.a_total.x,
          new_true_state.a_total.y, new_true_state.a_total.z,
          new_est_state.position.x, new_est_state.position.y,
          new_est_state.position.z, new_est_state.velocity.x,
          new_est_state.velocity.y, new_est_state.velocity.z,
          new_est_state.a_total.x, new_est_state.a_total.y,
          new_est_state.a_total.z, new_est_state.a_drag.x,
          new_est_state.a_drag.y, new_est_state.a_drag.z,
          old_true_state.a_lift.x, old_true_state.a_lift.y,
          old_true_state.a_lift.z, old_est_state.a_lift.x,
          old_est_state.a_lift.y, old_est_state.a_lift.z, new_true_state.roll);
    }

    // Update the old state
    old_true_state = new_true_state;
    old_est_state = new_est_state;
  }

  printf("Warning: Maximum number of steps reached with no impact\n");
  // Close the trajectory file
  if (traj_output == 1) {
    fclose(traj_file);
  }

  // Only save full trajectory on the first run.
  run_params->traj_output = 0;
  return new_true_state;
}

/**
 * Run a Monte Carlo trajectory simulation.
 *
 * @param run_params Run configuration parameters
 * @return Impact states for all runs
 */
impact_data mc_run(runparams run_params) {

  // Print the run parameters to the console
  // print_config(&run_params);

  // Initialize the variables
  int num_runs = run_params.num_runs;
  // printf("Simulating %d Monte Carlo runs...\n", num_runs);
  if (num_runs > MAX_RUNS) {
    printf("Error: Number of runs exceeds the maximum limit. Increase MAX_RUNS "
           "in src/include/trajectory.h and recompile. \n");
    printf("num_runs: %d, MAX_RUNS: %d\n", num_runs, MAX_RUNS);
    exit(1);
  }
  // state initial_state = init_state();
  // vehicle vehicle = init_mmiii_ballistic();
  impact_data impact_data;

  // Print an updated aimpoint
  // cart_vector aimpoint = update_aimpoint(run_params, 0.785398163397);
  // printf("Updated aimpoint: %f, %f, %f\n", aimpoint.x, aimpoint.y,
  // aimpoint.z);

  // Create a .txt file to store the impact data
  // impact_file = fopen(run_params.impact_data_path, "w");
  // fprintf(impact_file, "t, x, y, z, vx, vy, vz\n");

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

    impact_data.impact_states[i] =
        fly(&run_params, &initial_true_state, &vehicle);

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