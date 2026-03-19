#ifndef THRUST_H
#define THRUST_H

#include <math.h>

#include "../models/atmosphere.h"
#include "../models/grav.h"
#include "../models/vehicle.h"
#include "../utils/utils.h"

int get_current_stage(double t, vehicle *vehicle) {
  // Get the current stage
  int stage = 0;
  if (t > vehicle->booster.burn_time[0]) {
    stage = 1;
  }
  if (t > vehicle->booster.burn_time[0] + vehicle->booster.burn_time[1]) {
    stage = 2;
  }
  return stage;
}

/**
 * Calculate remaining delta-v by summing the delta-v of each stage
 */
double remaining_delta_v(state *state, vehicle *vehicle, double t) {
  double burn_time_in_stage;
  // Burn time in third stage
  if (t > vehicle->booster.burn_time[0] + vehicle->booster.burn_time[1]) {
    burn_time_in_stage =
        t - (vehicle->booster.burn_time[0] + vehicle->booster.burn_time[1]);
  }
  // Burn time in second stage
  else if (t > vehicle->booster.burn_time[0]) {
    burn_time_in_stage = t - vehicle->booster.burn_time[0];
  }
  // Burn time in first stage
  else {
    burn_time_in_stage = t;
  }

  int current_stage = get_current_stage(t, vehicle);
  double mass_start_of_stage = get_vehicle_mass(vehicle, t);
  double burned_mass =
      vehicle->booster.fuel_burn_rate[current_stage] * burn_time_in_stage;
  double remaining_fuel_mass =
      vehicle->booster.fuel_mass[current_stage] - burned_mass;
  // At the end of the current stage, the mass will be the current mass -
  // remaining fuel mass
  double final_mass_current_stage = mass_start_of_stage - remaining_fuel_mass;

  double delta_v = vehicle->booster.isp0[current_stage] *
                   log(mass_start_of_stage / final_mass_current_stage);

  // Remaining delta-v is the sum of each stage's delta v
  for (int i = current_stage + 1; i < vehicle->booster.num_stages; i++) {
    // Each unburned stage's mass is the total mass - sum previous stage's wet
    // mass
    double stage_initial_mass = vehicle->total_mass;
    for (int j = 0; j < i; j++) {
      stage_initial_mass -= vehicle->booster.wet_mass[j];
    }
    // At the end of the current stage, the mass will be the current mass -
    // remaining fuel mass
    double stage_final_mass =
        stage_initial_mass - vehicle->booster.fuel_mass[i];

    delta_v +=
        vehicle->booster.isp0[i] * log(stage_initial_mass / stage_final_mass);
  }
  return delta_v;
}

/**
 * Get the central angle in radians between current position and aimpoint
 */
double get_central_angle(cartvec position, cartvec aimpoint) {
  double phi =
      acos(dot(position, aimpoint) / (norm(position) * norm(aimpoint)));
  return phi;
}

double get_lambert_velocity(double r0, double rf, double phi, double gamma,
                            grav *grav_model) {
  double GM = grav_model->grav_const * grav_model->earth_mass;

  double v =
      sqrt((GM * (1 - cos(phi))) /
           (r0 * cos(gamma) * (r0 * cos(gamma) / rf - cos(phi + gamma))));
  return v;
}

double time_to_fly(double r0, double phi, double gamma, double lambert_velocity,
                   grav *grav_model) {
  double GM = grav_model->grav_const * grav_model->earth_mass;
  double lambda = r0 * lambert_velocity * lambert_velocity / GM;

  // Only valid for elliptical travel
  if (lambda >= 2) {
    printf("WARNING lambda=%f > 2\n", lambda);
    return NAN;
  }
  double t =
      r0 / (lambert_velocity * cos(gamma)) *
      (((tan(gamma) * (1 - cos(phi)) + (1 - lambda) * sin(phi)) /
        ((2 - lambda) * ((1 - cos(phi)) / (lambda * cos(gamma) * cos(gamma)) +
                         cos(gamma + phi) / cos(gamma)))) +
       (2 * cos(gamma) / (lambda * pow(2 / lambda - 1, 1.5)) *
        atan(sqrt(2 / lambda - 1) /
             (cos(gamma) * cos(phi / 2) / sin(phi / 2) - sin(gamma)))));
  return t;
}

double get_min_flight_angle(double r0, double rf, double phi) {
  double gamma_min =
      atan((sin(phi) - sqrt(2 * r0 / rf * (1 - cos(phi)))) / (1 - cos(phi)));
  return gamma_min;
}

double get_max_flight_angle(double r0, double rf, double phi) {
  double gamma_max =
      atan((sin(phi) + sqrt(2 * r0 / rf * (1 - cos(phi)))) / (1 - cos(phi)));
  return gamma_max;
}

/**
 * Find the flight angle (gamma) that results in the estimated flight time being
 * within tol (1e-8) of the desired flight time. Uses the secant method to find
 * the root.
 */
double get_flight_angle(double r0, double rf, double phi, double t_f_des,
                        grav *grav_model) {
  double tol = 1e-8;
  double gmin = get_min_flight_angle(r0, rf, phi);
  double gmax = get_max_flight_angle(r0, rf, phi);
  double gamma_current = (gmin + gmax) / 2.0;

  double v_lambert =
      get_lambert_velocity(r0, rf, phi, gamma_current, grav_model);
  double t_f_current =
      time_to_fly(r0, phi, gamma_current, v_lambert, grav_model);

  double gamma_prev = gamma_current;
  double t_f_prev = t_f_current;
  int icount = 0;

  while ((fabs(t_f_current - t_f_des) > tol * t_f_des) && icount < 1000) {
    // Update bisection bounds
    if (t_f_current > t_f_des) {
      gmax = gamma_current;
    } else {
      gmin = gamma_current;
    }

    // Calculate next gamma
    double gamma_next;
    if (icount == 0) {
      gamma_next = (gmax + gmin) / 2.0;
    } else {
      if (fabs(t_f_current - t_f_prev) < 1e-15) {
        gamma_next = (gmax + gmin) / 2.0;
      } else {
        gamma_next = gamma_current + (gamma_current - gamma_prev) *
                                         (t_f_des - t_f_current) /
                                         (t_f_current - t_f_prev);
        // Constrain to valid bounds
        if (gamma_next > gmax || gamma_next < gmin) {
          gamma_next = (gmax + gmin) / 2.0;
        }
      }
    }

    gamma_prev = gamma_current;
    t_f_prev = t_f_current;
    gamma_current = gamma_next;
    icount++;

    v_lambert = get_lambert_velocity(r0, rf, phi, gamma_current, grav_model);
    t_f_current = time_to_fly(r0, phi, gamma_current, v_lambert, grav_model);
  }
  if (icount == 1000) {
    printf(
        "WARNING: get_flight_angle did not converge after 1000 iterations\n");
    return NAN;
  }
  return gamma_current;
}

/**
 * See Zarchan (2016) Listing 28.2
 */
cartvec get_lambert_velocity_vector(cartvec position, cartvec aimpoint,
                                    double tf_des, grav *grav_model) {
  double r0 = norm(position);
  double rf = norm(aimpoint);
  double phi = get_central_angle(position, aimpoint);
  double gamma = get_flight_angle(r0, rf, phi, tf_des, grav_model);
  double v = get_lambert_velocity(r0, rf, phi, gamma, grav_model);
  cartvec lambert_velocity;

  double mag_pos = norm(position);
  double c2 = mag_pos * sin(M_PI_2 - gamma) / norm(cross(position, aimpoint));
  double c1 = cos(M_PI_2 - gamma) / mag_pos -
              dot(position, aimpoint) * c2 / (mag_pos * mag_pos);

  lambert_velocity = add(smultiply(position, c1), smultiply(aimpoint, c2));
  lambert_velocity = smultiply(lambert_velocity, v);
  return lambert_velocity;
}

/**
 * Generalized Energy Management
 *
 * See Zarchan (2016) Ch 13
 */
double thrust_offset(state *state, vehicle *vehicle, cartvec v_to_gain,
                     double t) {
  double dv = remaining_delta_v(state, vehicle, t);
  double vtg = norm(v_to_gain);
  double theta = 0;
  if (dv > vtg) {
    theta = sqrt(6 * (1 - vtg / dv));
  }
  return theta;
}

double get_a_thrust_magnitude(state *state, vehicle *vehicle, double t) {
  int stage = get_current_stage(t, vehicle);

  // Calculate the thrust acceleration components
  double a_thrust_mag = vehicle->booster.isp0[stage] *
                        vehicle->booster.fuel_burn_rate[stage] /
                        get_vehicle_mass(vehicle, t);
  return a_thrust_mag;
}

cartvec get_thrust_vector(state *state, vehicle *vehicle, runparams *run_params,
                          grav *grav_model, double t) {
  cartvec aimpoint = {run_params->x_aim, run_params->y_aim, run_params->z_aim};

  cartvec lambert_velocity = get_lambert_velocity_vector(
      state->position, aimpoint, run_params->t_des_final - t, grav_model);
  cartvec v_to_gain = subtract(lambert_velocity, state->velocity);
  // unit vector in direction of v to gain
  cartvec v_to_gain_hat = sdivide(v_to_gain, norm(v_to_gain));

  // GEM implemented by directing thrust at an angle theta from v_to_gain in the
  // transfer plane
  double theta = thrust_offset(state, vehicle, v_to_gain, t);

  // Vector orthogonal to position and aimpoint that should be rotated around
  cartvec u = cross(state->position, aimpoint);
  cartvec uhat = sdivide(u, norm(u));

  // Unit vector in direction of thrust
  cartvec thrust_hat = rotate(v_to_gain_hat, uhat, -theta);

  double a_thrust_mag = get_a_thrust_magnitude(state, vehicle, t);

  cartvec thrust = smultiply(thrust_hat, a_thrust_mag);
  return thrust;
}

/**
 * Get thrust acceleration using Lambert Guidance outside the atmosphere
 */
cartvec get_thrust_acc(state *true_state, state *est_state, vehicle *vehicle,
                       runparams *run_params, grav *true_grav, grav *est_grav,
                       double t) {
  cartvec a_thrust;
  if (t > vehicle->booster.total_burn_time) {
    return zeros();
  }

  state gyro_state;
  state *state;
  grav *grav_model;
  // To simulate perfect boost, use the estimated state for gyro error
  // calculation because the estimated gyro error is 0 and use the true state
  // for the remaining calculations because it has the true position & velocity
  if (run_params->perfect_boost) {
    gyro_state = *est_state;
    state = true_state;
    grav_model = true_grav;
  } else {
    gyro_state = *true_state;
    state = est_state;
    grav_model = est_grav;
  }

  double a_thrust_mag = get_a_thrust_magnitude(state, vehicle, t);

  // Vertical thrust for the beginning of the flight
  if (t < run_params->t_vert_boost) {
    a_thrust.x = a_thrust_mag * cos(gyro_state.gyro_error.pitch) *
                 cos(gyro_state.gyro_error.yaw);
    a_thrust.y = a_thrust_mag * cos(gyro_state.gyro_error.pitch) *
                 sin(gyro_state.gyro_error.yaw);
    a_thrust.z = a_thrust_mag * sin(gyro_state.gyro_error.pitch);
    return a_thrust;
  }

  if (get_altitude(state->position) < 100e3) {
    a_thrust.x = a_thrust_mag * cos(state->theta_long) * cos(state->theta_lat);
    a_thrust.y = a_thrust_mag * sin(state->theta_long) * cos(state->theta_lat);
    a_thrust.z = a_thrust_mag * sin(state->theta_lat);

    return a_thrust;
  }

  a_thrust = get_thrust_vector(state, vehicle, run_params, grav_model, t);
  return a_thrust;
}
#endif