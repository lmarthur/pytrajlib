#ifndef THRUST_H
#define THRUST_H

#include <math.h>

#include "../models/atmosphere.h"
#include "../models/grav.h"
#include "../models/vehicle.h"
#include "../utils.h"

int get_current_stage(state *state, vehicle *vehicle) {
  // Get the current stage
  int stage = 0;
  if (state->t > vehicle->booster.burn_time[0]) {
    stage = 1;
  }
  if (state->t >
      vehicle->booster.burn_time[0] + vehicle->booster.burn_time[1]) {
    stage = 2;
  }
  return stage;
}

/**
 * Calculate remaining delta-v by summing the delta-v of each stage
 */
double remaining_delta_v(state *state, vehicle *vehicle) {
  double burn_time_in_stage;
  // Burn time in third stage
  if (state->t >
      vehicle->booster.burn_time[0] + vehicle->booster.burn_time[1]) {
    burn_time_in_stage = state->t - (vehicle->booster.burn_time[0] +
                                     vehicle->booster.burn_time[1]);
  }
  // Burn time in second stage
  else if (state->t > vehicle->booster.burn_time[0]) {
    burn_time_in_stage = state->t - vehicle->booster.burn_time[0];
  }
  // Burn time in first stage
  else {
    burn_time_in_stage = state->t;
  }

  int current_stage = get_current_stage(state, vehicle);
  double mass_start_of_stage = vehicle->current_mass;
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
double get_central_angle(cart_vector position, cart_vector aimpoint) {
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
cart_vector get_lambert_velocity_vector(cart_vector position,
                                        cart_vector aimpoint, double tf_des,
                                        grav *grav_model) {
  double r0 = norm(position);
  double rf = norm(aimpoint);
  double phi = get_central_angle(position, aimpoint);
  double gamma = get_flight_angle(r0, rf, phi, tf_des, grav_model);
  double v = get_lambert_velocity(r0, rf, phi, gamma, grav_model);
  cart_vector lambert_velocity;

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
double thrust_offset(state *state, vehicle *vehicle, cart_vector v_to_gain) {
  double dv = remaining_delta_v(state, vehicle);
  double vtg = norm(v_to_gain);
  double theta = 0;
  if (dv > vtg) {
    theta = sqrt(6 * (1 - vtg / dv));
  }
  return theta;
}

double get_a_thrust_magnitude(state *state, vehicle *vehicle) {
  int stage = get_current_stage(state, vehicle);

  // Calculate the thrust acceleration components
  double a_thrust_mag = vehicle->booster.isp0[stage] *
                        vehicle->booster.fuel_burn_rate[stage] /
                        vehicle->current_mass;
  return a_thrust_mag;
}

cart_vector get_thrust_vector(state *state, vehicle *vehicle,
                              runparams *run_params, grav *grav_model) {
  cart_vector position = {state->x, state->y, state->z};
  cart_vector aimpoint = {run_params->x_aim, run_params->y_aim,
                          run_params->z_aim};
  cart_vector current_velocity = {state->vx, state->vy, state->vz};

  cart_vector lambert_velocity = get_lambert_velocity_vector(
      position, aimpoint, run_params->t_des_final - state->t, grav_model);
  cart_vector v_to_gain = subtract(lambert_velocity, current_velocity);
  // unit vector in direction of v to gain
  cart_vector v_to_gain_hat = sdivide(v_to_gain, norm(v_to_gain));

  // GEM implemented by directing thrust at an angle theta from v_to_gain in the
  // transfer plane
  double theta = thrust_offset(state, vehicle, v_to_gain);

  // Vector orthogonal to position and aimpoint that should be rotated around
  cart_vector u = cross(position, aimpoint);
  cart_vector uhat = sdivide(u, norm(u));

  // Unit vector in direction of thrust
  cart_vector thrust_hat = rotate(v_to_gain_hat, uhat, -theta);

  double a_thrust_mag = get_a_thrust_magnitude(state, vehicle);

  cart_vector thrust = smultiply(thrust_hat, a_thrust_mag);
  return thrust;
}

/**
 * Update thrust acceleration components using Lambert Guidance outside the
 * atmosphere
 */
void update_thrust(state *state, vehicle *vehicle, runparams *run_params,
                   grav *grav_model) {
  if (state->t > vehicle->booster.total_burn_time) {
    state->ax_thrust = 0;
    state->ay_thrust = 0;
    state->az_thrust = 0;
    return;
  }

  double a_thrust_mag = get_a_thrust_magnitude(state, vehicle);
  // Vertical thrust for the beginning of the flight
  if (state->t < run_params->t_vert_boost) {
    state->ax_thrust = a_thrust_mag;
    state->ay_thrust = 0;
    state->az_thrust = 0;
    return;
  }

  if (get_altitude(state->x, state->y, state->z) < 100e3) {
    state->ax_thrust =
        a_thrust_mag * cos(state->theta_long) * cos(state->theta_lat);
    state->ay_thrust =
        a_thrust_mag * sin(state->theta_long) * cos(state->theta_lat);
    state->az_thrust = a_thrust_mag * sin(state->theta_lat);

    return;
  }

  cart_vector a_thrust =
      get_thrust_vector(state, vehicle, run_params, grav_model);
  state->ax_thrust = a_thrust.x;
  state->ay_thrust = a_thrust.y;
  state->az_thrust = a_thrust.z;
}
#endif