#ifndef UTILS_H
#define UTILS_H

#include "integrator/args.h"
#include "models/atmosphere.h"
#include "models/state.h"
#include "utils/constants.h"

double get_altitude(state current_state) {
  return norm(current_state.position) - R_EARTH;
}

/**
 * Get the wind speed in Cartesian coordinates at the current state's position
 *
 * @param atm_cond atmosphere conditions
 * @param current_state state struct containing position of interest
 * @return cartesian wind vector
 */
cartvec get_cart_wind(state current_state, integrator_args args) {
  atm_cond cond = get_atm_cond(get_altitude(current_state), &args.atm,
                               &args.run_params, &args.atm_profile);
  spherevec sphere_wind;
  sphere_wind.r = cond.vertical_wind;
  sphere_wind.lat = cond.meridional_wind;
  sphere_wind.lon = cond.zonal_wind;

  spherevec sphere_position = cartcoords_to_sphercoords(current_state.position);

  cartvec wind_vec = spherevec_to_cartvec(sphere_wind, sphere_position);
  return wind_vec;
}

/**
 * Get atmospheric density (kg/m^3) at altitude given by current state
 */
double get_atm_density(state current_state, integrator_args args) {
  atm_cond cond = get_atm_cond(get_altitude(current_state), &args.atm,
                               &args.run_params, &args.atm_profile);
  return cond.density;
}

double get_max_a_exec(runparams run_params, vehicle veh) {
  double max_flap_force =
      run_params.actuator_force * run_params.gearing_ratio * 1000;
  double max_lift_force =
      (veh.rv.c_l_alpha * max_flap_force * (veh.rv.x_flap - veh.rv.x_com) /
       (veh.rv.c_m_alpha *
        veh.rv.rv_length)); // maximum lift force in N, based on moment arm and
                            // lift properties
  double max_a_exec =
      (max_lift_force / veh.rv.rv_mass); // maximum acceleration that can be
                                         // executed by the flaps in m/s^2
  return max_a_exec;
}

#endif