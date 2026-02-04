#ifndef THRUST_H
#define THRUST_H

#include "integrator/args.h"
#include "math/linalg.h"
#include "models/imu.h"
#include "models/state.h"

/**
 * Thrust is vertical for the first 5 seconds. After 5s and before burnout, the
 * thrust is along the direction specified by the thrust angles
 * @param t current flight time (seconds)
 * @param current_state True or estimated state. See State in the API
 * documentation.
 * @param args True or estimated DerivArgs. See DerivArgs in the API
 * documentation.
 *
 * @return Thrust acceleration in Cartesian coordinates x, y, z (m/s^2)
 */
cartvec get_thrust_acceleration(double t, state current_state,
                                integrator_args args) {
    double mass = get_mass(t, args.vehicle);
    int active_stage = get_current_stage(args.vehicle.booster, t);

    // Calculate magnitude of thrust acceleration (set to 0 after burnout)
    double a_thrust_mag;
    if (t > args.vehicle.booster.total_burn_time) {
        a_thrust_mag = 0;
    } else {
        a_thrust_mag = args.vehicle.booster.isp0[active_stage] *
                       args.vehicle.booster.fuel_burn_rate[active_stage] / mass;
    }

    cartvec a_thrust;

    // Vertical thrust for the first 5 seconds
    if (t < 5) {
        a_thrust.x = a_thrust_mag;
        a_thrust.y = 0;
        a_thrust.z = 0;
        return a_thrust;
    }

    // Thrust along angles given by gyroscope measurement
    anglevec thrust_angles =
        gyro_measurement(current_state, args.vehicle, args.run_params);

    spherevec thrust_vec;
    thrust_vec.r = a_thrust_mag;
    thrust_vec.lat = thrust_angles.lat;
    thrust_vec.lon = thrust_angles.lon;

    a_thrust = sphercoords_to_cartcoords(thrust_vec);
    return a_thrust;
}

#endif