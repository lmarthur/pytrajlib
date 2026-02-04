#ifndef STATE_H
#define STATE_H

#include "math/linalg.h"
#include "utils/constants.h"
#include "utils/run_params.h"

typedef struct {
    cartvec position;
    cartvec velocity;
    cartvec a_lift;
    cartvec a_lift_avail;
    anglevec gyro_error;
    quaternion quaternion;
} state;

typedef struct {
    state true_state;
    state est_state;
} dualstate;

typedef struct {
    state true_state;
    state est_state; // estimated state
    state des_state; // desired state
} multistate;

state init_true_state(runparams run_params) {
    state current_state;

    current_state.position.x = R_EARTH;
    current_state.position.y = run_params.initial_pos_error * ran_gaussian(1);
    current_state.position.z = run_params.initial_pos_error * ran_gaussian(1);

    current_state.velocity.x = ran_gaussian(run_params.initial_vel_error);
    current_state.velocity.y = ran_gaussian(run_params.initial_vel_error);
    current_state.velocity.z = ran_gaussian(run_params.initial_vel_error);

    current_state.a_lift = zeros();
    current_state.a_lift_avail = zeros();
    current_state.gyro_error.lat = 0;
    current_state.gyro_error.lon = 0;
    current_state.quaternion.w = 1.0;
    current_state.quaternion.x = 0.0;
    current_state.quaternion.y = 0.0;
    current_state.quaternion.z = 0.0;

    return current_state;
}

state init_est_state(runparams run_params) {
    state current_state;

    current_state.position.x = R_EARTH;
    current_state.position.y = 0.0;
    current_state.position.z = 0.0;

    current_state.velocity = zeros();
    current_state.a_lift = zeros();
    current_state.a_lift_avail = zeros();
    current_state.gyro_error.lat = -run_params.init_thrust_lat_pert;
    current_state.gyro_error.lon = -run_params.init_thrust_lon_pert;
    current_state.quaternion.w = 1.0;
    current_state.quaternion.x = 0.0;
    current_state.quaternion.y = 0.0;
    current_state.quaternion.z = 0.0;

    return current_state;
}

#endif