#ifndef STATE_H
#define STATE_H

#include "math/linalg.h"

struct {
    cartvec position;
    cartvec velocity;
    cartvec a_lift;
    cartvec a_lift_avail;
    anglevec gyro_error;
    quaternion quaternion;
} typedef state;

state init_true_state(runparams *run_params){
    state true_state;
    true_state.gyro_error.lat = run_params->init_thrust_lat_pert;
    true_state.gyro_error.lon = run_params->init_thrust_lon_pert;
}



#endif