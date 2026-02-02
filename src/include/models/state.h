#ifndef STATE_H
#define STATE_H

#include "math/linalg.h"
#include "utils/constants.h"
#include "utils/run_params.h"

struct {
  cartvec position;
  cartvec velocity;
  cartvec a_lift;
  cartvec a_lift_avail;
  anglevec gyro_error;
  quaternion quaternion;
} typedef state;

struct {
  state true_state;
  state est_state;
} typedef dualstate;

struct {
  state true_state;
  state est_state; // estimated state
  state des_state; // desired state
} typedef multistate;

state init_state(runparams run_params) {
  state current_state;

  current_state.position.x = R_EARTH;
  current_state.position.y = 0.0;
  current_state.position.z = 0.0;

  current_state.velocity = zeros();
  current_state.a_lift = zeros();
  current_state.a_lift_avail = zeros();
  current_state.gyro_error.lat = 0.0;
  current_state.gyro_error.lon = 0.0;
  current_state.quaternion.w = 1.0;
  current_state.quaternion.x = 0.0;
  current_state.quaternion.y = 0.0;
  current_state.quaternion.z = 0.0;

  return current_state;
}

#endif