#ifndef DERIVATIVES_H
#define DERIVATIVES_H

#include "forces/drag.h"
#include "forces/gravity.h"
#include "forces/thrust.h"
#include "models/state.h"
#include "utils.h"

typedef multistate (*DerivFunction)(double t, multistate *state,
                                    dualargs *args);

multistate dummy_deriv(double t, multistate *current_state,
                       dualargs *dual_args) {
    multistate dummy = {0};
    return dummy;
}

multistate ballistic_derivs(double t, multistate *current_state,
                            dualargs *dual_args) {
    integrator_args true_args = get_true_args(*dual_args);

    // Get the total acceleration (thrust + gravity + drag + lift)
    cartvec a_thrust =
        get_thrust_acceleration(t, current_state->true_state, true_args);
    cartvec a_grav =
        get_gravitational_acceleration(t, current_state->true_state, true_args);
    cartvec a_drag =
        get_drag_acceleration(t, current_state->true_state, true_args);

    cartvec a_total = add(a_thrust, add(a_grav, a_drag));

    // Set derivatives. Derivative of position is current velocity. Derivative
    // of velocity is new acceleration.
    state d_true_state_dt;
    d_true_state_dt.position = current_state->true_state.velocity;
    d_true_state_dt.velocity = a_total;

    multistate d_multi_dt = {0};
    d_multi_dt.true_state = d_true_state_dt;

    return d_multi_dt;
}
// TODO maneuverable derivative

#endif