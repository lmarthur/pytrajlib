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

    multistate d_multi_dt = {0};

    // Setup state pointers for true, estimated, and desired states
    state *states[] = {&current_state->true_state, &current_state->est_state,
                       &current_state->des_state};
    state *deriv_states[] = {&d_multi_dt.true_state, &d_multi_dt.est_state,
                             &d_multi_dt.des_state};

    // Loop over true, estimated, and desired states (if applicable)
    int num_states = dual_args->update_desired_state ? 3 : 2;
    for (int i = 0; i < num_states; i++) {
        // Get the total acceleration (thrust + gravity + drag + lift)
        cartvec a_thrust = get_thrust_acceleration(t, *states[i], true_args);
        cartvec a_grav =
            get_gravitational_acceleration(t, *states[i], true_args);
        cartvec a_drag = get_drag_acceleration(t, *states[i], true_args);

        cartvec a_total = add(a_thrust, add(a_grav, a_drag));

        // Set derivatives. Derivative of position is current velocity.
        // Derivative of velocity is new acceleration.
        deriv_states[i]->position = states[i]->velocity;
        deriv_states[i]->velocity = a_total;
    }

    return d_multi_dt;
}
// TODO maneuverable derivative

#endif