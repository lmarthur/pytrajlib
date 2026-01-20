#ifndef DERIVATIVES_H
#define DERIVATIVES_H

#include "utils.h"
#include "models/state.h"

typedef state (*DerivFunction)(double t, state *state, integrator_args *args);


// TODO ballistic derivative

// TODO maneuverable derivative

#endif