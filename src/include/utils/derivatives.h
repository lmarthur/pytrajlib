#ifndef DERIVATIVES_H
#define DERIVATIVES_H

#include "models/state.h"
#include "utils.h"

typedef multistate (*DerivFunction)(double t, multistate *state,
                                    integrator_args *args);

// TODO ballistic derivative

// TODO maneuverable derivative

#endif