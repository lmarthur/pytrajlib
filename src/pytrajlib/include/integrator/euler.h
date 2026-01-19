/*
Implements the (semi-implicit) Euler method 

TODO write out some latex for it

TODO how to integrate quaternions
*/

#ifndef EULER_H
#define EULER_H

#include "utils.h"
#include "state.h"
#include "derivatives.h"

void euler(State state, DerivFunction drift, DerivFunction diffusion, Args args, int max_steps, double dt) {
    /*
    Euler-Maruyama method for numerical integration of stochastic differential 
    equations. This is the semi-implicit variant — assuming the drift and diffusion 
    functions use the v_{n+1} instead of v_n for calculating the position.  

    Params:
        state: state object
        drift: function for deterministic term
        diffusion: function for stochastic term
        args: additional arguments for the drift and diffusion functions
        max_steps: maximum number of steps the integrator will take
        dt: time step
    */
}

#endif
