#ifndef GUIDANCE_H
#define GUIDANCE_H

#include "trajectory.h"
#include "utils.h"

cart_vector prop_nav(runparams *run_params, state *estimated_state){
    /*
    Proportional navigation guidance law

    INPUTS:
    ----------
        run_params: runparams *
            pointer to the run parameters struct
        estimated_state: state *
            pointer to the estimated state of the vehicle

    */

    // Calculate the relative position vector to the target
    cart_vector r_target;

    r_target.x = run_params->x_aim - estimated_state->x;
    r_target.y = run_params->y_aim - estimated_state->y;
    r_target.z = run_params->z_aim - estimated_state->z;

    // Calculate the relative velocity vector to the target (currently configured for a stationary target)
    cart_vector v_rel;
    v_rel.x = 0 - estimated_state->vx;
    v_rel.y = 0 - estimated_state->vy;
    v_rel.z = 0 - estimated_state->vz;

    // get the rotation vector by taking the cross product of the relative position and velocity vectors and dividing by |r|**2
    cart_vector rot;
    double r_dot_r = r_target.x*r_target.x + r_target.y*r_target.y + r_target.z*r_target.z;
    rot.x = (r_target.y*v_rel.z - r_target.z*v_rel.y) / r_dot_r;
    rot.y = (r_target.z*v_rel.x - r_target.x*v_rel.z) / r_dot_r;
    rot.z = (r_target.x*v_rel.y - r_target.y*v_rel.x) / r_dot_r;

    // Calculate the acceleration command by taking the cross product of the relative velocity and the rotation vector and scaling by the navigation gain
    cart_vector a_command;
    a_command.x = run_params->nav_gain * (v_rel.y*rot.z - v_rel.z*rot.y);
    a_command.y = run_params->nav_gain * (v_rel.z*rot.x - v_rel.x*rot.z);
    a_command.z = run_params->nav_gain * (v_rel.x*rot.y - v_rel.y*rot.x);

    return a_command;
}


#endif
