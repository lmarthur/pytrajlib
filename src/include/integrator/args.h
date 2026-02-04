#ifndef ARGS_H
#define ARGS_H

#include "models/atmosphere.h"
#include "models/grav.h"
#include "models/vehicle.h"
struct {
    atm_model atm;
    eg16_profile atm_profile;
    runparams run_params;
    vehicle vehicle;
    grav gravity;

} typedef integrator_args;

struct {
    atm_model true_atm;
    eg16_profile true_atm_profile;
    atm_model est_atm;
    eg16_profile est_atm_profile;
} typedef dualatm;

struct {
    dualatm dual_atm;
    runparams run_params;
    vehicle vehicle;
    grav gravity;
    int update_desired_state; // 0 no update, 1 update desired state
} typedef dualargs;

integrator_args get_est_args(dualargs dual_args) {
    integrator_args args;
    args.atm = dual_args.dual_atm.est_atm;
    args.atm_profile = dual_args.dual_atm.est_atm_profile;
    args.run_params = dual_args.run_params;
    args.vehicle = dual_args.vehicle;
    args.gravity = dual_args.gravity;
    return args;
}

integrator_args get_true_args(dualargs dual_args) {
    integrator_args args;
    args.atm = dual_args.dual_atm.true_atm;
    args.atm_profile = dual_args.dual_atm.true_atm_profile;
    args.run_params = dual_args.run_params;
    args.vehicle = dual_args.vehicle;
    args.gravity = dual_args.gravity;
    return args;
}

#endif