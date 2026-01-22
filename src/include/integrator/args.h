#ifndef ARGS_H
#define ARGS_H

#include "models/atmosphere.h"
#include "models/vehicle.h"

struct {
  atm_model atm;
  eg16_profile atm_profile;
  runparams run_params;
  vehicle vehicle;

} typedef integrator_args;

#endif