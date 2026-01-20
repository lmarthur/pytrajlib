#ifndef STATE_H
#define STATE_H

#include "math/linalg.h"

struct {
    cartvec position;
    cartvec velocity;
    cartvec a_lift;
    cartvec a_lift_avail;
    double gyro_error[2];
    quaternion quaternion;
} typedef state;

#endif