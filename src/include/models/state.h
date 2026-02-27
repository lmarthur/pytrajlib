#ifndef STATE_H
#define STATE_H

// Define a struct to store the state of a vehicle in 3D space
typedef struct state {
  // State parameters
  double t;       // time in seconds since launch
  double x;       // x-coordinate in meters
  double y;       // y-coordinate in meters
  double z;       // z-coordinate in meters
  double vx;      // x-velocity in meters per second
  double vy;      // y-velocity in meters per second
  double vz;      // z-velocity in meters per second
  double ax_grav; // x-acceleration due to gravity in meters per second squared
  double ay_grav; // y-acceleration due to gravity in meters per second squared
  double az_grav; // z-acceleration due to gravity in meters per second squared
  double ax_drag; // x-acceleration due to drag in meters per second squared
  double ay_drag; // y-acceleration due to drag in meters per second squared
  double az_drag; // z-acceleration due to drag in meters per second squared
  double ax_lift; // x-acceleration due to lift in meters per second squared
  double ay_lift; // y-acceleration due to lift in meters per second squared
  double az_lift; // z-acceleration due to lift in meters per second squared
  double ax_lift_avail; // "available" lift. Encodes flap positions
  double ay_lift_avail; // "available" lift. Encodes flap positions
  double az_lift_avail; // "available" lift. Encodes flap positions
  double ax_thrust; // x-acceleration due to thrust in meters per second squared
  double ay_thrust; // y-acceleration due to thrust in meters per second squared
  double az_thrust; // z-acceleration due to thrust in meters per second squared
  double ax_total;  // total x-acceleration in meters per second squared
  double ay_total;  // total y-acceleration in meters per second squared
  double az_total;  // total z-acceleration in meters per second squared
  double initial_theta_long_pert; // initial perturbation in the longitudinal
                                  // thrust angle in radians
  double initial_theta_lat_pert;  // initial perturbation in the latitudinal
                                  // thrust angle in radians
  double theta_long; // thrust angle in the longitudinal direction measured from
                     // the x-z plane in radians
  double theta_lat;  // thrust angle in the latitudinal direction measured from
                     // the x-y plane in radians
  double roll; // change in roll angle in radians from the initial orientation
               // at the start of reentry. A positive yaw angle and pitch
               // corresponds to a positive roll.

} state;

#endif