#ifndef SPHERE_H
#define SPHERE_H
/*
Conversions to/from spherical coordinates and Cartesian coordinates
*/

#include "linalg.h"
#include <math.h>

struct {
  double r;
  double lat;
  double lon;

} typedef spherevec;

/**
 * Converts Cartesian coordinates to spherical coordinates
 *
 * @param cart_coords cartvec to convert
 * @return r, lat, lon
 */
spherevec cartcoords_to_sphercoords(cartvec cart_coords) {
  spherevec sphere_coords;
  // Calculate the radial coordinate
  sphere_coords.r =
      sqrt(cart_coords.x * cart_coords.x + cart_coords.y * cart_coords.y +
           cart_coords.z * cart_coords.z);

  // Calculate the longitudinal coordinate
  sphere_coords.lon = atan2(cart_coords.y, cart_coords.x);

  // Calculate the latitudinal coordinate
  sphere_coords.lat = atan(cart_coords.z / sqrt(cart_coords.x * cart_coords.x +
                                                cart_coords.y * cart_coords.y));
  return sphere_coords;
}

/**
 * Converts spherical coordinates to Cartesian coordinates
 *
 * @param spherevec spherevec to convert
 * @return x, y, z
 */
cartvec sphercoords_to_cartcoords(spherevec spherecoords) {
  cartvec cart_vec;

  cart_vec.x = spherecoords.r * cos(spherecoords.lat) * cos(spherecoords.lon);
  cart_vec.y = spherecoords.r * cos(spherecoords.lat) * sin(spherecoords.lon);
  cart_vec.z = spherecoords.r * sin(spherecoords.lat);

  return cart_vec;
}

/**
 * Converts a spherical vector to a Cartesian vector at a given set of spherical
 * coordinates
 * @param sphervec pointer to spherical vector [r, long, lat]
 * @param sphere_coords pointer to spherical coordinates [r, long, lat]
 * @return cartvec
 */
cartvec spherevec_to_cartvec(spherevec sphere_vec, spherevec sphere_coords) {
  cartvec cart_vec;
  // Get the x-component of the spherical vector
  cart_vec.x =
      -sphere_vec.lon * sin(sphere_coords.lon) -
      sphere_vec.lat * sin(sphere_coords.lat) * cos(sphere_coords.lon) +
      sphere_vec.r * cos(sphere_coords.lon) * cos(sphere_coords.lat);

  // Get the y-component of the spherical vector
  cart_vec.y =
      sphere_vec.lon * cos(sphere_coords.lon) -
      sphere_vec.lat * sin(sphere_coords.lat) * sin(sphere_coords.lon) +
      sphere_vec.r * sin(sphere_coords.lon) * cos(sphere_coords.lat);

  // Get the z-component of the spherical vector
  cart_vec.z = sphere_vec.lat * cos(sphere_coords.lat) +
               sphere_vec.r * sin(sphere_coords.lat);

  return cart_vec;
}

#endif