#ifndef LINALG_H
#define LINALG_H

#include <math.h>

#include "../rng/rng.h"

typedef struct cartvec {
  double x;
  double y;
  double z;
} cartvec;

typedef struct anglevec {
  double lat;
  double lon;
} anglevec;

cartvec zeros() {
  cartvec z;
  z.x = 0;
  z.y = 0;
  z.z = 0;
  return z;
}

/**
 * Generate a cartesian vector with independent N(0, 1) entries.
 */
cartvec gaussian_cartvec() {
  cartvec g;
  g.x = ran_gaussian(1);
  g.y = ran_gaussian(1);
  g.z = ran_gaussian(1);
  return g;
}

/**
 * Generate an angle vector with independent N(0, 1) entries.
 */
anglevec gaussian_anglevec() {
  anglevec g;
  g.lat = ran_gaussian(1);
  g.lon = ran_gaussian(1);
  return g;
}

/**
 * Compute the dot product of two 3-vectors: result = a · b
 *
 * @param a First vector
 * @param b Second vector
 * @return The scalar dot product
 */
double dot(cartvec a, cartvec b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

/**
 * Get the L2 norm of a 3-vector
 *
 * @param vec Vector to compute norm of
 * @return The L2 norm
 */
double norm(cartvec vec) { return sqrt(dot(vec, vec)); }

/**
 * Multiply a 3-vector by a scalar
 */
cartvec smultiply(cartvec vec, double s) {
  cartvec result;
  result.x = vec.x * s;
  result.y = vec.y * s;
  result.z = vec.z * s;
  return result;
}

/**
 * Multiply an anglevec by a scalar
 */
anglevec smultiply_angle(anglevec vec, double s) {
  anglevec result;
  result.lat = vec.lat * s;
  result.lon = vec.lon * s;
  return result;
}

/**
 * Divide a 3-vector by a scalar
 */
cartvec sdivide(cartvec vec, double s) {
  cartvec result;
  result.x = vec.x / s;
  result.y = vec.y / s;
  result.z = vec.z / s;
  return result;
}

/**
 * Elementwise multiplication of two anglevecs
 */
anglevec multiply_anglevec(anglevec a, anglevec b) {
  anglevec result;
  result.lat = a.lat * b.lat;
  result.lon = a.lon * b.lon;
  return result;
}

/**
 * Add vector b to vector a
 */
cartvec add(cartvec a, cartvec b) {
  cartvec result;
  result.x = a.x + b.x;
  result.y = a.y + b.y;
  result.z = a.z + b.z;
  return result;
}

/**
 * Add anglevec b to anglevec a
 */
anglevec add_anglevec(anglevec a, anglevec b) {
  anglevec result;
  result.lat = a.lat + b.lat;
  result.lon = a.lon + b.lon;
  return result;
}

/**
 * Subtract vector b from vector a
 */
cartvec subtract(cartvec a, cartvec b) {
  cartvec result;
  result.x = a.x - b.x;
  result.y = a.y - b.y;
  result.z = a.z - b.z;
  return result;
}

/**
 * Multiply a 3x3 matrix by a 3-vector: result = matrix * vec
 *
 * @param matrix 3x3 matrix stored as double[3][3]
 * @param vec 3-vector
 * @return The product vector
 */
cartvec matvec_multiply(double matrix[3][3], cartvec vec) {
  cartvec result;
  result.x = matrix[0][0] * vec.x + matrix[0][1] * vec.y + matrix[0][2] * vec.z;
  result.y = matrix[1][0] * vec.x + matrix[1][1] * vec.y + matrix[1][2] * vec.z;
  result.z = matrix[2][0] * vec.x + matrix[2][1] * vec.y + matrix[2][2] * vec.z;
  return result;
}

/**
 * Compute the cross product of two 3-vectors: result = a x b
 *
 * @param a First vector
 * @param b Second vector
 * @return The cross product vector
 */
cartvec cross(cartvec a, cartvec b) {
  cartvec result;
  result.x = a.y * b.z - a.z * b.y;
  result.y = a.z * b.x - a.x * b.z;
  result.z = a.x * b.y - a.y * b.x;
  return result;
}

/**
 * Rotate vector v around unit vector k by specified angle.
 * Uses Rodrigues' rotation formula
 * https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
 */
cartvec rotate(cartvec v, cartvec k, double angle) {
  cartvec v_rot =
      add(add(smultiply(v, cos(angle)), smultiply(cross(k, v), sin(angle))),
          smultiply(smultiply(k, dot(k, v)), 1 - cos(angle)));
  return v_rot;
}

#endif