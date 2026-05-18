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
  double pitch;
  double yaw;
} anglevec;

typedef struct quaternion {
  double w;
  double x;
  double y;
  double z;
} quaternion;

/**
 * Create the identity quaternion.
 *
 * @return Quaternion with no rotation.
 */
quaternion identity_quaternion() {
  quaternion q;
  q.w = 1;
  q.x = 0;
  q.y = 0;
  q.z = 0;
  return q;
}

/**
 * Hamilton product of two quaternions
 *
 * @param q1 First quaternion
 * @param q2 Second quaternion
 * @return The product quaternion
 */
quaternion qmultiply(quaternion q1, quaternion q2) {
  quaternion result;
  result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
  result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
  result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
  result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
  return result;
}

/**
 * Multiply each quaternion component by a scalar.
 *
 * @param q Quaternion to scale.
 * @param s Scalar multiplier.
 * @return Scaled quaternion.
 */
quaternion qsmultiply(quaternion q, double s) {
  quaternion result;
  result.w = q.w * s;
  result.x = q.x * s;
  result.y = q.y * s;
  result.z = q.z * s;
  return result;
}

/**
 * Compute the L2 norm of a quaternion.
 *
 * @param q Quaternion to compute norm of.
 * @return Quaternion norm.
 */
double qnorm(quaternion q) {
  return sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

/**
 * Build the body-to-ECI direction cosine matrix from scalar-first quaternion
 * q_EB = [w, x, y, z].
 *
 * @param q_EB Quaternion rotating body-frame vectors into ECI.
 * @param C_EB Output 3x3 body-to-ECI DCM.
 */
void get_body_to_eci_matrix(quaternion q_EB, double C_EB[3][3]) {
  const double q0 = q_EB.w;
  const double q1 = q_EB.x;
  const double q2 = q_EB.y;
  const double q3 = q_EB.z;

  C_EB[0][0] = 1.0 - 2.0 * (q2 * q2 + q3 * q3);
  C_EB[0][1] = 2.0 * (q1 * q2 - q0 * q3);
  C_EB[0][2] = 2.0 * (q1 * q3 + q0 * q2);

  C_EB[1][0] = 2.0 * (q1 * q2 + q0 * q3);
  C_EB[1][1] = 1.0 - 2.0 * (q1 * q1 + q3 * q3);
  C_EB[1][2] = 2.0 * (q2 * q3 - q0 * q1);

  C_EB[2][0] = 2.0 * (q1 * q3 - q0 * q2);
  C_EB[2][1] = 2.0 * (q2 * q3 + q0 * q1);
  C_EB[2][2] = 1.0 - 2.0 * (q1 * q1 + q2 * q2);
}

/**
 * Transform a vector from body coordinates to ECI coordinates using q_EB.
 *
 * @param v_B Vector in body coordinates.
 * @param q_EB Quaternion rotating body-frame vectors into ECI.
 * @return Vector expressed in ECI coordinates.
 */
cartvec body_to_eci(cartvec v_B, quaternion q_EB) {
  double C_EB[3][3];
  get_body_to_eci_matrix(q_EB, C_EB);
  cartvec v_E;
  v_E.x = C_EB[0][0] * v_B.x + C_EB[0][1] * v_B.y + C_EB[0][2] * v_B.z;
  v_E.y = C_EB[1][0] * v_B.x + C_EB[1][1] * v_B.y + C_EB[1][2] * v_B.z;
  v_E.z = C_EB[2][0] * v_B.x + C_EB[2][1] * v_B.y + C_EB[2][2] * v_B.z;
  return v_E;
}

/**
 * Transform a vector from ECI coordinates to body coordinates using q_EB.
 * Applies the transpose of C_EB (the ECI-to-body rotation) using flipped
 * indexing. Equivalent to: v_B = C_EB^T * v_E where C_EB^T = C_EB^(-1) for
 * orthogonal rotations.
 *
 * @param v_E Vector in ECI coordinates.
 * @param q_EB Quaternion rotating body-frame vectors into ECI.
 * @return Vector expressed in body coordinates.
 */
cartvec eci_to_body(cartvec v_E, quaternion q_EB) {
  double C_EB[3][3];
  get_body_to_eci_matrix(q_EB, C_EB);

  /* Compute v_B = C_EB^T * v_E */
  cartvec v_B;
  v_B.x = C_EB[0][0] * v_E.x + C_EB[1][0] * v_E.y + C_EB[2][0] * v_E.z;
  v_B.y = C_EB[0][1] * v_E.x + C_EB[1][1] * v_E.y + C_EB[2][1] * v_E.z;
  v_B.z = C_EB[0][2] * v_E.x + C_EB[1][2] * v_E.y + C_EB[2][2] * v_E.z;
  return v_B;
}

/**
 * Create a zero 3-vector.
 *
 * @return Vector with all components equal to zero.
 */
cartvec zeros() {
  cartvec z;
  z.x = 0;
  z.y = 0;
  z.z = 0;
  return z;
}

/**
 * Generate a cartesian vector with independent N(0, 1) entries.
 *
 * @return Gaussian random 3-vector.
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
 *
 * @return Gaussian random angle vector.
 */
anglevec gaussian_anglevec() {
  anglevec g;
  g.pitch = ran_gaussian(1);
  g.yaw = ran_gaussian(1);
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
 * Multiply a 3-vector by a scalar.
 *
 * @param vec Vector to scale.
 * @param s Scalar multiplier.
 * @return Scaled vector.
 */
cartvec smultiply(cartvec vec, double s) {
  cartvec result;
  result.x = vec.x * s;
  result.y = vec.y * s;
  result.z = vec.z * s;
  return result;
}

/**
 * Multiply an angle vector by a scalar.
 *
 * @param vec Angle vector to scale.
 * @param s Scalar multiplier.
 * @return Scaled angle vector.
 */
anglevec smultiply_angle(anglevec vec, double s) {
  anglevec result;
  result.pitch = vec.pitch * s;
  result.yaw = vec.yaw * s;
  return result;
}

/**
 * Divide a 3-vector by a scalar.
 *
 * @param vec Vector to divide.
 * @param s Scalar divisor.
 * @return Scaled vector.
 */
cartvec sdivide(cartvec vec, double s) {
  cartvec result;
  result.x = vec.x / s;
  result.y = vec.y / s;
  result.z = vec.z / s;
  return result;
}

/**
 * Elementwise multiplication of two angle vectors.
 *
 * @param a First angle vector.
 * @param b Second angle vector.
 * @return Elementwise product.
 */
anglevec multiply_anglevec(anglevec a, anglevec b) {
  anglevec result;
  result.pitch = a.pitch * b.pitch;
  result.yaw = a.yaw * b.yaw;
  return result;
}

/**
 * Elementwise multiplication of two cartvecs.
 *
 * @param a First cartvec.
 * @param b Second cartvec.
 * @return Elementwise product.
 */
cartvec multiply_cartvec(cartvec a, cartvec b) {
  cartvec result;
  result.x = a.x * b.x;
  result.y = a.y * b.y;
  result.z = a.z * b.z;
  return result;
}

/**
 * Add vector b to vector a.
 *
 * @param a First vector.
 * @param b Second vector.
 * @return Sum vector.
 */
cartvec add(cartvec a, cartvec b) {
  cartvec result;
  result.x = a.x + b.x;
  result.y = a.y + b.y;
  result.z = a.z + b.z;
  return result;
}

/**
 * Add angle vector b to angle vector a.
 *
 * @param a First angle vector.
 * @param b Second angle vector.
 * @return Sum angle vector.
 */
anglevec add_anglevec(anglevec a, anglevec b) {
  anglevec result;
  result.pitch = a.pitch + b.pitch;
  result.yaw = a.yaw + b.yaw;
  return result;
}

/**
 * Subtract vector b from vector a.
 *
 * @param a First vector.
 * @param b Second vector.
 * @return Difference vector.
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
 * Uses Rodrigues' rotation formula.
 *
 * >Wikipedia contributors. (2026). Rodrigues' rotation formula—Wikipedia, The
 * Free Encyclopedia.
 * https://en.wikipedia.org/w/index.php?title=Rodrigues%27_rotation_formula&oldid=1340370650
 *
 * @param v Vector to rotate.
 * @param k Unit vector defining rotation axis.
 * @param angle Rotation angle in radians.
 * @return Rotated vector.
 */
cartvec rotate(cartvec v, cartvec k, double angle) {
  cartvec v_rot =
      add(add(smultiply(v, cos(angle)), smultiply(cross(k, v), sin(angle))),
          smultiply(smultiply(k, dot(k, v)), 1 - cos(angle)));
  return v_rot;
}

/**
 * Project vector v onto vector u.
 *
 * @param v Vector to project.
 * @param u Direction to project onto.
 * @return Projection of v onto u.
 */
cartvec project(cartvec v, cartvec u) {
  return smultiply(u, dot(v, u) / dot(u, u));
}

/**
 * Create an orthonormal basis e0, e1 from linearly independent vectors
 * v0, v1 where e0 = v0 / norm(v0).
 *
 * @param v0 First input vector.
 * @param v1 Second input vector.
 * @param e0 Output first orthonormal basis vector.
 * @param e1 Output second orthonormal basis vector.
 */
void gram_schmidt_orthonorm(cartvec v0, cartvec v1, cartvec *e0, cartvec *e1) {
  cartvec u0 = v0;
  cartvec u1 = subtract(v1, project(v1, u0));

  *e0 = sdivide(u0, norm(u0));
  *e1 = sdivide(u1, norm(u1));
}

#endif