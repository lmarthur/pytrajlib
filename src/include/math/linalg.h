#ifndef LINALG_H
#define LINALG_H

/**
 * Multiply two quaternions: result = q1 * q2
 * Quaternion format: [w, x, y, z] where w is the scalar part
 * 
 * @param q1 First quaternion [w, x, y, z]
 * @param q2 Second quaternion [w, x, y, z]
 * @param result Output quaternion [w, x, y, z]
 */
static inline void quat_multiply(const double q1[4], const double q2[4], double result[4]) {
    result[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];  // w
    result[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];  // x
    result[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];  // y
    result[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];  // z
}

#endif // LINALG_H
