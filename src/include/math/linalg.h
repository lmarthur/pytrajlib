#ifndef LINALG_H
#define LINALG_H

struct {
    double x;
    double y;
    double z;

} typedef cartvec;

struct {
    double lat;
    double lon;
} typedef anglevec;

/**
 * A quaternion has a real component w and a vector component x, y, z
 */
typedef struct {
    double w;
    double x;
    double y;
    double z;
} quaternion;

/**
 * Get the L2 norm of a 3-vector
 * 
 */
double norm(cartvec vec) {
    return sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
}

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
 * Divide a 3-vector by a scalar
 */
cartvec divide(cartvec vec, double s) {
    cartvec result;
    result.x = vec.x / s;
    result.y = vec.y / s;
    result.z = vec.z / s;
    return result;
}

/**
 * Multiply two quaternions: result = q1 * q2
 * 
 * @param q1 First quaternion
 * @param q2 Second quaternion
 * @return The product quaternion q1 * q2
 */
quaternion qmultiply(quaternion q1, quaternion q2) {
    quaternion result;
    result.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
    result.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
    result.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
    result.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
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

#endif // LINALG_H
