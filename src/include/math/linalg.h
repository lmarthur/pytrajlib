#ifndef LINALG_H
#define LINALG_H

#include <math.h>

typedef struct cart_vector{
    double x;
    double y;
    double z;
} cart_vector;

cart_vector zeros() {
    cart_vector z;
    z.x = 0;
    z.y = 0;
    z.z = 0;
    return z;
}

/**
 * Compute the dot product of two 3-vectors: result = a · b
 *
 * @param a First vector
 * @param b Second vector
 * @return The scalar dot product
 */
double dot(cart_vector a, cart_vector b) { return a.x * b.x + a.y * b.y + a.z * b.z; }


/**
 * Get the L2 norm of a 3-vector
 *
 * @param vec Vector to compute norm of
 * @return The L2 norm
 */
double norm(cart_vector vec) { return sqrt(dot(vec, vec)); }

/**
 * Multiply a 3-vector by a scalar
 */
cart_vector smultiply(cart_vector vec, double s) {
    cart_vector result;
    result.x = vec.x * s;
    result.y = vec.y * s;
    result.z = vec.z * s;
    return result;
}


/**
 * Divide a 3-vector by a scalar
 */
cart_vector sdivide(cart_vector vec, double s) {
    cart_vector result;
    result.x = vec.x / s;
    result.y = vec.y / s;
    result.z = vec.z / s;
    return result;
}

/**
 * Add vector b to vector a
 */
cart_vector add(cart_vector a, cart_vector b) {
    cart_vector result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;
    return result;
}

/**
 * Subtract vector b from vector a
 */
cart_vector subtract(cart_vector a, cart_vector b) {
    cart_vector result;
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
cart_vector matvec_multiply(double matrix[3][3], cart_vector vec) {
    cart_vector result;
    result.x =
        matrix[0][0] * vec.x + matrix[0][1] * vec.y + matrix[0][2] * vec.z;
    result.y =
        matrix[1][0] * vec.x + matrix[1][1] * vec.y + matrix[1][2] * vec.z;
    result.z =
        matrix[2][0] * vec.x + matrix[2][1] * vec.y + matrix[2][2] * vec.z;
    return result;
}

/**
 * Compute the cross product of two 3-vectors: result = a x b
 *
 * @param a First vector
 * @param b Second vector
 * @return The cross product vector
 */
cart_vector cross(cart_vector a, cart_vector b) {
    cart_vector result;
    result.x = a.y * b.z - a.z * b.y;
    result.y = a.z * b.x - a.x * b.z;
    result.z = a.x * b.y - a.y * b.x;
    return result;
}

/**
 * Rotate vector v around unit vector k by specified angle.
 * Uses Rodrigues' rotation formula https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
 */
cart_vector rotate(cart_vector v, cart_vector k, double angle) {
    cart_vector v_rot  = add(add(
        smultiply(v, cos(angle)),
        smultiply(cross(k, v), sin(angle))),
        smultiply(smultiply(k, dot(k, v)), 1 - cos(angle))
    );
    return v_rot;
}


#endif