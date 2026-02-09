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
 * Subtract vector b from vector a
 */
cart_vector subtract(cart_vector a, cart_vector b) {
    cart_vector result;
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    result.z = a.z - b.z;
    return result;
}
#endif