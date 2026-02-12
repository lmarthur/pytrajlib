#include <tau/tau.h>
#include "../src/include/math/linalg.h"
#include <math.h>

TEST(linalg, dot_product_orthogonal) {
  // Test dot product of orthogonal vectors
  cart_vector a = {1.0, 0.0, 0.0};
  cart_vector b = {0.0, 1.0, 0.0};
  double result = dot(a, b);
  REQUIRE_EQ(result, 0.0);
}

TEST(linalg, dot_product_parallel) {
  // Test dot product of parallel vectors
  cart_vector a = {1.0, 2.0, 3.0};
  cart_vector b = {2.0, 4.0, 6.0};
  double result = dot(a, b);
  REQUIRE_EQ(result, 28.0); // 1*2 + 2*4 + 3*6 = 28
}

TEST(linalg, dot_product_self) {
  // Test dot product of vector with itself
  cart_vector v = {3.0, 4.0, 0.0};
  double result = dot(v, v);
  REQUIRE_EQ(result, 25.0); // 3^2 + 4^2 = 25
}

TEST(linalg, norm_basic) {
  // Test L2 norm calculation
  cart_vector v = {3.0, 4.0, 0.0};
  double result = norm(v);
  REQUIRE_EQ(result, 5.0);
}

TEST(linalg, norm_unit_vector) {
  // Test norm of unit vector
  cart_vector v = {1.0, 0.0, 0.0};
  double result = norm(v);
  REQUIRE_EQ(result, 1.0);
}

TEST(linalg, smultiply_basic) {
  // Test scalar multiplication
  cart_vector v = {1.0, 2.0, 3.0};
  cart_vector result = smultiply(v, 2.0);
  REQUIRE_EQ(result.x, 2.0);
  REQUIRE_EQ(result.y, 4.0);
  REQUIRE_EQ(result.z, 6.0);
}

TEST(linalg, smultiply_zero) {
  // Test scalar multiplication by zero
  cart_vector v = {1.0, 2.0, 3.0};
  cart_vector result = smultiply(v, 0.0);
  REQUIRE_EQ(result.x, 0.0);
  REQUIRE_EQ(result.y, 0.0);
  REQUIRE_EQ(result.z, 0.0);
}

TEST(linalg, divide_basic) {
  // Test scalar division
  cart_vector v = {2.0, 4.0, 6.0};
  cart_vector result = sdivide(v, 2.0);
  REQUIRE_EQ(result.x, 1.0);
  REQUIRE_EQ(result.y, 2.0);
  REQUIRE_EQ(result.z, 3.0);
}
TEST(linalg, matvec_multiply_identity) {
  // Test matrix-vector multiplication with identity matrix
  double identity[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
  cart_vector v = {1.0, 2.0, 3.0};
  cart_vector result = matvec_multiply(identity, v);
  REQUIRE_EQ(result.x, 1.0);
  REQUIRE_EQ(result.y, 2.0);
  REQUIRE_EQ(result.z, 3.0);
}

TEST(linalg, matvec_multiply_scale) {
  // Test matrix-vector multiplication with scaling matrix
  double scale[3][3] = {{2.0, 0.0, 0.0}, {0.0, 3.0, 0.0}, {0.0, 0.0, 4.0}};
  cart_vector v = {1.0, 1.0, 1.0};
  cart_vector result = matvec_multiply(scale, v);
  REQUIRE_EQ(result.x, 2.0);
  REQUIRE_EQ(result.y, 3.0);
  REQUIRE_EQ(result.z, 4.0);
}

TEST(linalg, matvec_multiply_general) {
  // Test general matrix-vector multiplication
  double matrix[3][3] = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {7.0, 8.0, 9.0}};
  cart_vector v = {2.0, 1.0, 1.0};
  cart_vector result = matvec_multiply(matrix, v);
  REQUIRE_EQ(result.x, 7.0);  // 1*2 + 2*1 + 3*1
  REQUIRE_EQ(result.y, 19.0); // 4*2 + 5*1 + 6*1
  REQUIRE_EQ(result.z, 31.0); // 7*2 + 8*1 + 9*1
}

TEST(linalg, cross_perpendicular) {
  // Test cross product of two perpendicular unit vectors
  cart_vector x_axis = {1.0, 0.0, 0.0};
  cart_vector y_axis = {0.0, 1.0, 0.0};
  cart_vector result = cross(x_axis, y_axis);
  // x x y = z
  REQUIRE_EQ(result.x, 0.0);
  REQUIRE_EQ(result.y, 0.0);
  REQUIRE_EQ(result.z, 1.0);
}

TEST(linalg, cross_anticommutative) {
  // Test that cross product is anticommutative: a x b = -(b x a)
  cart_vector a = {1.0, 2.0, 3.0};
  cart_vector b = {4.0, 5.0, 6.0};
  cart_vector result1 = cross(a, b);
  cart_vector result2 = cross(b, a);
  // result1 should be negative of result2
  REQUIRE_EQ(result1.x, -result2.x);
  REQUIRE_EQ(result1.y, -result2.y);
  REQUIRE_EQ(result1.z, -result2.z);
}

TEST(linalg, cross_self) {
  // Test that a x a = 0
  cart_vector v = {3.0, 4.0, 5.0};
  cart_vector result = cross(v, v);
  REQUIRE_EQ(result.x, 0.0);
  REQUIRE_EQ(result.y, 0.0);
  REQUIRE_EQ(result.z, 0.0);
}

TEST(linalg, cross_general) {
  // Test general cross product calculation
  cart_vector a = {1.0, 0.0, 0.0};
  cart_vector b = {0.0, 1.0, 1.0};
  cart_vector result = cross(a, b);
  // (1,0,0) x (0,1,1) = (0*1 - 0*1, 0*0 - 1*1, 1*1 - 0*0) = (0, -1, 1)
  REQUIRE_EQ(result.x, 0.0);
  REQUIRE_EQ(result.y, -1.0);
  REQUIRE_EQ(result.z, 1.0);
}

TEST(linalg, rotate) {
    // Test Rodrigues' rotation

    // Rotation around orthogonal vectors
    cart_vector a = {1.0, 0.0, 0.0};
    cart_vector b = {0.0, 1.0, 0};
    cart_vector c = rotate(a, b, M_PI_2);
    REQUIRE_LT(fabs(c.x), 1e-10);
    REQUIRE_LT(fabs(c.y), 1e-10);
    REQUIRE_EQ(c.z, -1);

}

TEST(linalg, rotate_non_orthogonal) {
    // Test Rodrigues' rotation

    // Rotation around non orthogonal vectors
    cart_vector a = {1.0, 1.0, 0.0};
    cart_vector b = {0.0, 1.0, 0};
    cart_vector c = rotate(a, b, M_PI);
    REQUIRE_EQ(c.x, -1);
    REQUIRE_EQ(c.y, 1);
    REQUIRE_LT(c.z, 1e-10);
}


