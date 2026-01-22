#include "../src/include/math/linalg.h"
#include <math.h>
#include <tau/tau.h>

TEST(linalg, quat_multiply_identity) {
  // Test identity quaternion multiplication: q * [1, 0, 0, 0] = q
  quaternion q1 = {0.5, 0.5, 0.5, 0.5};
  quaternion identity = {1.0, 0.0, 0.0, 0.0};

  quaternion result = qmultiply(q1, identity);

  REQUIRE_EQ(result.w, 0.5);
  REQUIRE_EQ(result.x, 0.5);
  REQUIRE_EQ(result.y, 0.5);
  REQUIRE_EQ(result.z, 0.5);
}

TEST(linalg, quat_multiply_basic) {
  // Test basic quaternion multiplication
  quaternion q1 = {1.0, 0.0, 1.0, 0.0};
  quaternion q2 = {1.0, 0.5, 0.5, 0.75};

  quaternion result = qmultiply(q1, q2);

  REQUIRE_EQ(result.w, 0.5);  // w: 1*1 - 0*0.5 - 1*0.5 - 0*0.75 = 0.5
  REQUIRE_EQ(result.x, 1.25); // x: 1*0.5 + 0*1 + 1*0.75 - 0*0.5 = 1.25
  REQUIRE_EQ(result.y, 1.5);  // y: 1*0.5 - 0*0.75 + 1*1 + 0*0.5 = 1.5
  REQUIRE_EQ(result.z, 0.25); // z: 1*0.75 + 0*0.5 - 1*0.5 + 0*1 = 0.25
}

TEST(linalg, quat_multiply_90deg_rotations) {
  // Test 90-degree rotation about z-axis
  double theta = M_PI / 2;
  quaternion q_z_90 = {cos(theta / 2), 0.0, 0.0, sin(theta / 2)};

  // Multiplying 90-degree rotation by itself should give 180-degree rotation
  quaternion result = qmultiply(q_z_90, q_z_90);

  REQUIRE_LT(fabs(result.w), 1e-10); // cos(90°) = 0
  REQUIRE_EQ(result.x, 0.0);
  REQUIRE_EQ(result.y, 0.0);
  REQUIRE_EQ(result.z, 1.0); // sin(90°) = 1
}

TEST(linalg, dot_product_orthogonal) {
  // Test dot product of orthogonal vectors
  cartvec a = {1.0, 0.0, 0.0};
  cartvec b = {0.0, 1.0, 0.0};
  double result = dot(a, b);
  REQUIRE_EQ(result, 0.0);
}

TEST(linalg, dot_product_parallel) {
  // Test dot product of parallel vectors
  cartvec a = {1.0, 2.0, 3.0};
  cartvec b = {2.0, 4.0, 6.0};
  double result = dot(a, b);
  REQUIRE_EQ(result, 28.0); // 1*2 + 2*4 + 3*6 = 28
}

TEST(linalg, dot_product_self) {
  // Test dot product of vector with itself
  cartvec v = {3.0, 4.0, 0.0};
  double result = dot(v, v);
  REQUIRE_EQ(result, 25.0); // 3^2 + 4^2 = 25
}

TEST(linalg, norm_basic) {
  // Test L2 norm calculation
  cartvec v = {3.0, 4.0, 0.0};
  double result = norm(v);
  REQUIRE_EQ(result, 5.0);
}

TEST(linalg, norm_unit_vector) {
  // Test norm of unit vector
  cartvec v = {1.0, 0.0, 0.0};
  double result = norm(v);
  REQUIRE_EQ(result, 1.0);
}

TEST(linalg, smultiply_basic) {
  // Test scalar multiplication
  cartvec v = {1.0, 2.0, 3.0};
  cartvec result = smultiply(v, 2.0);
  REQUIRE_EQ(result.x, 2.0);
  REQUIRE_EQ(result.y, 4.0);
  REQUIRE_EQ(result.z, 6.0);
}

TEST(linalg, smultiply_zero) {
  // Test scalar multiplication by zero
  cartvec v = {1.0, 2.0, 3.0};
  cartvec result = smultiply(v, 0.0);
  REQUIRE_EQ(result.x, 0.0);
  REQUIRE_EQ(result.y, 0.0);
  REQUIRE_EQ(result.z, 0.0);
}

TEST(linalg, divide_basic) {
  // Test scalar division
  cartvec v = {2.0, 4.0, 6.0};
  cartvec result = divide(v, 2.0);
  REQUIRE_EQ(result.x, 1.0);
  REQUIRE_EQ(result.y, 2.0);
  REQUIRE_EQ(result.z, 3.0);
}
TEST(linalg, matvec_multiply_identity) {
  // Test matrix-vector multiplication with identity matrix
  double identity[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
  cartvec v = {1.0, 2.0, 3.0};
  cartvec result = matvec_multiply(identity, v);
  REQUIRE_EQ(result.x, 1.0);
  REQUIRE_EQ(result.y, 2.0);
  REQUIRE_EQ(result.z, 3.0);
}

TEST(linalg, matvec_multiply_scale) {
  // Test matrix-vector multiplication with scaling matrix
  double scale[3][3] = {{2.0, 0.0, 0.0}, {0.0, 3.0, 0.0}, {0.0, 0.0, 4.0}};
  cartvec v = {1.0, 1.0, 1.0};
  cartvec result = matvec_multiply(scale, v);
  REQUIRE_EQ(result.x, 2.0);
  REQUIRE_EQ(result.y, 3.0);
  REQUIRE_EQ(result.z, 4.0);
}

TEST(linalg, matvec_multiply_general) {
  // Test general matrix-vector multiplication
  double matrix[3][3] = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {7.0, 8.0, 9.0}};
  cartvec v = {2.0, 1.0, 1.0};
  cartvec result = matvec_multiply(matrix, v);
  REQUIRE_EQ(result.x, 7.0);  // 1*2 + 2*1 + 3*1
  REQUIRE_EQ(result.y, 19.0); // 4*2 + 5*1 + 6*1
  REQUIRE_EQ(result.z, 31.0); // 7*2 + 8*1 + 9*1
}

TEST(linalg, cross_perpendicular) {
  // Test cross product of two perpendicular unit vectors
  cartvec x_axis = {1.0, 0.0, 0.0};
  cartvec y_axis = {0.0, 1.0, 0.0};
  cartvec result = cross(x_axis, y_axis);
  // x x y = z
  REQUIRE_EQ(result.x, 0.0);
  REQUIRE_EQ(result.y, 0.0);
  REQUIRE_EQ(result.z, 1.0);
}

TEST(linalg, cross_anticommutative) {
  // Test that cross product is anticommutative: a x b = -(b x a)
  cartvec a = {1.0, 2.0, 3.0};
  cartvec b = {4.0, 5.0, 6.0};
  cartvec result1 = cross(a, b);
  cartvec result2 = cross(b, a);
  // result1 should be negative of result2
  REQUIRE_EQ(result1.x, -result2.x);
  REQUIRE_EQ(result1.y, -result2.y);
  REQUIRE_EQ(result1.z, -result2.z);
}

TEST(linalg, cross_self) {
  // Test that a x a = 0
  cartvec v = {3.0, 4.0, 5.0};
  cartvec result = cross(v, v);
  REQUIRE_EQ(result.x, 0.0);
  REQUIRE_EQ(result.y, 0.0);
  REQUIRE_EQ(result.z, 0.0);
}

TEST(linalg, cross_general) {
  // Test general cross product calculation
  cartvec a = {1.0, 0.0, 0.0};
  cartvec b = {0.0, 1.0, 1.0};
  cartvec result = cross(a, b);
  // (1,0,0) x (0,1,1) = (0*1 - 0*1, 0*0 - 1*1, 1*1 - 0*0) = (0, -1, 1)
  REQUIRE_EQ(result.x, 0.0);
  REQUIRE_EQ(result.y, -1.0);
  REQUIRE_EQ(result.z, 1.0);
}