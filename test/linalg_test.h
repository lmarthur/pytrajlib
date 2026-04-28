#include "../src/include/math/linalg.h"
#include <math.h>
#include <tau/tau.h>

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
  cartvec result = sdivide(v, 2.0);
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

TEST(linalg, rotate) {
  // Test Rodrigues' rotation

  // Rotation around orthogonal vectors
  cartvec a = {1.0, 0.0, 0.0};
  cartvec b = {0.0, 1.0, 0};
  cartvec c = rotate(a, b, M_PI_2);
  REQUIRE_LT(fabs(c.x), 1e-10);
  REQUIRE_LT(fabs(c.y), 1e-10);
  REQUIRE_EQ(c.z, -1);
}

TEST(linalg, rotate_non_orthogonal) {
  // Test Rodrigues' rotation

  // Rotation around non orthogonal vectors
  cartvec a = {1.0, 1.0, 0.0};
  cartvec b = {0.0, 1.0, 0};
  cartvec c = rotate(a, b, M_PI);
  REQUIRE_EQ(c.x, -1);
  REQUIRE_EQ(c.y, 1);
  REQUIRE_LT(c.z, 1e-10);
}

TEST(linalg, get_body_to_eci_matrix_identity) {
  quaternion q = identity_quaternion();
  double C_EB[3][3] = {{0}};

  get_body_to_eci_matrix(q, C_EB);

  REQUIRE_EQ(C_EB[0][0], 1.0);
  REQUIRE_EQ(C_EB[0][1], 0.0);
  REQUIRE_EQ(C_EB[0][2], 0.0);
  REQUIRE_EQ(C_EB[1][0], 0.0);
  REQUIRE_EQ(C_EB[1][1], 1.0);
  REQUIRE_EQ(C_EB[1][2], 0.0);
  REQUIRE_EQ(C_EB[2][0], 0.0);
  REQUIRE_EQ(C_EB[2][1], 0.0);
  REQUIRE_EQ(C_EB[2][2], 1.0);
}

TEST(linalg, body_to_eci_quarter_turn_about_z) {
  const double half_sqrt2 = sqrt(0.5);
  quaternion q = {
      .w = half_sqrt2,
      .x = 0.0,
      .y = 0.0,
      .z = half_sqrt2,
  };
  cartvec v_B = {1.0, 0.0, 0.0};

  cartvec v_E = body_to_eci(v_B, q);

  REQUIRE_LT(fabs(v_E.x), 1e-12);
  REQUIRE_LT(fabs(v_E.y - 1.0), 1e-12);
  REQUIRE_LT(fabs(v_E.z), 1e-12);
}

TEST(linalg, eci_to_body_inverse_of_body_to_eci) {
  /* Start with a vector in body frame */
  cartvec v_B_original = {1.0, 2.0, 3.0};

  const double half_sqrt2 = sqrt(0.5);
  quaternion q = {
      .w = half_sqrt2,
      .x = 0.0,
      .y = 0.0,
      .z = half_sqrt2,
  };

  /* Transform body -> ECI -> body, should return to original */
  cartvec v_E = body_to_eci(v_B_original, q);
  cartvec v_B_recovered = eci_to_body(v_E, q);

  REQUIRE_LT(fabs(v_B_recovered.x - v_B_original.x), 1e-12);
  REQUIRE_LT(fabs(v_B_recovered.y - v_B_original.y), 1e-12);
  REQUIRE_LT(fabs(v_B_recovered.z - v_B_original.z), 1e-12);
}

TEST(linalg, eci_to_body_quarter_turn_about_z) {
  /* 90 degree rotation about z-axis for body_to_eci: (1,0,0) body -> (0,1,0)
   * ECI */
  /* For eci_to_body (transpose = -90 degrees): (1,0,0) ECI -> (0,-1,0) body */
  const double half_sqrt2 = sqrt(0.5);
  quaternion q = {
      .w = half_sqrt2,
      .x = 0.0,
      .y = 0.0,
      .z = half_sqrt2,
  };
  cartvec v_E = {1.0, 0.0, 0.0};

  cartvec v_B = eci_to_body(v_E, q);

  REQUIRE_LT(fabs(v_B.x), 1e-12);
  REQUIRE_LT(fabs(v_B.y + 1.0), 1e-12); /* v_B.y should be -1 */
  REQUIRE_LT(fabs(v_B.z), 1e-12);
}
