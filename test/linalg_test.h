#include <math.h>
#include <tau/tau.h>
#include "../src/include/math/linalg.h"


TEST(linalg, quat_multiply_identity){
    // Test identity quaternion multiplication: q * [1, 0, 0, 0] = q
    double q1[4] = {0.5, 0.5, 0.5, 0.5};
    double identity[4] = {1.0, 0.0, 0.0, 0.0};
    double result[4];
    
    quat_multiply(q1, identity, result);
    
    REQUIRE_EQ(result[0], 0.5);
    REQUIRE_EQ(result[1], 0.5);
    REQUIRE_EQ(result[2], 0.5);
    REQUIRE_EQ(result[3], 0.5);
}

TEST(linalg, quat_multiply_basic){
    // Test basic quaternion multiplication
    double q1[4] = {1.0, 0.0, 1.0, 0.0};
    double q2[4] = {1.0, 0.5, 0.5, 0.75};
    double result[4];
    
    quat_multiply(q1, q2, result);
    
    REQUIRE_EQ(result[0], 0.5);   // w: 1*1 - 0*0.5 - 1*0.5 - 0*0.75 = 0.5
    REQUIRE_EQ(result[1], 1.25);  // x: 1*0.5 + 0*1 + 1*0.75 - 0*0.5 = 1.25
    REQUIRE_EQ(result[2], 1.5);   // y: 1*0.5 - 0*0.75 + 1*1 + 0*0.5 = 1.5
    REQUIRE_EQ(result[3], 0.25);  // z: 1*0.75 + 0*0.5 - 1*0.5 + 0*1 = 0.25
}

TEST(linalg, quat_multiply_90deg_rotations){
    // Test 90-degree rotation about z-axis
    double theta = M_PI / 2;
    double q_z_90[4] = {cos(theta / 2), 0.0, 0.0, sin(theta / 2)};
    double result[4];
    
    // Multiplying 90-degree rotation by itself should give 180-degree rotation
    quat_multiply(q_z_90, q_z_90, result);
    
    REQUIRE_LT(fabs(result[0]), 1e-10);  // cos(90°) = 0
    REQUIRE_EQ(result[1], 0.0);
    REQUIRE_EQ(result[2], 0.0);
    REQUIRE_EQ(result[3], 1.0);  // sin(90°) = 1
}
