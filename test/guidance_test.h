#include <tau/tau.h>
#include "../src/include/guidance.h"

TEST(guidance, prop_nav){
    runparams run_params;
    state estimated_state;

    run_params.x_aim = 0;
    run_params.y_aim = 0;
    run_params.z_aim = 0;
    run_params.nav_gain = 5;

    estimated_state.x = 10;
    estimated_state.y = 0;
    estimated_state.z = 0;
    estimated_state.vx = -1;
    estimated_state.vy = 0;
    estimated_state.vz = 0;

    cart_vector a_command = prop_nav(&run_params, &estimated_state);
    REQUIRE_EQ(a_command.x, 0);
    REQUIRE_EQ(a_command.y, 0);
    REQUIRE_EQ(a_command.z, 0);

    estimated_state.x = 10;
    estimated_state.y = 0;
    estimated_state.z = 0;
    estimated_state.vx = -1;
    estimated_state.vy = 1;
    estimated_state.vz = 0;

    a_command = prop_nav(&run_params, &estimated_state);
    REQUIRE_LT(a_command.x, 0);
    REQUIRE_LT(a_command.y, 0);
    REQUIRE_EQ(a_command.z, 0);

}