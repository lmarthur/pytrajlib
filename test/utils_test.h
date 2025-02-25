#include <tau/tau.h>
#include "../src/include/utils.h"

TEST(utils, cartcoords_to_sphercoords){
    double cart_coords[3] = {1, 0, 0};
    double spher_coords[3];

    cartcoords_to_sphercoords(cart_coords, spher_coords);

    REQUIRE_EQ(spher_coords[0], 1);
    REQUIRE_EQ(spher_coords[1], 0);
    REQUIRE_EQ(spher_coords[2], 0);

    cart_coords[0] = 0;
    cart_coords[1] = 1;
    cart_coords[2] = 0;

    cartcoords_to_sphercoords(cart_coords, spher_coords);

    REQUIRE_EQ(spher_coords[0], 1);
    REQUIRE_EQ(spher_coords[1], M_PI/2);
    REQUIRE_EQ(spher_coords[2], 0);

    cart_coords[0] = 0;
    cart_coords[1] = 0;
    cart_coords[2] = 1;

    cartcoords_to_sphercoords(cart_coords, spher_coords);

    REQUIRE_EQ(spher_coords[0], 1);
    REQUIRE_EQ(spher_coords[1], 0);
    REQUIRE_EQ(spher_coords[2], M_PI/2);

    cart_coords[0] = 1;
    cart_coords[1] = 1;
    cart_coords[2] = 0;

    cartcoords_to_sphercoords(cart_coords, spher_coords);

    REQUIRE_EQ(spher_coords[0], sqrt(2));
    REQUIRE_EQ(spher_coords[1], M_PI/4);
    REQUIRE_EQ(spher_coords[2], 0);

    cart_coords[0] = 1;
    cart_coords[1] = 0;
    cart_coords[2] = 1;

    cartcoords_to_sphercoords(cart_coords, spher_coords);

    REQUIRE_EQ(spher_coords[0], sqrt(2));
    REQUIRE_EQ(spher_coords[1], 0);
    REQUIRE_LT(fabs(spher_coords[2] - M_PI/4), 1e-6);

    cart_coords[0] = 0;
    cart_coords[1] = 1;
    cart_coords[2] = 1;

    cartcoords_to_sphercoords(cart_coords, spher_coords);

    REQUIRE_EQ(spher_coords[0], sqrt(2));
    REQUIRE_EQ(spher_coords[1], M_PI/2);
    REQUIRE_LT(fabs(spher_coords[2] - M_PI/4), 1e-6);

    cart_coords[0] = 1;
    cart_coords[1] = 1;
    cart_coords[2] = 1;

    cartcoords_to_sphercoords(cart_coords, spher_coords);

    REQUIRE_EQ(spher_coords[0], sqrt(3));
    REQUIRE_EQ(spher_coords[1], M_PI/4);
    REQUIRE_EQ(spher_coords[2], atan(1/sqrt(2)));

}

TEST(utils, sphercoords_to_cartcoords){
    double spher_coords[3] = {1, 0, 0};
    double cart_coords[3];

    sphercoords_to_cartcoords(spher_coords, cart_coords);

    REQUIRE_EQ(cart_coords[0], 1);
    REQUIRE_EQ(cart_coords[1], 0);
    REQUIRE_EQ(cart_coords[2], 0);

    spher_coords[0] = 1;
    spher_coords[1] = M_PI/2;
    spher_coords[2] = 0;

    sphercoords_to_cartcoords(spher_coords, cart_coords);

    REQUIRE_LT(fabs(cart_coords[0] - 0), 1e-6);
    REQUIRE_EQ(cart_coords[1], 1);
    REQUIRE_EQ(cart_coords[2], 0);

    spher_coords[0] = 1;
    spher_coords[1] = 0;
    spher_coords[2] = M_PI/2;

    sphercoords_to_cartcoords(spher_coords, cart_coords);

    REQUIRE_LT(fabs(cart_coords[0] - 0), 1e-6);
    REQUIRE_EQ(cart_coords[1], 0);
    REQUIRE_EQ(cart_coords[2], 1);

    spher_coords[0] = sqrt(2);
    spher_coords[1] = M_PI/4;
    spher_coords[2] = 0;

    sphercoords_to_cartcoords(spher_coords, cart_coords);

    REQUIRE_LT(fabs(cart_coords[0] - 1), 1e-6);
    REQUIRE_EQ(cart_coords[1], 1);
    REQUIRE_EQ(cart_coords[2], 0);

    spher_coords[0] = sqrt(2);
    spher_coords[1] = 0;
    spher_coords[2] = M_PI/4;

    sphercoords_to_cartcoords(spher_coords, cart_coords);

    REQUIRE_LT(fabs(cart_coords[0] - 1), 1e-6);
    REQUIRE_EQ(cart_coords[1], 0);
    REQUIRE_EQ(cart_coords[2], 1);

    spher_coords[0] = sqrt(2);
    spher_coords[1] = 0;
    spher_coords[2] = M_PI/4;

    sphercoords_to_cartcoords(spher_coords, cart_coords);

    REQUIRE_LT(fabs(cart_coords[0] - 1), 1e-6);
    REQUIRE_EQ(cart_coords[1], 0);
    REQUIRE_EQ(cart_coords[2], 1);
}

TEST(utils, sphervec_to_cartvec){
    double spher_coords[3] = {1, 0, 0};
    double sphervec[3] = {1, 0, 0};
    double cartvec[3];

    sphervec_to_cartvec(sphervec, cartvec, spher_coords);

    REQUIRE_EQ(cartvec[0], 1);
    REQUIRE_EQ(cartvec[1], 0);
    REQUIRE_EQ(cartvec[2], 0);

    spher_coords[0] = 1;
    spher_coords[1] = M_PI/2;
    spher_coords[2] = 0;

    sphervec_to_cartvec(sphervec, cartvec, spher_coords);

    REQUIRE_LT(fabs(cartvec[0] - 0), 1e-6);
    REQUIRE_EQ(cartvec[1], 1);
    REQUIRE_EQ(cartvec[2], 0);

    spher_coords[0] = 1;
    spher_coords[1] = 0;
    spher_coords[2] = M_PI/2;

    sphervec_to_cartvec(sphervec, cartvec, spher_coords);

    REQUIRE_LT(fabs(cartvec[0] - 0), 1e-6);
    REQUIRE_EQ(cartvec[1], 0);
    REQUIRE_EQ(cartvec[2], 1);
}

TEST(utils, get_altitude){
    REQUIRE_EQ(get_altitude(6371e3, 0, 0), 0);
    REQUIRE_EQ(get_altitude(0, 6371e3, 0), 0);
    REQUIRE_EQ(get_altitude(0, 0, 6371e3), 0);
    REQUIRE_EQ(get_altitude(1, 0, 0), 1 - 6371e3);
    REQUIRE_EQ(get_altitude(0, 1, 0), 1 - 6371e3);
    REQUIRE_EQ(get_altitude(0, 0, 1), 1 - 6371e3);
    REQUIRE_EQ(get_altitude(1, 1, 0), sqrt(2) - 6371e3);
    REQUIRE_EQ(get_altitude(1, 0, 1), sqrt(2) - 6371e3);
    REQUIRE_EQ(get_altitude(0, 1, 1), sqrt(2) - 6371e3);
    REQUIRE_EQ(get_altitude(1, 1, 1), sqrt(3) - 6371e3);
}


