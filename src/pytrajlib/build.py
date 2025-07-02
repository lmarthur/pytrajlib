import glob
import platform
import shutil

from cffi import FFI

include_dirs = ["src/"]
ffibuilder = FFI()
ffibuilder.cdef(
    """
    extern "Python" void update_loading_bar(int, int);
    void loading_bar_callback(int x, int y);

    struct runparams {
        char *run_name; 
        int run_type; 
        char *impact_data_path; 
        char *trajectory_path; 
        char *atm_profile_path; 
        int num_runs; 
        double time_step_main; 
        double time_step_reentry; 
        int traj_output; 
        int impact_output; 
        double x_launch; 
        double y_launch; 
        double z_launch;
        double x_aim; 
        double y_aim; 
        double z_aim; 
        double theta_long; 
        double theta_lat; 
        double east;
        double north;
        double up;

        int grav_error; 
        int atm_model; 
        int atm_error; 
        int gnss_nav; 
        int ins_nav; 
        int rv_maneuv; 
        double reentry_vel; 
        double deflection_time; 

        int booster_type;
        int rv_type; 

        double initial_x_error; 
        double initial_pos_error; 
        double initial_vel_error; 
        double initial_angle_error; 
        double acc_scale_stability; 
        double gyro_bias_stability; 
        double gyro_noise; 
        double gnss_noise; 
        double cl_pert; 
        double step_acc_mag; 
        double step_acc_hgt; 
        double step_acc_dur; 
    };
    struct cart_vector{
        double x;
        double y;
        double z;
    };
    struct state{
        double t; 
        double x; 
        double y; 
        double z; 
        double vx; 
        double vy; 
        double vz; 
        double ax_grav; 
        double ay_grav; 
        double az_grav; 
        double ax_drag; 
        double ay_drag; 
        double az_drag; 
        double ax_lift; 
        double ay_lift; 
        double az_lift; 
        double ax_thrust; 
        double ay_thrust; 
        double az_thrust; 
        double ax_total; 
        double ay_total; 
        double az_total; 
        double initial_theta_long_pert; 
        double initial_theta_lat_pert; 
        double theta_long; 
        double theta_lat; 
        double east;
        double north;
        double up;
    };
    #define MAX_RUNS 1000
    struct impact_data{
        struct state impact_states[MAX_RUNS];
    };
    struct cart_vector update_aimpoint(struct runparams run_params);
    struct impact_data mc_run(struct runparams run_params);
    """
)

# Make _traj part of the package so it is included in the wheel
module_name = "_traj" if __name__ == "__main__" else "pytrajlib._traj"
ffibuilder.set_source(
    module_name,
    """
    static void update_loading_bar(int, int);

    void loading_bar_callback(int x, int y) {
        update_loading_bar(x, y);
    }

    #include "include/rng/mt19937-64/mt64.h"
    #include "include/rng/rng.h"

    #include "include/optimize/brent.h"
    #include "include/optimize/mnbrak.h"
    #include "include/optimize/nrutil.h"

    #include "include/utils.h"
    #include "include/vehicle.h"
    #include "include/gravity.h"
    #include "include/atmosphere.h"
    #include "include/physics.h"
    #include "include/trajectory.h"
""",
    include_dirs=include_dirs,
    sources=[
        "src/include/rng/mt19937-64/mt19937-64.c",
        "src/include/optimize/nrutil.c",
    ],
    extra_compile_args=["-DFROM_PYTHON"],
)

if __name__ == "__main__":
    ffibuilder.compile(verbose=True)
    # If Windows, move .pyd to project dir so it can be found by pytrajlib
    if platform.system() == "Windows":
        file_to_move = glob.glob("_traj*.pyd")[0]
        shutil.move(file_to_move, "src/pytrajlib/_traj.pyd")
    # If Linux or Mac, move .so
    else:
        file_to_move = glob.glob("_traj*.so")[0]
        shutil.move(file_to_move, "src/pytrajlib/_traj.so")
