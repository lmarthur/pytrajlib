import glob
import platform
import shutil

from cffi import FFI

include_dirs = ["src/include"]

ffibuilder = FFI()
ffibuilder.cdef(
    """
    extern "Python" void update_loading_bar(int, int);
    void loading_bar_callback(int x, int y);

    typedef struct runparams {
        char *run_name;
        int run_type;
        char *output_path;
        char *trajectory_path;
        char *atm_path;
        char *mean_atm_path;
        int num_runs;
        double time_step_main;
        double time_step_reentry;
        int traj_output;
        double x_aim;
        double y_aim;
        double z_aim;
        double theta_long;
        double theta_lat;

        int grav_error;
        int atm_model;
        int gnss_nav;
        int ins_nav;
        int rv_maneuv;
        double reentry_vel;
        int perfect_boost;
        double t_des_final;
        double t_vert_boost;

        int rv_type;
        double deflection_time;
        double actuator_force;
        double gearing_ratio;
        double nav_gain;
        double flap_gain;

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
    } runparams;

    typedef struct cartvec {
        double x;
        double y;
        double z;
    } cartvec;

    typedef struct anglevec {
        double lat;
        double lon;
    } anglevec;

    typedef struct state {
        double t;
        cartvec position;
        cartvec velocity;
        cartvec a_drag;
        cartvec a_lift;
        cartvec a_lift_avail;
        cartvec a_thrust;
        cartvec a_total;
        double initial_theta_long_pert;
        double initial_theta_lat_pert;
        double theta_long;
        double theta_lat;
        double roll;
        anglevec gyro_error;
    } state;

    typedef struct impact_data {
        state impact_states[1000];
    } impact_data;

    impact_data mc_run(runparams run_params);
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

#include "trajectory.h"
""",
    include_dirs=include_dirs,
    sources=[
        "src/include/rng/mt19937-64/mt19937-64.c",
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
