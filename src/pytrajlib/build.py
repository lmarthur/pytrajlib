import glob
import platform
import shutil

from cffi import FFI

include_dirs = ["src/include"]

ffibuilder = FFI()
ffibuilder.cdef(
    """
    extern "Python" void _update_loading_bar(int);

    typedef struct runparams {
        char *run_name;
        char *output_path;
        char *trajectory_path;
        char *atm_path;
        char *mean_atm_path;
        int num_runs;
        int num_runs_optimizer;
        double time_step_boost;
        double time_step_lambert;
        double time_step_midcourse;
        double time_step_reentry;
        int traj_output;
        double range;
        double x_aim;
        double y_aim;
        double z_aim;
        double theta_long;
        double theta_lat;
        int integrator;
        long random_seed;

        int grav_error;
        int ballistic_drag;
        int atm_model;
        int gnss_nav;
        int rv_maneuv;
        double reentry_vel;
        int perfect_boost;
        int optimize_boost;
        int optimize_maneuv;
        double t_des_final;
        double t_vert_boost;

        int rv_type;
        double deflection_time;
        double actuator_force;
        double gearing_ratio;
        double actuator_resolution;
        double max_deflection_angle;
        double nav_gain_0;
        double nav_gain_1;
        double tau_deflect;
        double K_q;
        double K_pp;


        double initial_x_error;
        double initial_pos_error;
        double initial_vel_error;
        double initial_angle_error;
        double acc_scale_stability;
        double gyro_bias_stability;
        double gyro_noise;
        double gnss_noise;
        double gnss_freq;
        double cl_pert;
        double step_acc_mag;
        double step_acc_hgt;
        double step_acc_dur;

        double rv_mass;
        double rv_length;
        double rv_radius;
        double rv_c_d_0;
        double rv_c_d_alpha;
        double rv_c_m_delta;

        double booster_area;
        double booster_maxdiam;
        double booster_c_d_0;
        double booster_bus_mass;
    } runparams;

    typedef struct cartvec {
        double x;
        double y;
        double z;
    } cartvec;

    typedef struct anglevec {
        double pitch;
        double yaw;
    } anglevec;

    typedef struct quaternion {
        double w;
        double x;
        double y;
        double z;
    } quaternion;

    typedef struct state {
        cartvec position;
        cartvec velocity;
        quaternion q_EB;
        cartvec angular_vel_B;
        double theta_long;
        double theta_lat;
        cartvec orientation_angle_change;
        double delta_1;
        double delta_2;
    } state;

    typedef struct impact_data {
        state impact_states[1000];
        double impact_times[1000];
        double burnout_speed[1000];
        double burnout_altitude[1000];
        double burnout_angle[1000];
        double apogee[1000];
        double reentry_speed[1000];
        double reentry_angle[1000];
    } impact_data;

    impact_data mc_run(runparams run_params);
    """
)

# Make _traj part of the package so it is included in the wheel
module_name = "_traj" if __name__ == "__main__" else "pytrajlib._traj"
ffibuilder.set_source(
    module_name,
    """
static void _update_loading_bar(int);

void update_loading_bar(int x) {
    _update_loading_bar(x);
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
