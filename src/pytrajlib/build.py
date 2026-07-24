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
        int num_trials_optimizer;
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
        int perfect_boost;
        int optimize_boost;
        int optimize_reentry;

        double t_des_final;
        double lambert_v_offset;
        double t_vert_boost;
        double deflection_time;
        double actuator_force;
        double gearing_ratio;
        double actuator_resolution;
        double max_deflection_angle;
        double nav_gain_0;
        double nav_gain_1;
        double K_q;
        double K_pp;
        double K_delta_p;
        double K_delta_d;

        double initial_x_error;
        double initial_pos_error;
        double initial_vel_error;
        double initial_angle_error;
        double acc_scale_stability;
        double gyro_bias_stability;
        double gyro_noise;
        double gnss_noise;
        double gnss_freq;
        double roll_gyro_error_factor;
        double geoid_height_error;

        double burn_time_error;
    } runparams;

    typedef struct booster {
        char name[32];
        int num_stages;
        double area;
        double total_burn_time;
        double bus_mass;
        double total_mass;
        double c_d_0;
        double wet_mass[10];
        double fuel_mass[10];
        double dry_mass[10];
        double isp0[10];
        double burn_time[10];
        double fuel_burn_rate[10];
    } booster;

    typedef struct rv {
        char name[32];
        int maneuverability_flag;
        double rv_mass;
        double rv_length;
        double rv_radius;
        double half_angle;
        double rv_area;
        double c_d_0;
        double c_d_alpha;
        double c_m_alpha;
        double c_m_q;
        double c_m_delta;
        double c_l_alpha;
        double flap_area;
        double x_flap;
        double x_com;
        double Iyy;
        int aero_table_size;
        double aero_alpha_deg_table[51];
        double c_d_table[51];
        double c_l_table[51];
        double c_m_table[51];
        double c_m_q_table[51];
    } rv;

    typedef struct vehicle {
        booster booster;
        rv rv;
        double total_mass;
    } vehicle;

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
        double dot_delta_1;
        double dot_delta_2;
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

    impact_data mc_run(runparams run_params, vehicle vehicle);
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
