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

    #define MAX_RUNS 1000

    typedef struct {
        double x;
        double y;
        double z;

    } cartvec;

    typedef struct {
        double lat;
        double lon;
    } anglevec;

    typedef struct {
        double w;
        double x;
        double y;
        double z;
    } quaternion;

    typedef struct {
        cartvec position;
        cartvec velocity;
        cartvec a_lift;
        cartvec a_lift_avail;
        anglevec gyro_error;
        quaternion quaternion;
    } state;

    typedef struct {
        state impact_event;
        double t;
    } integration_result;

    typedef struct {
        integration_result results[MAX_RUNS];
    } integration_results;

    integration_results fly(int N);
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
#include "run.h"
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
