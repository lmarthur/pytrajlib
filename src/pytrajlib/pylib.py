import numpy as np
from ctypes import c_char_p, c_int, c_double, Structure, CDLL
import pandas as pd


import importlib.resources

with importlib.resources.path("pytrajlib", "libPyTraj.so") as so_path:
    pytraj = CDLL(str(so_path))


# define the runparam struct
class runparams(Structure):
    _fields_ = [
        ("run_name", c_char_p),
        ("run_type", c_int),
        ("output_path", c_char_p),
        ("impact_data_path", c_char_p),
        ("trajectory_path", c_char_p),
        ("atm_profile_path", c_char_p),
        ("num_runs", c_int),
        ("time_step_main", c_double),
        ("time_step_reentry", c_double),
        ("traj_output", c_int),
        ("impact_output", c_int),
        ("x_aim", c_double),
        ("y_aim", c_double),
        ("z_aim", c_double),
        ("theta_long", c_double),
        ("theta_lat", c_double),
        ("grav_error", c_int),
        ("atm_model", c_int),
        ("atm_error", c_int),
        ("gnss_nav", c_int),
        ("ins_nav", c_int),
        ("rv_maneuv", c_int),
        ("reentry_vel", c_double),
        ("deflection_time", c_double),
        ("rv_type", c_int),  # 0 for ballistic, 1 for maneuverable
        ("initial_x_error", c_double),
        ("initial_pos_error", c_double),
        ("initial_vel_error", c_double),
        ("initial_angle_error", c_double),
        ("acc_scale_stability", c_double),
        ("gyro_bias_stability", c_double),
        ("gyro_noise", c_double),
        ("gnss_noise", c_double),
        ("cl_pert", c_double),
        ("step_acc_mag", c_double),
        ("step_acc_hgt", c_double),
        ("step_acc_dur", c_double),
    ]

    def __iter__(self):
        """
        Iterate over the fields of the runparams structure. This method allows a 
        user to, e.g., call `dict(runparams)` to get a dictionary representation 
        of the runparams.

        Yields:
            tuple: A tuple containing the (field name, the type, and its value).
        """
        for field_name, type in self._fields_:
            yield field_name, (type, getattr(self, field_name))


class cart_vector(Structure):
    _fields_ = [
        ("x", c_double),
        ("y", c_double),
        ("z", c_double),
    ]


class State(Structure):
    _fields_ = [
        ("t", c_double),
        ("x", c_double),
        ("y", c_double),
        ("z", c_double),
        ("vx", c_double),
        ("vy", c_double),
        ("vz", c_double),
        ("ax_grav", c_double),
        ("ay_grav", c_double),
        ("az_grav", c_double),
        ("ax_drag", c_double),
        ("ay_drag", c_double),
        ("az_drag", c_double),
        ("ax_lift", c_double),
        ("ay_lift", c_double),
        ("az_lift", c_double),
        ("ax_thrust", c_double),
        ("ay_thrust", c_double),
        ("az_thrust", c_double),
        ("ax_total", c_double),
        ("ay_total", c_double),
        ("az_total", c_double),
        ("initial_theta_long_pert", c_double),
        ("initial_theta_lat_pert", c_double),
        ("theta_long", c_double),
        ("theta_lat", c_double),
    ]

    def __iter__(self):
        """
        Iterate over the fields of the State structure. This method allows a user
        to, e.g., call `dict(state)` to get a dictionary representation of the state.

        Yields:
            tuple: A tuple containing the field name and its value.
        """
        for field_name, _ in self._fields_:
            yield field_name, getattr(self, field_name)


class ImpactData(Structure):
    MAX_RUNS = 1000
    _fields_ = [("impact_states", State * MAX_RUNS)]

    def to_dataframe(self):
        """
        Convert the ImpactData structure to a Pandas DataFrame.

        OUTPUTS:
        ----------
            df: pandas.DataFrame
                A DataFrame containing the impact data. Each row is a run, and
                each column is a field from the State structure.
        """
        data = []
        for i in range(ImpactData.MAX_RUNS):
            data.append(dict(self.impact_states[i]))

        df = pd.DataFrame(data)
        return df


def run_param_type(param):
    """
    Convert a run parameter to its corresponding Python type.

    INPUTS:
    ----------
        param: tuple
            The parameter to convert.

    OUTPUTS:
    ----------
        python_type: type
            The converted type.
    """
    run_params = runparams()
    run_param_dict = dict(run_params)
    c_to_python_type = {
        c_int: int,
        c_double: float,
        c_char_p: str,
    }
    python_type = c_to_python_type[run_param_dict[param][0]]
    return python_type


def to_c_type(value, c_type):
    """
    Convert a Python value to its corresponding C type.

    INPUTS:
    ----------
        value: any
            The value to convert.

    OUTPUTS:
    ----------
        c_value: ctype
            The converted value.

    """
    if c_type == c_int:
        return c_int(int(value))
    elif c_type == c_double:
        return c_double(float(value))
    elif c_type == c_char_p:
        return c_char_p(value.encode("utf-8"))
    else:
        raise ValueError(f"Unsupported C type: {c_type}")


def set_runparams(config):
    """
    Set the the runparams struct from the config.

    INPUTS:
    ----------
        config: dict
            The configuration dictionary.
    OUTPUTS:
    ----------
        run_params: runparams
            The run parameters.
    """
    run_params = runparams()
    run_param_dict = dict(run_params)
    for key, value in config.items():
        c_type = run_param_dict[key][0]
        run_params.__setattr__(key, to_c_type(value, c_type))

    return run_params


def get_cep(impact_data, run_params):
    """
    Function to calculate the circular error probable (CEP) from the impact data.

    INPUTS:
    ----------
        impact_data: numpy.ndarray
            The impact data.
        run_params: runparams
            The run parameters.
    OUTPUTS:
    ----------
        cep: double
            The circular error probable.
    """
    # get longitude and latitude of aimpoint
    aimpoint_lon = np.arctan2(run_params.y_aim, run_params.x_aim)
    aimpoint_lat = np.arctan2(
        run_params.z_aim, np.sqrt(run_params.x_aim**2 + run_params.y_aim**2)
    )

    impact_x = impact_data[:, 1]
    impact_y = impact_data[:, 2]
    impact_z = impact_data[:, 3]

    # get vector relative to aimpoint
    impact_x = impact_x - run_params.x_aim
    impact_y = impact_y - run_params.y_aim
    impact_z = impact_z - run_params.z_aim

    # convert impact data to local tangent plane coordinates
    impact_x_local = -np.sin(aimpoint_lon) * impact_x + np.cos(aimpoint_lon) * impact_y
    impact_y_local = (
        -np.sin(aimpoint_lat) * np.cos(aimpoint_lon) * impact_x
        - np.sin(aimpoint_lat) * np.sin(aimpoint_lon) * impact_y
        + np.cos(aimpoint_lat) * impact_z
    )

    # get the miss distances
    miss_distance = np.sqrt(impact_x_local**2 + impact_y_local**2)
    cep = np.percentile(miss_distance, 50)

    return cep


def update_aimpoint(run_params):
    """
    Function to update the aimpoint based on the current run parameters.

    INPUTS:
    ----------
        run_params: runparams
            The run parameters.
    OUTPUTS:
    ----------
        aimpoint: cart_vector
            The updated aimpoint.
    """
    # Set the output of update_aimpoint to be a cart_vector struct
    pytraj.update_aimpoint.restype = cart_vector
    aimpoint = pytraj.update_aimpoint(run_params, c_double(run_params.theta_long))
    run_params.x_aim = aimpoint.x
    run_params.y_aim = aimpoint.y
    run_params.z_aim = aimpoint.z

    return aimpoint


def mc_run(run_params):
    """
    Function to run the Monte Carlo simulation.

    INPUTS:
    ----------
        run_params: runparams
            The run parameters.
    OUTPUTS:
    ----------
        impact_data: pd.DataFrame
            The impact data as a Pandas DataFrame. .
    """
    # Set the output of mc_run to be an ImpactData struct
    pytraj.mc_run.restype = ImpactData
    impact_data = pytraj.mc_run(run_params)
    # Only return data for the number of runs
    impact_df = impact_data.to_dataframe().iloc[: run_params.num_runs]
    return impact_df
