import argparse
from copy import deepcopy
import importlib.resources
import os
import tomllib

import numpy as np
from scipy.optimize import minimize

from pytrajlib._traj import lib as traj
from pytrajlib.main import (
	_TEMP_DIR,
	_UNSET,
	_LOADING_BAR_DISABLED,
	_get_default_config,
	_keep_alive,
	_set_aimpoint_from_range,
	create_runparams_struct,
	impact_data_to_df,
)
from pytrajlib.utils import get_miss_distance


def optimize_kp(config_dict, maxfev=200):
	"""
	Tune proportional gain Kp for realistic RV maneuverability (rv_maneuv = 1).

	The objective is average miss distance over ``num_runs_optimizer`` Monte
	Carlo trajectories with stochastic/navigation errors disabled.
	"""
	_keep_alive["loading_bar"] = _LOADING_BAR_DISABLED
	without_error_params = deepcopy(config_dict)
	without_error_params["traj_output"] = 0
	without_error_params["num_runs"] = without_error_params["num_runs_optimizer"]
	without_error_params["initial_pos_error"] = 0
	without_error_params["initial_vel_error"] = 0
	without_error_params["initial_angle_error"] = 0
	without_error_params["acc_scale_stability"] = 0
	without_error_params["gyro_bias_stability"] = 0
	without_error_params["gyro_noise"] = 0
	without_error_params["gnss_noise"] = 0
	without_error_params["grav_error"] = 0

	without_error_params["gnss_nav"] = 0
	without_error_params["perfect_boost"] = 1
	without_error_params["rv_maneuv"] = 1

	def obj(params):
		kp = float(params[0])
		without_error_params["Kp"] = kp
		rp = create_runparams_struct(without_error_params)

		impact_df = impact_data_to_df(traj.mc_run(rp[0]), without_error_params)
		miss_dist = np.mean(
			get_miss_distance(
				impact_df=impact_df,
				aimpoint=(
					without_error_params["x_aim"],
					without_error_params["y_aim"],
					without_error_params["z_aim"],
				),
			)
		)

		_keep_alive.clear()
		_keep_alive["loading_bar"] = _LOADING_BAR_DISABLED
		print(f"{miss_dist=:.9f} (avg), {kp=:.9f}")
		return miss_dist

	result = minimize(
		obj,
		x0=(float(without_error_params["Kp"]),),
		method="Nelder-Mead",
		bounds=[(1e-6, 1e3)],
		options=dict(maxfev=maxfev),
	)
	print(result)
	return float(result.x[0])


def _load_optimizer_config(config_path=None):
	if config_path is not None:
		with open(config_path, "rb") as f:
			loaded = tomllib.load(f)
		config_dict = {
			**loaded.get("RUN", {}),
			**loaded.get("FLIGHT", {}),
			**loaded.get("VEHICLE", {}),
			**loaded.get("ERRORPARAMS", {}),
		}
	else:
		config_dict = _get_default_config().copy()

	atm_path = str(
		importlib.resources.files("pytrajlib.config").joinpath("atmprofiles.txt")
	)
	mean_atm_path = str(
		importlib.resources.files("pytrajlib.config").joinpath("mean_atm.txt")
	)
	config_dict["atm_path"] = atm_path
	config_dict["mean_atm_path"] = mean_atm_path
	config_dict["trajectory_path"] = os.path.join(_TEMP_DIR, "trajectory.txt")
	_set_aimpoint_from_range(config_dict)

	return config_dict


def cli():
	parser = argparse.ArgumentParser(
		description="Optimize Kp for rv_maneuv=1",
		formatter_class=argparse.RawDescriptionHelpFormatter,
	)
	parser.add_argument(
		"--config", type=str, default=None, help="Path to TOML config file"
	)
	parser.add_argument(
		"--maxfev",
		type=int,
		default=200,
		help="Maximum number of objective evaluations for Nelder-Mead",
	)

	for param_name, default_value in _get_default_config().items():
		parser.add_argument(
			f"--{param_name.replace('_', '-')}",
			type=type(default_value),
			default=_UNSET,
			help=f"Default: {default_value}",
		)

	args = parser.parse_args()
	kwargs = vars(args)
	config = kwargs.pop("config")
	maxfev = kwargs.pop("maxfev")

	config_dict = _load_optimizer_config(config)

	explicit_kwargs = {k: v for k, v in kwargs.items() if v is not _UNSET}
	if explicit_kwargs:
		config_dict.update(explicit_kwargs)

	optimized_kp = optimize_kp(config_dict, maxfev=maxfev)
	print(f"optimized Kp: {optimized_kp}")


if __name__ == "__main__":
	cli()

