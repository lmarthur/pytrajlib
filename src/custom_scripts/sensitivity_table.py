"""Build default-point sensitivity tables from the recorded sweep outputs.

For each run, the sensitivity CSV is written in blocks of seven rows per
parameter sweep, using the log-spaced grid from 0.1x to 10x the configured
default. The row we want for each parameter is the middle row in its block,
where the varied parameter is back at its configured default and every other
parameter is zero. The final block is the combined total row.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pandas as pd

sys.path.append(".")
from src.pylib import read_config


GRID_SIZE = 7
DEFAULT_GRID_INDEX = GRID_SIZE // 2
PARAMETER_ORDER = [
	("initial_pos_error", "Initial Position Error"),
	("initial_vel_error", "Initial Velocity Error"),
	("initial_angle_error", "Initial Angle Error"),
	("acc_scale_stability", "Acceleration Scale Stability"),
	("gyro_bias_stability", "Gyro Bias Stability"),
	("gyro_noise", "Gyro Noise"),
	("gnss_noise", "GNSS Noise"),
]


def build_table(run_name: str) -> pd.DataFrame:
	"""Return the default-point CEP table for one run."""

	config_path = Path("./input") / f"{run_name}.toml"
	output_dir = Path("./output") / run_name
	sensitivity_path = output_dir / "sensitivity_data.csv"

	if not config_path.is_file():
		raise FileNotFoundError(f"Missing configuration file: {config_path}")
	if not sensitivity_path.is_file():
		raise FileNotFoundError(f"Missing sensitivity data file: {sensitivity_path}")

	run_params = read_config(run_name)
	sensitivity_data = pd.read_csv(sensitivity_path)

	if len(sensitivity_data) % GRID_SIZE != 0:
		raise ValueError(
			f"Unexpected row count in {sensitivity_path}: {len(sensitivity_data)}"
		)

	parameter_names = [name for name, _ in PARAMETER_ORDER]
	if not run_params.gnss_nav:
		parameter_names = [name for name in parameter_names if name != "gnss_noise"]

	rows = []
	for index, parameter_name in enumerate(parameter_names):
		row_index = index * GRID_SIZE + DEFAULT_GRID_INDEX
		row = sensitivity_data.iloc[row_index]
		rows.append(
			{
				"parameter": parameter_name,
				"default_value": float(getattr(run_params, parameter_name)),
				"cep": row["cep"],
			}
		)

	total_row_index = len(parameter_names) * GRID_SIZE + DEFAULT_GRID_INDEX
	if total_row_index >= len(sensitivity_data):
		raise ValueError(
			f"Missing total row in {sensitivity_path}: expected index {total_row_index}"
		)
	total_row = sensitivity_data.iloc[total_row_index]
	rows.append({"parameter": "total", "default_value": None, "cep": total_row["cep"]})

	return pd.DataFrame(rows)


def main() -> None:
	runs = ["run_0", "run_2", "run_3"]

	for run_name in runs:
		table = build_table(run_name)
		output_path = Path("./output") / run_name / "sensitivity_table.csv"
		table.to_csv(output_path, index=False, float_format="%.15g", na_rep="None")

		print(f"\n{run_name}")
		print(table.to_string(index=False))


if __name__ == "__main__":
	main()
