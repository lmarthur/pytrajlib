# This script runs the sensitivity analysis.
#!/bin/bash

uv run python ./src/custom_scripts/sensitivity_icbm.py

uv run python ./src/custom_scripts/sensitivity_ins.py

uv run python ./src/custom_scripts/sensitivity_ins_gnss.py