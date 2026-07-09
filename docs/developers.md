# For developers

!!! info

    These are instructions for compiling the project from scratch. If you just 
    want to run pytrajlib, the easiest way is to install it using pip:
    ```bash 
    pip install pytrajlib
    ```

## Build C library and run Python script
We use `uv` to manage dependencies and build the package. [Install `uv`](https://docs.astral.sh/uv/#installation).

Dependencies and settings for building the Python package are set in `pyproject.toml`. You don't need to change these.

`maneuv.json` defines the default simulation parameters.

From the project directory, run 

```bash
source scripts/compile.sh
```

!!! tip
    If you are on Linux, you might need to install the Python developer package (`python-dev` or `python-devel`) to have the Python header files on your machine. 

Test everything worked correctly:
```bash
uv run pytrajlib --version
```