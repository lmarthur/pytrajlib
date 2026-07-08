# pytrajlib 
## Python developer documentation

Note: These are instructions for compiling the project from scratch. If you just 
want to run pytrajlib, the easiest way is to install it using pip:
```bash
pip install pytrajlib
```

### Build C library and run Python script
We use `uv` to manage dependencies and build the package. [Install `uv`](https://docs.astral.sh/uv/#installation).

Dependencies and settings for building the Python package are set in [pyproject.toml](../../pyproject.toml). You don't need to change these.

[maneuv.json](./config/maneuv.json) defines the default simulation parameters.

From the project directory, run 

```bash
source scripts/compile.sh
```

>Note: If you are on Linux, you might need to install the Python developer package (`python-dev` or `python-devel`) to have the Python header files on your machine. 

Now pytrajlib should be built:
```bash
uv run pytrajlib --version
```


## Basic usage

After you have installed it, you can see all available command line arguments with 

```bash
uv run pytrajlib --help
```

To run 200 simulation runs (the default) run 

```bash
uv run pytrajlib
```

If you want only 10 runs (faster), run 

```bash 
uv run pytrajlib --num-runs 10
```