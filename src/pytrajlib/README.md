# pytrajlib 
## Python developer documentation

There are three ways to install pytrajlib. 1) by building the C shared object library and then running a Python script or 2) with a local pip installation from source 3) with a pre-built wheel. 

After you have installed it, you can see all available command line arguments with 

```bash
uv run pytrajlib --help
```

To run 1000 simulation runs (the default) run 

```bash
uv run pytrajlib
```

If you want only 10 runs (faster), run 

```bash 
uv run pytrajlib --num-runs 10
```


### 1. Build C library and run Python script
We use `uv` to manage dependencies and build the package. [Install `uv`](https://docs.astral.sh/uv/#installation).

Dependencies and settings for building the Python package are set in [pyproject.toml](../../pyproject.toml). 

[default.toml](./config/default.toml) defines the default simulation parameters.

From the project directory, run 

```bash
source scripts/compile.sh
```

to create the shared object library (e.g. `_traj.so`, for Linux). Now we can import C code with Python using `from ._traj import lib as traj`. 

>Note: If you are on Linux, you might need to install the Python developer package (`python-dev` or `python-devel`) to have the Python header files on your machine. 


### 2. Local pip installation from source
To install the pip package locally, we recommend creating a virtual environment. 

```bash
python -m venv venv
source venv/bin/activate

# Windows
.venv\Scripts\activate 
```

Inside the virtual environment, pip install the local project

```bash
pip install .
```

Now you can run the `pytrajlib` command line tool

```bash
pytrajlib --num-runs 10
```

To exit the virtual environment, run

```bash
deactivate
```

### 3. pip installation from wheel
Grab the `.whl` file appropriate for your OS and Python version from pip or
create a wheel for your machine by running 

```bash
uv build
```

which creates `.tar.gz` (source distribution) and `.whl` (build distribution) files.

(Optional, but recommended) create and enter a virtual environment.
```bash
python -m venv venv
source venv/bin/activate

# Windows
.venv\Scripts\activate 
```

Inside the virtual environment, pip install the wheel:

```bash
pip install <name of wheel>.whl
```