from setuptools import setup, find_packages
setup(
    name="pytrajlib",
    version="0.1.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    cffi_modules=["src/pytrajlib/build.py:ffibuilder"],
)