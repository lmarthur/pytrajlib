from setuptools import setup
setup(
    name="pytrajlib",
    version="0.1.0",
    cffi_modules=["src/pytrajlib/build.py:ffibuilder"],
)