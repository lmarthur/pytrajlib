from setuptools import setup
print("running setup.py")
setup(
    name="pytrajlib",
    version="0.1.0",
    cffi_modules=["src/pytrajlib/build.py:ffibuilder"],
)