# This script compiles the program and test scripts, and runs the unit and integration tests.
#!/bin/bash
# Compile the program
echo "Compiling the program..."

# mamba activate pytraj_env

rm ./test/build/PyTraj_test
rm ./build/libPyTraj.so

# Compile with CMake
cmake -S ./test -B test/build -Wno-dev
make -C ./test/build

# Run the tests
echo "Running the library tests..."
./test/build/PyTraj_test

# Compile the shared library
echo "Compiling the shared library..."
gcc -shared -fPIC -o ./build/libPyTraj.so ./src/main.c

# Run integration tests
echo "Running integration tests..."
uv run pytest -v -s ./test/

echo "Done."