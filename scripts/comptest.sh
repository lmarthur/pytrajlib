# This script compiles the program and test scripts, and runs the unit and integration tests.
#!/bin/bash
# Compile the program
echo "Compiling the program..."

rm -f ./test/build/PyTraj_test
rm -f ./src/pytrajlib/_traj.so

# Compile with CMake
cmake -S ./test -B test/build -Wno-dev
make -C ./test/build

# Run the tests
echo "Running the library tests..."
./test/build/PyTraj_test

# Compile the shared library
echo "Compiling the shared library..."
uv run src/pytrajlib/build.py

# Run integration tests
echo "Running integration tests..."
uv run pytest -v ./test/

echo "Done."