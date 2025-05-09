# This script compiles the program and test scripts, and runs the compile-time tests.
#!/bin/bash
# Compile the program
echo "Compiling the program..."

# mamba activate pytraj_env

rm -f ./test/build/PyTraj_test
rm -f ./build/libPyTraj.so
rm -f ./src/pytrajlib/libPyTraj.so

# Compile with CMake
cmake -S ./test -B test/build -Wno-dev
make -C ./test/build

# Run the tests
echo "Running the library tests..."
./test/build/PyTraj_test

# Compile the shared library
echo "Compiling the shared library..."
gcc -shared -fPIC -o ./build/libPyTraj.so ./src/main.c

# Copy the .so file to the pytrajlib directory
cp ./build/libPyTraj.so ./src/pytrajlib/libPyTraj.so

echo "Done."