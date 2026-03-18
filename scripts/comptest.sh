#!/bin/bash
# This script compiles the program and test scripts, and runs the unit and integration tests.
set -o pipefail

fail() {
  echo "Error: $1"
  exit 1
}

# Compile the program
echo "Compiling the program..."

rm -f ./test/build/PyTraj_test
rm -f ./src/pytrajlib/*.so

# Compile with CMake
cmake -S ./test -B test/build -Wno-dev || fail "Failed to configure CMake"
make -C ./test/build || fail "Failed to compile test binary"

# Run the tests
echo "Running the library tests..."
./test/build/PyTraj_test || fail "Library tests failed"

# Compile the shared library
echo "Compiling the shared library..."

if ! uv run src/pytrajlib/build.py; then
  fail "Failed to compile shared library"
fi

echo "Running integration tests..."
uv run pytest -v || fail "Integration tests failed"

echo "Done."