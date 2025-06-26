#!/bin/bash
echo "Compiling the program with Emscripten..."

# You need to have downloaded and installed Emscripten SDK
# source ../emsdk/emsdk_env.sh

echo "Compiling the shared library to WebAssembly..."

emcc src/main.c \
    src/include/rng/mt19937-64/mt19937-64.c \
    src/include/optimize/nrutil.c \
    -Iinclude \
    -Iinclude/rng/mt19937-64 \
    -Iinclude/optimize \
    -s EXPORTED_FUNCTIONS='["_mc_run_wrapper", "_test", "_malloc", "_free"]' \
    -s EXPORTED_RUNTIME_METHODS='["ccall", "getValue", "setValue"]' \
    -s STACK_SIZE=1024000 \
    -sASYNCIFY=1 \
    -s ALLOW_MEMORY_GROWTH=1 \
    -s WASM=1 \
    -s MODULARIZE=1 \
    -s EXPORT_NAME="createTrajlib" \
    -s ENVIRONMENT=web \
    -O3 \
    -o build/trajlib.js \
    --preload-file input/

cp build/trajlib.* trajmap/app/
cp build/trajlib.* trajmap/public/

echo "Done."
