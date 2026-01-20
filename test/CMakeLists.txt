cmake_minimum_required(VERSION 3.20)
project(PyTraj_test)

# Tau requires at least C11/C++11
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 11)

include(FetchContent)
FetchContent_Declare(
    Tau
    URL https://github.com/jasmcaus/tau/archive/dev.zip
)

FetchContent_MakeAvailable(Tau)

enable_testing()

add_executable(${PROJECT_NAME}
    main_test.c
    ../src/include/rng/mt19937-64/mt19937-64.c
    # ../src/include/optimize/nrutil.c
)

target_link_libraries(
    PyTraj_test
    Tau
    m
)