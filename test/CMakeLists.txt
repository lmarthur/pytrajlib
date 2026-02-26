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
)

target_include_directories(${PROJECT_NAME} PRIVATE ../src/include)

target_link_libraries(
    PyTraj_test
    Tau
    m
)

# Set working directory for tests to the project root
add_test(
    NAME ${PROJECT_NAME}
    COMMAND ${PROJECT_NAME}
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/.."
)