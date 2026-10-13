cmake_minimum_required(VERSION 3.20)
project(itpms LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# The detector is the "embedded" part: strict warnings, no exceptions/RTTI,
# and float-only math (-Wdouble-promotion catches accidental double use,
# which would be software-emulated on a Cortex-M4F).
add_library(itpms_core STATIC core/src/detector.cpp)
target_include_directories(itpms_core PUBLIC core/include)
target_compile_options(itpms_core PRIVATE
  -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wdouble-promotion
  -Wshadow -Wcast-align -Wold-style-cast -Werror
  -fno-exceptions -fno-rtti)

add_library(itpms_sim STATIC sim/vehicle_sim.cpp sim/harness.cpp)
target_include_directories(itpms_sim PUBLIC sim)
target_link_libraries(itpms_sim PUBLIC itpms_core)
target_compile_options(itpms_sim PRIVATE -Wall -Wextra)

add_executable(itpms_cli tools/cli.cpp)
target_link_libraries(itpms_cli PRIVATE itpms_sim)

add_executable(itpms_tests tests/tests.cpp)
target_link_libraries(itpms_tests PRIVATE itpms_sim)

add_executable(itpms_bench bench/bench.cpp)
target_link_libraries(itpms_bench PRIVATE itpms_sim)

enable_testing()
add_test(NAME itpms_tests COMMAND itpms_tests)

find_package(Threads REQUIRED)
add_executable(itpms_accuracy tools/accuracy.cpp)
target_link_libraries(itpms_accuracy PRIVATE itpms_sim Threads::Threads)
