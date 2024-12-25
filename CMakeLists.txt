cmake_minimum_required(VERSION 3.20)

project(Simulator)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_CXX_COMPILER "g++")

find_package(OpenMP REQUIRED)

include_directories(include/)

add_executable(HW2 main.cpp
                    include/Fixed.hpp
                    include/Simulator.hpp)

enable_testing()

add_test(NAME TEST1
    COMMAND HW2 --p-type=FLOAT --v-type=DOUBLE --v-flow-type=DOUBLE
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

set_tests_properties(TEST1 PROPERTIES LABELS "G1")

add_test(NAME TEST2
    COMMAND HW2 --p-type=FIXED\(32,8\) --v-type=DOUBLE --v-flow-type=DOUBLE
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

set_tests_properties(TEST2 PROPERTIES LABELS "G1")

# should fail, because --p-type is not specified
add_test(NAME TEST3
    COMMAND HW2 --v-type=DOUBLE --v-flow-type=DOUBLE
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

set_tests_properties(TEST3 PROPERTIES LABELS "G1")

# should fail, because FIXED(32, 9) is not defined 
add_test(NAME TEST4
    COMMAND HW2 --p-type=FIXED\(32,9\) --v-type=DOUBLE --v-flow-type=DOUBLE
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

set_tests_properties(TEST4 PROPERTIES LABELS "G1")


add_executable(HW3 second_main.cpp
                   include/BetterSimulator.hpp
                   include/Fixed.hpp)

target_link_libraries(HW3 PRIVATE OpenMP::OpenMP_CXX)
target_compile_options(HW3 PRIVATE -fopenmp)