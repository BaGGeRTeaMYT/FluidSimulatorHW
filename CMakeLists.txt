cmake_minimum_required(VERSION 3.20)

project(Simulator)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_CXX_COMPILER "g++")

include_directories(include/)

add_executable(TEST main.cpp
                    include/Fixed.hpp
                    include/Simulator.hpp)
                    
# target_compile_definitions(TEST PRIVATE TYPES=FLOAT,DOUBLE,FIXED(32,8),FAST_FIXED(64,12))
# target_compile_definitions(TEST PRIVATE SIZES=S(8,5))

enable_testing()

# Добавляем тест для запуска программы с аргументами
add_test(NAME TEST1
    COMMAND TEST --p-type=FLOAT --v-type=DOUBLE --v-flow-type=DOUBLE
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

set_tests_properties(TEST1 PROPERTIES LABELS "G1")

add_test(NAME TEST2
    COMMAND TEST --p-type=FIXED\(32,8\) --v-type=DOUBLE --v-flow-type=DOUBLE
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

set_tests_properties(TEST2 PROPERTIES LABELS "G1")

# should fail, because --p-type is not specified
add_test(NAME TEST3
    COMMAND TEST --v-type=DOUBLE --v-flow-type=DOUBLE
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

# should fail, because FIXED(32, 9) is not defined 
add_test(NAME TEST4
    COMMAND TEST --p-type=FIXED\(32,9\) --v-type=DOUBLE --v-flow-type=DOUBLE
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

set_tests_properties(TEST3 PROPERTIES LABELS "G1")