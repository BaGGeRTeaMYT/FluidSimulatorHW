cmake_minimum_required(VERSION 3.20)

project(Simulator)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_CXX_COMPILER "g++")

include_directories(include/)

add_executable(TEST main.cpp
                    include/Fixed.hpp
                    include/Simulator.hpp)       