cmake_minimum_required(VERSION 3.14 FATAL_ERROR)
project(motion3d_example)

find_package(motion3d REQUIRED)

add_executable(motion3d_example main.cpp)
target_link_libraries(motion3d_example motion3d::motion3d)
