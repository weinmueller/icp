cmake_minimum_required(VERSION 3.10)
project(icp CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

add_subdirectory(src)

add_subdirectory(examples)

enable_testing()
add_subdirectory(tests)
