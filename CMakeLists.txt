cmake_minimum_required(VERSION 2.8)
project( eigen_9_1 )
find_package( Eigen3 REQUIRED )
add_executable( eigen_9_1 eigen_9_1.cpp )
include_directories(${EIGEN3_INCLUDE_DIR})
target_link_libraries( eigen_9_1 )
