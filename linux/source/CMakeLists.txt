cmake_minimum_required(VERSION 3.4.1)

add_compile_options(-fPIC)
add_definitions(-std=c++11)

set(CMAKE_CXX_FLAGS "-Wl,-rpath=./")

project(open_cam3d_sdk CXX)
find_package(Threads)
set(APP_SRC open_cam3d.cpp)

add_library(open_cam3d_sdk SHARED open_cam3d.cpp triangulation.cpp socket_tcp.cpp utils.cpp easylogging++.cc) 
target_link_libraries(open_cam3d_sdk PRIVATE Threads::Threads)
