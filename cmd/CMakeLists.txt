cmake_minimum_required(VERSION 3.4.1)

add_compile_options(-fPIC)
add_definitions(-std=c++11)

set(CMAKE_CXX_FLAGS "-Wl,-rpath=./")
project(open_cam3d CXX)

include_directories(${OpenCV_INCLUDE_DIRS}  ../test ../sdk)

set(APP_SRC cmd.cpp getopt.c ../sdk/socket_tcp.cpp ../sdk/utils.cpp ../firmware/easylogging++.cc ../gui/PrecisionTest.cpp ../calibration/calibrate_function.cpp ../test/reconstruct.cpp ../test/AnalyseError.cpp ../test/encode.cpp ../test/solution.cpp ../test/LookupTableFunction.cpp ../test/FilterModule.cpp ../test/support_function.cpp)

link_libraries(${PROJECT_SOURCE_DIR}/../example/libopen_cam3d_sdk.so)
#print message
message(${PROJECT_SOURCE_DIR})   

 
find_package( OpenCV REQUIRED ) 
add_executable(${PROJECT_NAME} ${APP_SRC}) 

find_package(Threads)
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} Threads::Threads)
