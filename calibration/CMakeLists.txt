cmake_minimum_required(VERSION 3.4.1)

add_compile_options(-fPIC)
add_definitions(-std=c++11)

set(CMAKE_CXX_FLAGS "-Wl,-rpath=./")
project(calibration CXX)

include_directories(${OpenCV_INCLUDE_DIRS}  ../test ../sdk ../gui)

set(APP_SRC calibration.cpp calibrate_function.cpp calibrate_machine.h  ../gui/PrecisionTest.cpp ../test/encode.cpp ../test/support_function.cpp)

link_libraries(${PROJECT_SOURCE_DIR}/../example/libopen_cam3d_sdk.so)
#print message
message(${PROJECT_SOURCE_DIR})   

 
find_package( OpenCV REQUIRED ) 
add_executable(${PROJECT_NAME} ${APP_SRC}) 

find_package(Threads)
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} Threads::Threads)
