cmake_minimum_required(VERSION 3.4.1)

add_compile_options(-fPIC)
add_definitions(-std=c++11)

project(open_cam3d_test CXX)

include_directories(${OpenCV_INCLUDE_DIRS} ../sdk)

set(APP_SRC test.cpp reconstruct.cpp AnalyseError.cpp encode.cpp solution.cpp LookupTableFunction.cpp FilterModule.cpp support_function.cpp triangulation.cpp ../sdk/socket_tcp.cpp ../sdk/utils.cpp ../firmware/easylogging++.cc ../gui/PrecisionTest.cpp ../calibration/calibrate_function.cpp  )

link_libraries(${PROJECT_SOURCE_DIR}/../example/libopen_cam3d_sdk.so)
#print message
message(${PROJECT_SOURCE_DIR})   

 
find_package( OpenCV REQUIRED ) 
add_executable(${PROJECT_NAME} ${APP_SRC}) 

find_package(Threads)
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} Threads::Threads)
