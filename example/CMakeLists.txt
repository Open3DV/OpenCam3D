cmake_minimum_required(VERSION 3.4.1)

add_compile_options(-fPIC)
#add_definitions(-std=c++11)

project(example CXX)

set(APP_SRC example.cpp)


#print message
message(${PROJECT_SOURCE_DIR})   


link_libraries(${PROJECT_SOURCE_DIR}/libopen_cam3d_sdk.so) 
add_executable(${PROJECT_NAME} ${APP_SRC}) 
