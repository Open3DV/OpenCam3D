cmake_minimum_required(VERSION 3.4.1)

add_compile_options(-fPIC)
add_definitions(-std=c++11)

set(CMAKE_CXX_FLAGS   "-no-pie")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

SET(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

project(open_cam3d_gui CXX)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/bin)

include_directories(${OpenCV_INCLUDE_DIRS}  ../test ../sdk)

set(HEADERS 
    camera_gui.h 
    camera_capture_gui.h
    calibration_param_gui.h
    select_calibration_board_gui.h
)

set(SOURCES 
    main.cpp
    camera_gui.cpp 
    camera_capture_gui.cpp
    calibration_param_gui.cpp 
    select_calibration_board_gui.cpp
)

set(RESOURCES 
    dexforce_camera_gui.qrc 
)

set(APP_SRC
 calibration_param_gui.cpp 
 settings_file_function.cpp 
 ../sdk/socket_tcp.cpp 
 ../sdk/utils.cpp 
 ../firmware/easylogging++.cc
 ../firmware/system_config_settings.cpp
 PrecisionTest.cpp
 file_io_function.cpp
 ../calibration/calibrate_function.cpp
 )

link_libraries(${PROJECT_SOURCE_DIR}/../example/libopen_cam3d_sdk.so)
#print message
message(${PROJECT_SOURCE_DIR})   

find_package(Qt5 COMPONENTS Core Widgets Xml REQUIRED) 

#添加预编译宏
ADD_DEFINITIONS(-DTORRENT_LIBRARY)
ADD_DEFINITIONS(-DNO_FREETYPE)
ADD_DEFINITIONS(-DUNICODE -D_UNICODE)
ADD_DEFINITIONS(-DHCPP_LEGACY_API)

#将各种文件归类到指定变量
FILE(GLOB_RECURSE HEADER_FILES "*.h")
FILE(GLOB_RECURSE SRC_FILES "*.cpp")
FILE(GLOB_RECURSE FORMS "*.ui")
FILE(GLOB_RECURSE RESOURCE_FILES "*.qrc")
FILE(GLOB TS_FILES "*.ts")
FILE(GLOB MOC_FILES "moc_*.cpp")
 
find_package( OpenCV REQUIRED ) 
add_executable(${PROJECT_NAME} ${APP_SRC} ${HEADERS} ${SOURCES} ${RESOURCES} ) 

find_package(Threads)
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} Threads::Threads Qt5::Core Qt5::Gui Qt5::Widgets Qt5::Xml)
