cmake_minimum_required(VERSION 3.1)

set(CMAKE_BUILD_TYPE "Release")

project(DF8 CXX CUDA)
find_package(Threads)
find_package(CUDA REQUIRED)
set(HOME /home/dexforce)

message(${PROJECT_SOURCE_DIR})

# include_directories(${HOME}/opencv-4.4.0/include)
# include_directories(${HOME}/opencv-4.4.0/modules/core/include)
# include_directories(${HOME}/opencv-4.4.0/modules/highgui/include)
# include_directories(${HOME}/opencv-4.4.0/modules/imgcodecs/include)
# include_directories(${HOME}/opencv-4.4.0/modules/videoio/include)
# link_directories(${HOME}/opencv-4.4.0/build/lib)
# link_libraries(${HOME}/opencv-4.4.0/build/lib/libopencv_core.so.4.4)
# link_libraries(${HOME}/opencv-4.4.0/build/lib/libopencv_highgui.so.4.4)
# link_libraries(${HOME}/opencv-4.4.0/build/lib/libopencv_imgcodecs.so.4.4)

include_directories(/usr/local/cuda-10.2/targets/aarch64-linux/include/)
link_libraries(/usr/lib/libgxiapi.so)
add_executable(camera_server camera_server.cpp system_config_settings.cpp camera_dh.cpp i2c.cpp lightcrafter3010.cpp easylogging++.cc encode_cuda.cu)
target_link_libraries(camera_server PRIVATE Threads::Threads)
