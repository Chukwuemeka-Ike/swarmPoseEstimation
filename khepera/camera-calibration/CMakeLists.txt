cmake_minimum_required(VERSION 2.8)

project( camera_calibration )

set( CMAKE_SYSTEM_NAME Linux )
set( CMAKE_SYSTEM_PROCESSOR arm )
# set( CMAKE_C_COMPILER ${tools}/usr/local/khepera4-yocto/build/tmp/sysroots/i686-linux/usr/bin/armv7a-vfp-neon-poky-linux-gnueabi/arm-poky-linux-gnueabi-gcc )
# set( CMAKE_CXX_COMPILER ${tools}/usr/local/khepera4-yocto/build/tmp/sysroots/i686-linux/usr/bin/armv7a-vfp-neon-poky-linux-gnueabi/arm-poky-linux-gnueabi-g++ )

set( CMAKE_C_COMPILER ${tools}$ENV{K4_YOCTO}/armv7a-vfp-neon-poky-linux-gnueabi/arm-poky-linux-gnueabi-gcc )
set( CMAKE_CXX_COMPILER ${tools}$ENV{K4_YOCTO}/armv7a-vfp-neon-poky-linux-gnueabi/arm-poky-linux-gnueabi-g++ )

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set( OpenCV_DIR "~/kh4_dev/opencv/build" )
find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS} "../include"  khepera )
add_compile_options(-std=c++11)

add_executable( cameraCalibration cameraCalibration.cpp )
target_link_libraries( cameraCalibration PUBLIC ${OpenCV_LIBS})