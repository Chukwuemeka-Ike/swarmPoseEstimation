cmake_minimum_required( VERSION 2.8 )

project( pose_publisher )

set( CMAKE_SYSTEM_NAME Linux )
set( CMAKE_SYSTEM_PROCESSOR arm )
set( CMAKE_C_COMPILER ${tools}/usr/local/khepera4-yocto/build/tmp/sysroots/i686-linux/usr/bin/armv7a-vfp-neon-poky-linux-gnueabi/arm-poky-linux-gnueabi-gcc )
set( CMAKE_CXX_COMPILER ${tools}/usr/local/khepera4-yocto/build/tmp/sysroots/i686-linux/usr/bin/armv7a-vfp-neon-poky-linux-gnueabi/arm-poky-linux-gnueabi-g++ )
add_compile_options(-std=c++11)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set( OpenCV_DIR ~/kh4_dev/opencv/build )
find_package( OpenCV REQUIRED )

include_directories(${OpenCV_INCLUDE_DIRS} ${PACKAGE_INCLUDE_DIR}  khepera )

add_executable( tagDodger tag_dodger.cpp )
target_link_libraries( tagDodger PUBLIC ${OpenCV_LIBS} khepera )