cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#rosbuild_add_executable(test1 src/test.cpp)
#rosbuild_add_executable(shame src/shame.cpp)
#rosbuild_add_executable(meta src/meta.cpp)
#rosbuild_add_executable(main src/NavMap.cpp src/ImageConverter.cpp src/main.cpp)
#rosbuild_add_executable(lines src/plines.cpp)
#rosbuild_add_executable(plane_detection_rui src/plane_detection_rui.cpp src/NavMap.cpp src/ImageConverter.cpp)
#rosbuild_add_executable(matcher_simple src/matcher_simple.cpp src/NavMap.cpp src/ImageConverter.cpp)
#rosbuild_add_executable(basic_threshold src/basic_threshold.cpp src/NavMap.cpp src/ImageConverter.cpp)
#rosbuild_add_executable(histogram src/histogram.cpp src/NavMap.cpp src/ImageConverter.cpp)
#rosbuild(plane_detection src/plane_detection)
#rosbuild_add_executable(plane_detection_pcl src/plane_detection_pcl.cpp src/NavMap.cpp src/ImageConverter.cpp)
#rosbuild_add_executable(plane_detection_sctrach src/plane_detection_scratch.cpp src/ImageConverter.cpp)
#rosbuild_add_executable(plane_detection_no_lib src/plane_detection_no_lib.cpp src/ImageConverter.cpp)
rosbuild_add_executable(plane_detection_no_lib src/plane_detection_no_lib.cpp src/NavMap.cpp src/ImageConverter.cpp)
