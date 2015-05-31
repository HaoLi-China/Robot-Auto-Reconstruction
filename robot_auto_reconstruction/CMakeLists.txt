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
rosbuild_gensrv()

rosbuild_add_library(${PROJECT_NAME} src/arm_control.cpp)
rosbuild_add_library(${PROJECT_NAME} src/gripper_control.cpp)
rosbuild_add_library(${PROJECT_NAME} src/head_control.cpp)
rosbuild_add_library(${PROJECT_NAME} src/torso_control.cpp)
rosbuild_add_library(${PROJECT_NAME} src/base_control.cpp)
rosbuild_add_library(${PROJECT_NAME} src/robot_control.cpp)
rosbuild_add_library(${PROJECT_NAME} src/auto_reconstruction.cpp)
rosbuild_add_library(${PROJECT_NAME} src/up_down_gripper.cpp)

rosbuild_add_executable(socketServer src/socketServer.cpp)
target_link_libraries(socketServer ${PROJECT_NAME})

rosbuild_add_executable(workBeforeCalibration src/workBeforeCalibration.cpp)
target_link_libraries(workBeforeCalibration ${PROJECT_NAME})

rosbuild_add_executable(calibration src/calibration.cpp)
target_link_libraries(calibration ${PROJECT_NAME})

rosbuild_add_executable(testt src/test.cpp)
target_link_libraries(testt ${PROJECT_NAME})

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
