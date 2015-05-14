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

#rosbuild_add_executable(test_app test/test_app.cpp)
#target_link_libraries(test_app ${PROJECT_NAME})

#rosbuild_add_executable(init_arm_pose test/init_arm_pose.cpp)
#target_link_libraries(init_arm_pose ${PROJECT_NAME})

#rosbuild_add_executable(test0 test/test0.cpp)
#target_link_libraries(test0 ${PROJECT_NAME})

#rosbuild_add_executable(test1 test/test1.cpp)
#target_link_libraries(test1 ${PROJECT_NAME})

#rosbuild_add_executable(test2 test/test2.cpp)
#target_link_libraries(test2 ${PROJECT_NAME})

#rosbuild_add_executable(test3 test/test3.cpp)
#target_link_libraries(test3 ${PROJECT_NAME})

#rosbuild_add_executable(test4 test/test4.cpp)
#target_link_libraries(test4 ${PROJECT_NAME})

#rosbuild_add_executable(test5 test/test5.cpp)
#target_link_libraries(test5 ${PROJECT_NAME})

#rosbuild_add_executable(test6 test/test6.cpp)
#target_link_libraries(test6 ${PROJECT_NAME})

#rosbuild_add_executable(test7 test/test7.cpp)
#target_link_libraries(test7 ${PROJECT_NAME})

#rosbuild_add_executable(test8 test/test8.cpp)
#target_link_libraries(test8 ${PROJECT_NAME})

#rosbuild_add_executable(test9 test/test9.cpp)
#target_link_libraries(test9 ${PROJECT_NAME})

#rosbuild_add_executable(test10 test/test10.cpp)
#target_link_libraries(test10 ${PROJECT_NAME})

#rosbuild_add_executable(test11 test/test11.cpp)
#target_link_libraries(test11 ${PROJECT_NAME})

#rosbuild_add_executable(test12 test/test12.cpp)
#target_link_libraries(test12 ${PROJECT_NAME})

#rosbuild_add_executable(test13 test/test13.cpp)
#target_link_libraries(test13 ${PROJECT_NAME})

#rosbuild_add_executable(test14 test/test14.cpp)
#target_link_libraries(test14 ${PROJECT_NAME})

#rosbuild_add_executable(test15 test/test15.cpp)
#target_link_libraries(test15 ${PROJECT_NAME})

#rosbuild_add_executable(test16 test/test16.cpp)
#target_link_libraries(test16 ${PROJECT_NAME})

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
