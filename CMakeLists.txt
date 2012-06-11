cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

# For action lib
rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()
rosbuild_add_boost_directories()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
rosbuild_check_for_sse()
rosbuild_genmsg()
rosbuild_gensrv()

rosbuild_add_executable(scoop_litterbox src/scoop_litterbox_action.cpp)
rosbuild_add_executable(move_to_position src/move_to_position_action.cpp)
rosbuild_add_executable(dump_poop src/dump_poop_action.cpp)
rosbuild_add_executable(explore src/explore_action.cpp)
rosbuild_add_executable(detect_object src/detect_object.cpp)
rosbuild_add_executable(init src/init_action.cpp)
rosbuild_add_executable(detect_scooper_attached src/detect_scooper_attached.cpp)
rosbuild_add_executable(insert_scooper src/insert_scooper_action.cpp)
rosbuild_add_executable(determine_lb_dimensions_server src/determine_lb_dimensions_server.cpp src/find_rectangles.cpp)
rosbuild_add_executable(display_rectangles src/display_rectangles.cpp src/find_rectangles.cpp)
rosbuild_add_executable(move_left_arm_away src/move_left_arm_away.cpp)
rosbuild_add_executable(move_right_arm_away src/move_right_arm_away.cpp)
rosbuild_add_executable(octomap_updater src/octomap_updater.cpp)
rosbuild_add_executable(laserscan_to_pointcloud src/laserscan_to_pointcloud.cpp)
target_link_libraries(insert_scooper tinyxml)
