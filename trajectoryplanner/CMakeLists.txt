#
# The find_package macro for Orocos-RTT requires
# cmake >= 2.8
#
cmake_minimum_required(VERSION 2.8)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(trajectoryplanner)

# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()
set(orocos_kdl_DIR /opt/ros/kinetic/share/orocos_kdl)
set(orocos_kdl_INCLUDE_DIR /opt/ros/kinetic/include/kdl)
find_package(catkin REQUIRED COMPONENTS
    rtt_ros
    tf_conversions
    orocos_kdl
    )
#target_link_libraries(my_lib ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories( include ${catkin_INCLUDE_DIRS} ${USE_OROCOS_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIR})
# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

# Rest of the configuration is in src/
add_subdirectory( src )

#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package()
