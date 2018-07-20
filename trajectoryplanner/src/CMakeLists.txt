
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libtrajectoryplanner-<target>.so
# and installs in the directory lib/orocos/trajectoryplanner/
#
orocos_component(trajectoryplanner trajectoryplanner-component.hpp trajectoryplanner-component.cpp) # ...you may add multiple source files
target_link_libraries(trajectoryplanner ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/trajectoryplanner/ directory
#
orocos_install_headers(trajectoryplanner-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

