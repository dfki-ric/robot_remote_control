add_library(robot_remote_control-extended_controlled_robot
ExtendedControlledRobot.cpp
)

target_link_libraries (robot_remote_control-extended_controlled_robot
robot_remote_control-controlled_robot
robot_remote_control-types
myrobot-types
robot_remote_control-update_thread
)

target_include_directories(robot_remote_control-extended_controlled_robot
PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/ControlledRobot>

$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)