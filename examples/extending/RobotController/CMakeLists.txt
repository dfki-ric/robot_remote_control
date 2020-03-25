add_library(robot_remote_control-extended_robot_controller
ExtendedRobotController.cpp
../ExtendedTelemetryBuffer.cpp
)

target_link_libraries (robot_remote_control-extended_robot_controller
robot_remote_control-robot_controller
robot_remote_control-types
myrobot-types
robot_remote_control-update_thread
)

target_include_directories(robot_remote_control-extended_robot_controller
PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/RobotController>
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/examples/extending/>
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/examples/extending/RobotController>

$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)