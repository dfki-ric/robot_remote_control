
find_package(robot_remote_control REQUIRED)

add_executable(robot_remote_control-extended_robot_controller_bin ExtendedRobotControllerMain.cpp)
target_link_libraries(robot_remote_control-extended_robot_controller_bin
    robot_remote_control-extended_robot_controller
    robot_remote_control-transport_zmq
)
target_include_directories(robot_remote_control-extended_robot_controller_bin
	PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/examples>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/RobotController>
)

add_executable(robot_remote_control-extended_controlled_robot_bin ExtendedControlledRobotMain.cpp)
target_link_libraries(robot_remote_control-extended_controlled_robot_bin
    robot_remote_control-extended_controlled_robot
    robot_remote_control-transport_zmq
)
target_include_directories(robot_remote_control-extended_controlled_robot_bin
	PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/examples>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/ControlledRobot>
)