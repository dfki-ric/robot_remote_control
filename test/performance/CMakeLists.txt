cmake_minimum_required(VERSION 3.1.0)


add_executable(robot_remote_control-controlled_robot_performance_test ControlledRobotMain.cpp)
target_link_libraries(robot_remote_control-controlled_robot_performance_test
    robot_remote_control-controlled_robot
    robot_remote_control-transport_zmq
)
target_include_directories(robot_remote_control-controlled_robot_performance_test
	PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/ControlledRobot>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)


add_executable(robot_remote_control-robot_controller_performance_test RobotControllerMain.cpp)
target_link_libraries(robot_remote_control-robot_controller_performance_test
    robot_remote_control-robot_controller
    robot_remote_control-transport_zmq
)
target_include_directories(robot_remote_control-robot_controller_performance_test
	PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/RobotController>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
