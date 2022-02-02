cmake_minimum_required(VERSION 3.1.0)

add_executable(robot_remote_control-robot_controller_cli RobotControllerCLI.cpp ConsoleCommands.cpp)
target_link_libraries(robot_remote_control-robot_controller_cli
    robot_remote_control-robot_controller
    robot_remote_control-transport_zmq
    readline
)
target_include_directories(robot_remote_control-robot_controller_cli
	PUBLIC
        $<BUILD_INTERFACE:/usr/include/readline>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/RobotController>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

