
#add_subdirectory(extending/src)

add_executable(robot_remote_control-controlled_robot_bin ControlledRobotMain.cpp)
target_link_libraries(robot_remote_control-controlled_robot_bin
    robot_remote_control-controlled_robot
    robot_remote_control-transport_zmq
)
target_include_directories(robot_remote_control-controlled_robot_bin
	PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/ControlledRobot>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)


add_executable(robot_remote_control-robot_controller_bin RobotControllerMain.cpp)
target_link_libraries(robot_remote_control-robot_controller_bin
    robot_remote_control-robot_controller
    robot_remote_control-transport_zmq
)
target_include_directories(robot_remote_control-robot_controller_bin
	PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/RobotController>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)


# install (TARGETS
#         robot_remote_control-controlled_robot_bin
#         robot_remote_control-robot_controller_bin
#         RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
# )

