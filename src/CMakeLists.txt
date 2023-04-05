
#set(CMAKE_BUILD_TYPE Release)

include(GNUInstallDirs)

link_directories(${CMAKE_INSTALL_PREFIX}/lib)

add_subdirectory(Types)
add_subdirectory(UpdateThread)
add_subdirectory(Transports)

add_subdirectory(ControlledRobot)
add_subdirectory(RobotController)

add_subdirectory(CLI)
add_subdirectory(Tools)

#install src folder headers
install(FILES
	MessageTypes.hpp
	RingBuffer.hpp
	CommandBuffer.hpp
	TelemetryBuffer.hpp
	SimpleBuffer.hpp
	Statistics.hpp
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robot_remote_control
)



#Export the library interface
install(
    EXPORT robot_remote_control-targets
	#NAMESPACE robot_remote_control::
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/robot_remote_control
)


# Create and install the version file
include(CMakePackageConfigHelpers)
write_basic_package_version_file("robot_remote_control-version.cmake"
	VERSION ${VERSION}
	COMPATIBILITY SameMajorVersion
)

install(
	FILES
		robot_remote_control-config.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/robot_remote_control-version.cmake
	DESTINATION
		${CMAKE_INSTALL_LIBDIR}/cmake/robot_remote_control
)


# Install pkg-config files
configure_file(robot_remote_control-types.pc.in robot_remote_control-types.pc @ONLY)
configure_file(robot_remote_control-update_thread.pc.in robot_remote_control-update_thread.pc @ONLY)
configure_file(robot_remote_control-transport_zmq.pc.in robot_remote_control-transport_zmq.pc @ONLY)
configure_file(robot_remote_control-controlled_robot.pc.in robot_remote_control-controlled_robot.pc @ONLY)
configure_file(robot_remote_control-robot_controller.pc.in robot_remote_control-robot_controller.pc @ONLY)
configure_file(robot_remote_control-send_timers.pc.in robot_remote_control-send_timers.pc @ONLY)

install(FILES 
	${CMAKE_CURRENT_BINARY_DIR}/robot_remote_control-types.pc
	${CMAKE_CURRENT_BINARY_DIR}/robot_remote_control-update_thread.pc
	${CMAKE_CURRENT_BINARY_DIR}/robot_remote_control-transport_zmq.pc
	${CMAKE_CURRENT_BINARY_DIR}/robot_remote_control-controlled_robot.pc
	${CMAKE_CURRENT_BINARY_DIR}/robot_remote_control-robot_controller.pc
	${CMAKE_CURRENT_BINARY_DIR}/robot_remote_control-send_timers.pc
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
)

# install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
# 	DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
# 	FILES_MATCHING PATTERN "*.pc"
# )
