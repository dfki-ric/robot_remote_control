
#set(CMAKE_BUILD_TYPE Release)

include(GNUInstallDirs)


add_subdirectory(Types)
add_subdirectory(UpdateThread)
add_subdirectory(Transports)




add_library(robot_remote_control-controlled_robot
            ControlledRobot.cpp TelemetryBuffer.cpp 
)
target_link_libraries (robot_remote_control-controlled_robot robot_remote_control-types robot_remote_control-update_thread)
target_include_directories(robot_remote_control-controlled_robot
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:include>
)

add_library(robot_remote_control-robot_controller
            RobotController.cpp TelemetryBuffer.cpp SimpleSensorBuffer.cpp
)
target_link_libraries (robot_remote_control-robot_controller robot_remote_control-types robot_remote_control-update_thread)
target_include_directories(robot_remote_control-robot_controller
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:include>
)


# #target_link_libraries (robot_remote_control-robot_controller ${PROTOBUF_LIBRARIES} )

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
	DESTINATION include/robot_remote_control
	FILES_MATCHING PATTERN "*.hpp"
)


add_executable(robot_remote_control-robot_controller_bin RobotControllerMain.cpp)
target_link_libraries(robot_remote_control-robot_controller_bin
    robot_remote_control-robot_controller
    robot_remote_control-transport_zmq
)

add_executable(robot_remote_control-controlled_robot_bin ControlledRobotMain.cpp)
target_link_libraries(robot_remote_control-controlled_robot_bin
    robot_remote_control-controlled_robot
    robot_remote_control-transport_zmq
)


install (TARGETS
        robot_remote_control-controlled_robot
        robot_remote_control-robot_controller
        robot_remote_control-robot_controller_bin
        robot_remote_control-controlled_robot_bin
        EXPORT robot_remote_control-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
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

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
	DESTINATION lib/pkgconfig
	FILES_MATCHING PATTERN "*.pc"
)
