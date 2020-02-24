
set(CMAKE_BUILD_TYPE Release)



find_package(Protobuf REQUIRED)

add_custom_command( OUTPUT Types/RobotRemoteControl.pb.cc Types/RobotRemoteControl.pb.h
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Types
                    COMMAND protoc --cpp_out=${CMAKE_CURRENT_SOURCE_DIR}/Types RobotRemoteControl.proto
                    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Types/RobotRemoteControl.pb.cc ${CMAKE_CURRENT_BINARY_DIR}/Types/RobotRemoteControl.pb.cc 
                    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Types/RobotRemoteControl.pb.h ${CMAKE_CURRENT_BINARY_DIR}/Types/RobotRemoteControl.pb.h
                    MAIN_DEPENDENCY Types/RobotRemoteControl.proto
                    )

INSTALL(FILES Types/RobotRemoteControl.proto DESTINATION protobuf)


#add_subdirectory(UpdateThread)

add_library(robot_remote_control-update_thread
            UpdateThread/UpdateThread.cpp
            )
target_include_directories(robot_remote_control-update_thread
	PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:include>
)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/UpdateThread
	DESTINATION include/robot_remote_control
	FILES_MATCHING PATTERN "*.hpp"
)



add_library(robot_remote_control-types 
            Types/RobotRemoteControl.pb.cc
)
target_include_directories(robot_remote_control-types
	PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:include>
)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/Types/RobotRemoteControl.pb.h
	DESTINATION include/robot_remote_control/Types
)



find_package(PkgConfig)
pkg_check_modules(ZMQ REQUIRED libzmq)

add_library(robot_remote_control-transport_zmq 
            Transports/TransportZmq.cpp
)
target_include_directories(robot_remote_control-transport_zmq 
	PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:include>
)
target_link_libraries (robot_remote_control-transport_zmq ${ZMQ_LIBRARIES})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Transports
	DESTINATION include/robot_remote_control
	FILES_MATCHING PATTERN "*.hpp"
)


add_library(robot_remote_control-controlled_robot
            ControlledRobot.cpp TelemetryBuffer.cpp 
)
#target_link_libraries (robot_remote_control-controlled_robot robot_remote_control-types robot_remote_control-update_thread ${PROTOBUF_LIBRARIES})
target_include_directories(robot_remote_control-controlled_robot
	PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
		$<INSTALL_INTERFACE:include>
)

add_library(robot_remote_control-robot_controller
            RobotController.cpp TelemetryBuffer.cpp SimpleSensorBuffer.cpp
)
target_include_directories(robot_remote_control-robot_controller
	PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:include>
)


#target_link_libraries (robot_remote_control-robot_controller ${PROTOBUF_LIBRARIES} )

#install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
#	DESTINATION include/robot_remote_control
#	FILES_MATCHING PATTERN "*.hpp"
#)


add_executable(robot_remote_control-robot_controller_bin RobotControllerMain.cpp)
target_link_libraries(robot_remote_control-robot_controller_bin
    robot_remote_control-robot_controller
    robot_remote_control-transport_zmq
    robot_remote_control-types
    robot_remote_control-update_thread 
    ${PROTOBUF_LIBRARIES}
)

#rock_executable(robot_remote_control-robot_controller_bin RobotControllerMain.cpp
#    DEPS robot_remote_control-robot_controller robot_remote_control-transport_zmq)

add_executable(robot_remote_control-controlled_robot_bin ControlledRobotMain.cpp)
target_link_libraries(robot_remote_control-controlled_robot_bin
    robot_remote_control-controlled_robot
    robot_remote_control-transport_zmq
    robot_remote_control-types
    robot_remote_control-update_thread 
    ${PROTOBUF_LIBRARIES}
)


#rock_executable(robot_remote_control-controlled_robot_bin ControlledRobotMain.cpp
#    DEPS robot_remote_control-controlled_robot robot_remote_control-transport_zmq)
#set_target_properties(robot_remote_control-controlled_robot_bin PROPERTIES PUBLIC_HEADER "ControlledRobot.hpp;RobotController.hpp")
install (TARGETS
        robot_remote_control-types
        robot_remote_control-update_thread
        robot_remote_control-transport_zmq 
        robot_remote_control-controlled_robot
        robot_remote_control-robot_controller
        robot_remote_control-robot_controller_bin
        robot_remote_control-controlled_robot_bin
        EXPORT robot_remote_control-targets
        LIBRARY DESTINATION lib/
        RUNTIME DESTINATION bin/
)

#Export the library interface
install(EXPORT robot_remote_control-targets
	NAMESPACE robot_remote_control::
	DESTINATION lib/cmake/robot_remote_control
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
        ${PROJECT_BINARY_DIR}/src/robot_remote_control-version.cmake
	DESTINATION
		lib/cmake/robot_remote_control
)


# Install pkg-config file
#get_property(PKG_REQUIRES GLOBAL PROPERTY robot_remote_control_ALL_LIBRARIES)

configure_file(robot_remote_control-types.pc.in robot_remote_control-types.pc @ONLY)
configure_file(robot_remote_control-update_thread.pc.in robot_remote_control-update_thread.pc @ONLY)
configure_file(robot_remote_control-transport_zmq.pc.in robot_remote_control-transport_zmq.pc @ONLY)
configure_file(robot_remote_control-controlled_robot.pc.in robot_remote_control-controlled_robot.pc @ONLY)
configure_file(robot_remote_control-robot_controller.pc.in robot_remote_control-robot_controller.pc @ONLY)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
	DESTINATION lib/pkgconfig
	FILES_MATCHING PATTERN "*.pc"
)
