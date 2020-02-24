
set(CMAKE_BUILD_TYPE Release)

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_custom_command( OUTPUT Types/RobotRemoteControl.pb.cc Types/RobotRemoteControl.pb.h
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Types
                    COMMAND protoc --cpp_out=${CMAKE_CURRENT_SOURCE_DIR}/Types RobotRemoteControl.proto
                    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Types/RobotRemoteControl.pb.cc ${CMAKE_CURRENT_BINARY_DIR}/Types/RobotRemoteControl.pb.cc 
                    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Types/RobotRemoteControl.pb.h ${CMAKE_CURRENT_BINARY_DIR}/Types/RobotRemoteControl.pb.h
                    MAIN_DEPENDENCY Types/RobotRemoteControl.proto
                    )

include_directories(${CMAKE_CURRENT_BINARY_DIR})

INSTALL(FILES Types/RobotRemoteControl.proto DESTINATION protobuf)


#add_subdirectory(UpdateThread)

add_library(robot_remote_control-update_thread
            UpdateThread/UpdateThread.cpp
            )
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/UpdateThread
	DESTINATION include/robot_remote_control
	FILES_MATCHING PATTERN "*.hpp"
)


add_library(robot_remote_control-types 
            Types/RobotRemoteControl.pb.cc
)
#target_link_libraries (robot_remote_control-types ${PROTOBUF_LIBRARIES})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/Types/RobotRemoteControl.pb.h
	DESTINATION include/robot_remote_control/Types
)

find_package(PkgConfig)
pkg_check_modules(ZMQ REQUIRED libzmq)

add_library(robot_remote_control-transport_zmq 
            Transports/TransportZmq.cpp
)
target_link_libraries (robot_remote_control-types ${ZMQ_LIBRARIES})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Transports
	DESTINATION include/robot_remote_control
	FILES_MATCHING PATTERN "*.hpp"
)


add_library(robot_remote_control-controlled_robot
            ControlledRobot.cpp TelemetryBuffer.cpp 
)
#target_link_libraries (robot_remote_control-controlled_robot robot_remote_control-types robot_remote_control-update_thread ${PROTOBUF_LIBRARIES})


add_library(robot_remote_control-robot_controller
            RobotController.cpp TelemetryBuffer.cpp SimpleSensorBuffer.cpp
)
#target_link_libraries (robot_remote_control-robot_controller ${PROTOBUF_LIBRARIES} )

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
	DESTINATION include/robot_remote_control
	FILES_MATCHING PATTERN "*.hpp"
)


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


install (TARGETS 
        robot_remote_control-types
        robot_remote_control-update_thread
        robot_remote_control-transport_zmq 
        robot_remote_control-controlled_robot
        robot_remote_control-robot_controller
        robot_remote_control-robot_controller_bin
        robot_remote_control-controlled_robot_bin
        LIBRARY DESTINATION lib/
        RUNTIME DESTINATION bin/
)


#include(CMakePackageConfigHelpers)
#configure_package_config_file(
#    "${PROJECT_SOURCE_DIR}/cmake/mylibConfig.cmake.in"
#    "${PROJECT_BINARY_DIR}/mylibConfig.cmake"
#)