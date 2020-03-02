

find_package(Protobuf REQUIRED)

add_custom_command( OUTPUT Types/RobotRemoteControl.pb.cc RobotRemoteControl.pb.h
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                    COMMAND protoc --cpp_out=${CMAKE_CURRENT_SOURCE_DIR} RobotRemoteControl.proto
                    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/RobotRemoteControl.pb.cc ${CMAKE_CURRENT_BINARY_DIR}/RobotRemoteControl.pb.cc 
                    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/RobotRemoteControl.pb.h ${CMAKE_CURRENT_BINARY_DIR}/RobotRemoteControl.pb.h
                    MAIN_DEPENDENCY RobotRemoteControl.proto
                    )

INSTALL(FILES RobotRemoteControl.proto DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/protobuf)


add_library(robot_remote_control-types 
            RobotRemoteControl.pb.cc
)
target_include_directories(robot_remote_control-types
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(robot_remote_control-types protobuf)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/RobotRemoteControl.pb.h
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robot_remote_control/Types
)

install (TARGETS robot_remote_control-types
        EXPORT robot_remote_control-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)



