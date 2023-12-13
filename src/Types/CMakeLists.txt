

find_package(Protobuf REQUIRED)

if (DEFINED ${PROTOBUF_IMPORT_DIRS})
        set (RRC_PROTO_INCLUDES -I ${CMAKE_CURRENT_SOURCE_DIR} -I ${PROTOBUF_IMPORT_DIRS})
endif()

message("${PROTOBUF_PROTOC_EXECUTABLE} -I ${CMAKE_CURRENT_SOURCE_DIR} -I ${PROTOBUF_IMPORT_DIRS} --cpp_out=${CMAKE_CURRENT_SOURCE_DIR} RobotRemoteControl.proto")

add_custom_command( OUTPUT RobotRemoteControl.pb.cc RobotRemoteControl.pb.h
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} ${RRC_PROTO_INCLUDES} --cpp_out=${CMAKE_CURRENT_SOURCE_DIR} RobotRemoteControl.proto
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
                $<BUILD_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>
                $<BUILD_INTERFACE:${PROTOBUF_INCLUDE_DIR}>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
                $<INSTALL_INTERFACE:${PROTOBUF_INCLUDE_DIR}>
)
link_directories(${CMAKE_INSTALL_PREFIX}/lib)


target_link_libraries(robot_remote_control-types ${PROTOBUF_LIBRARY})

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/RobotRemoteControl.pb.h
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robot_remote_control/Types
)

install (TARGETS robot_remote_control-types
        EXPORT robot_remote_control-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)


# install uncompiled conversion headers
install(DIRECTORY Conversions/
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robot_remote_control/Types/Conversions
)