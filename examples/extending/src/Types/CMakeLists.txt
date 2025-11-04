

find_package(Protobuf REQUIRED)
find_package(robot_remote_control REQUIRED)

set(PROTOBUF_PACKAGENAME myrobot)
set(robot_remote_control_DIR /opt/workspace/src/interaction/libraries/robot_remote_control)
set(protobuf_DIR /opt/workspace/install/robot_remote_control/include/protobuf/)

add_custom_command( OUTPUT ${PROTOBUF_PACKAGENAME}.pb.cc ${PROTOBUF_PACKAGENAME}.pb.h
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		    COMMAND protoc -I ${CMAKE_CURRENT_SOURCE_DIR} -I ${protobuf_DIR} --cpp_out=${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/${PROTOBUF_PACKAGENAME}.proto
                    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/${PROTOBUF_PACKAGENAME}.pb.cc ${CMAKE_CURRENT_BINARY_DIR}/${PROTOBUF_PACKAGENAME}.pb.cc 
                    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/${PROTOBUF_PACKAGENAME}.pb.h ${CMAKE_CURRENT_BINARY_DIR}/${PROTOBUF_PACKAGENAME}.pb.h
		    COMMAND ${CMAKE_COMMAND} -E copy ${robot_remote_control_DIR}/src/Types/RobotRemoteControl.pb.h ${CMAKE_CURRENT_SOURCE_DIR}/
                    MAIN_DEPENDENCY ${PROTOBUF_PACKAGENAME}.proto
                    )

INSTALL(FILES ${PROTOBUF_PACKAGENAME}.proto DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/protobuf)


add_library(${PROTOBUF_PACKAGENAME}-types 
${PROTOBUF_PACKAGENAME}.pb.cc
)

target_include_directories(${PROTOBUF_PACKAGENAME}-types
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(${PROTOBUF_PACKAGENAME}-types robot_remote_control-types protobuf)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/${PROTOBUF_PACKAGENAME}.pb.h
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROTOBUF_PACKAGENAME}/Types
)

install (TARGETS ${PROTOBUF_PACKAGENAME}-types
        EXPORT ${PROTOBUF_PACKAGENAME}-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)



