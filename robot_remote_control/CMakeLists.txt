#set(CMAKE_BUILD_TYPE Release)

#include(GNUInstallDirs)

#find_package(Protobuf REQUIRED)

#add_custom_command( OUTPUT Types/RobotRemoteControl.pb.cc Types/RobotRemoteControl.pb.h
#                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Types
#                    COMMAND protoc --cpp_out=${CMAKE_CURRENT_SOURCE_DIR}/Types RobotRemoteControl.proto
#                    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Types/RobotRemoteControl.pb.cc ${CMAKE_CURRENT_BINARY_DIR}/Types/RobotRemoteControl.pb.cc 
#                    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Types/RobotRemoteControl.pb.h ${CMAKE_CURRENT_BINARY_DIR}/Types/RobotRemoteControl.pb.h
#                    MAIN_DEPENDENCY Types/RobotRemoteControl.proto
#                    )

#INSTALL(FILES Types/RobotRemoteControl.proto DESTINATION protobuf)


add_subdirectory(UpdateThread)

#Export the library interface
install(
  EXPORT robot_remote_control-targets
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
        ${PROJECT_BINARY_DIR}/robot_remote_control/robot_remote_control-version.cmake
	DESTINATION
		lib/cmake/robot_remote_control
)

