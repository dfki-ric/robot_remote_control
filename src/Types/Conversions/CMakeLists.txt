set(ROS_VERSION "2")

if(${ROS_VERSION} EQUAL "2")
    set(CODE_DIRECTORY "ros2")
else()
    set(CODE_DIRECTORY "ros")
endif()

install(DIRECTORY Conversions/rock/.
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robot_remote_control/Types/Conversions/rock
)
install(DIRECTORY Conversions/${CODE_DIRECTORY}/.
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robot_remote_control/Types/Conversions/ros
)