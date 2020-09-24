add_executable(robot_remote_control-transport_udt_server ServerMain.cpp)
target_link_libraries(robot_remote_control-transport_udt_server
	              robot_remote_control-transport_udt)

add_executable(robot_remote_control-transport_udt_client ClientMain.cpp)
target_link_libraries(robot_remote_control-transport_udt_client
	              robot_remote_control-transport_udt)
