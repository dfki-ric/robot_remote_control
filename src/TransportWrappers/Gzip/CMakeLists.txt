add_library(robot_remote_control-transport_wrapper_gzip
TransportWrapperGzip.cpp
)
target_include_directories(robot_remote_control-transport_wrapper_gzip
PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries (robot_remote_control-transport_wrapper_gzip
           robot_remote_control-compression
)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/src/TransportWrappers/Gzip/
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robot_remote_control/Transports
	FILES_MATCHING PATTERN "*.hpp"
)
install (TARGETS robot_remote_control-transport_wrapper_gzip
EXPORT robot_remote_control-targets
LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)