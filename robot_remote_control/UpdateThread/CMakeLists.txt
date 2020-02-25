add_library(update_thread
  UpdateThread.cpp
)

target_include_directories(update_thread
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:include>
)

install(DIRECTORY ./
	DESTINATION include/robot_remote_control
	FILES_MATCHING PATTERN "*.hpp"
)

install (TARGETS update_thread
         EXPORT robot_remote_control-targets
         LIBRARY DESTINATION lib
         RUNTIME DESTINATION bin
)

