
find_package (Threads REQUIRED)

add_library(robot_remote_control-update_thread
            UpdateThread.cpp
            Timer.cpp
            )
            
if(CMAKE_USE_PTHREADS_INIT)
    target_compile_options(robot_remote_control-update_thread 
        PUBLIC -DUSES_PTHREAD
        INTERFACE -DUSES_PTHREAD
        )
endif()
    

target_include_directories(robot_remote_control-update_thread
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries (robot_remote_control-update_thread ${CMAKE_THREAD_LIBS_INIT})

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robot_remote_control
	FILES_MATCHING PATTERN "*.hpp"
)

install (TARGETS robot_remote_control-update_thread
        EXPORT robot_remote_control-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
