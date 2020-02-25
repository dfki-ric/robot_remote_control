
find_package (Threads)

add_executable(test_suite
   suite.cpp
   test_Communication.cpp
   test_RingBuffer.cpp
   test_Thread.cpp
)

find_package(robot_remote_control REQUIRED) 
#find_package(Protobuf REQUIRED)
find_package(Boost REQUIRED COMPONENTS unit_test_framework system)

add_definitions(-DBOOST_TEST_DYN_LINK)

target_link_libraries(test_suite
   ${CMAKE_THREAD_LIBS_INIT}
   ${robot_remote_control_LIBRARIES}
   ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
)


set_target_properties(test_suite PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)
