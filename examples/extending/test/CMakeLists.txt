
find_package (Threads)

add_executable(test_suite
   suite.cpp
   test_Communication.cpp
)

find_package(robot_remote_control REQUIRED)
# Boost.System is header only since boost 1.69, its library was removed in 1.89
find_package(Boost REQUIRED COMPONENTS unit_test_framework)

set(myrobot_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR/../src})
include_directories(${myrobot_SOURCE_DIR})

add_definitions(-DBOOST_TEST_DYN_LINK)

target_link_libraries(test_suite
   ${CMAKE_THREAD_LIBS_INIT}
   robot_remote_control-transport_zmq
   robot_remote_control-controlled_robot
   robot_remote_control-robot_controller
   robot_remote_control-extended_controlled_robot
   robot_remote_control-extended_robot_controller
   myrobot-types
   Boost::unit_test_framework
)


#set_target_properties(test_suite PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)
