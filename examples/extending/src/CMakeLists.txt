include(GNUInstallDirs)

find_package(robot_remote_control REQUIRED)

add_subdirectory(Types)
add_subdirectory(RobotController)
add_subdirectory(ControlledRobot)

