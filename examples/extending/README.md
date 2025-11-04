# Extending the library

This example explains how to extend the library with further messages.

This Example does not compile with the main library, as the dependencies have to be installed first.

To start extending, just copy this folder and start yourt lib


## build with colcon

copy this folder to src dir and add a package.xml

in src/types/CMakeLists.txt:

set correct dirs:

set(robot_remote_control_DIR /opt/workspace/src/interaction/libraries/robot_remote_control)

set(protobuf_DIR /opt/workspace/install/robot_remote_control/include/protobuf/)

and fix the copy command for the RobotRemoteControl.pb.h file


 COMMAND ${CMAKE_COMMAND} -E copy ${robot_remote_control_DIR}/src/Types/RobotRemoteControl.pb.h ${CMAKE_CURRENT_SOURCE_DIR}/
