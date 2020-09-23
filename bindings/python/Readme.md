# Python Interface

This interface only provides the RobotController side. It is expected that the robot side is using the native RCOS to wrap the ControlledRobot class.

## Adding Types

There a several places to modify in order to make a functionavailable in python.

The safest way to move protobuf from cpp to python is to serialize in cpp and de-serialize in python.

* In oder to "hide" the cpp protobuf types from python, rather than add all of them to cython, you need to add the disired function to the RobotControllerWrapper.hpp. This file wraps the function to ise std::string binary data as interface (serialized protobuf).
* You also need to provide the now wrapped function to the .pxd file (RobotControllerWrapper.hpp part) to make it availabe in python.
* In the pyx file, add a function to PyRobotController that used this wrapped function to de-serialize the data to thy python protobuf type.


