# FAQ

## Why are there two libraries?

RRC is a directed Communication with the possibility to implement different Quality of Service for commands and telemetry.

With the default ZMQ Transport, telemetry ist send without acknowledge message, 


## How do I control different parts using Twist, Pose or GoTo

There are several approaches possible, our recommondation is to use the header field to differentiate the targets of the control.

For generic controller implementations you can report frames that can be used by using initControllableFrames()/requestControllableFrames()

When using this option, please set the "onlyNewest" option of the command getters in the ControlledRobot to false in order not to loose commands for different frames ans select a fitting buffer size in the ControlledRobot constructor (default: 10).

A less recommended way is to define a "SimpleAction" that switches the controlled part instead of using the header.

## Ho can I save Bandwidth

In the default setup (ZeroMQ), each RobotController receives ALL telemetry and each instance connects directly to the robot. When you connect multiple times, you can use the included proxy to connect to the robot and then connect additional connections to your PC. 

```$> robot_remote_control-zmq_proxy ROBOT_IP [robot_commandport robot_telemetryport local_commandport local_telemetryport]``` 

Another option is to compress the data using the gzip transport wrapper (see doc/README.md).


## How can I increase reaction times

Rather than polling in your update loop, you can 
Both classes define functions to set callbacks when data arrives. 

```c++
ControlledRobot::addCommandReceivedCallback()
RobotController::addTelemetryReceivedCallback()
```

Both are available with a generic callback and for specific message types.
