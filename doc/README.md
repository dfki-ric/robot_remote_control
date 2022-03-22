# Robot Remote Control (RRC)

This readme is meant as a quick start guide and does not cover the full capabilities of the rrc library

## Libraries to use

RRC has two main libraries:

* RobotController: The part that sends command to the robot.
* ControlledRobot: Tha part that sends telemetry.

Both need to be instanciated with appropriate Implementations of a "Transport", which actually sends data through a "divice".

The default here is the "TransportZmq", where the comamnds are send using the REQ-REP pattern (request-reply) and telemetry is send using PUB-SUB (Publish-Subscribe). 

### Default Robot Setup
```c++
    TransportSharedPtr commands = TransportSharedPtr(new TransportZmq("tcp://*:7001", TransportZmq::REP));
    TransportSharedPtr telemetry = TransportSharedPtr(new TransportZmq("tcp://*:7002", TransportZmq::PUB));
    ControlledRobot robot(commands, telemetry);
```
### Default Controller Setup
```c++
    TransportSharedPtr commands = TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7001", TransportZmq::REQ));
    TransportSharedPtr telemetry = TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7002", TransportZmq::SUB));
    robot_remote_control::RobotController controller(commands, telemetry);
```

This way, the telemetry is send without reply and the commands are send as request (with reply) top be sure the command actually arrived.

The instanciation outside the class makes it possible to easily replace the transport with custom Transport implementations (a [UDT](https://udt.sourceforge.io/) based implementation for high latency connections is included).

Also wrapping a Transport is possible this way, for example to have an encrypted or compressed communication (Gzip- based compression is included). 
```c++
    TransportSharedPtr telemetry = TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7002", TransportZmq::SUB));
    TransportSharedPtr compressed_telemetry = TransportSharedPtr(new TransportWrapperGzip(telemetry);
    ControlledRobot robot(commands, compressed_telemetry);
```

You need to call update() regulary of use the startUpdateThread() function to do this in a Thread (RRC is thread-save).

## get, request, init, and set Functions

Both librareis are defining functions to send/receive data.

The ControlledRobot defines functions with set and init prefix.
The RobotController defines functions with get and request prefix.

* ControlledRobot
  * init functions are normally set once and are **not** send by default. Their data is only available via request like initRobotName().
  * set functions are for telemetry that is regulary send.

* RobotController
  * get functions are for receiving telemetry, they return true as long new data is read by the function
  * request functions are for reqesting init data **or** normal telemetry via the command connection.


## Detection of connection losses

Generally connection breaks (Ethernet) are detected while sending.

* The ZeroMQ Transport will automatically reconnect after connection losses.
* Sending telemetry via ZeroMQ will not detect connection losses (publish-subscribe).

You can setup a automatic regular send of a message using controller.setHeartBeatDuration(0.5);
This will send a message every 0.5 seconds and detect a connection loss during the send.

You can define a callback which is called on a connection loss using RobotController::setupLostConnectionCallback()

On the ControlledRobot side, you can use ControlledRobot::setupHeartbeatCallback() to define a function that reacts to connection losses.
You must also set a allowedLatency here.

The total time after which the callback on the robot side (ControlledRobot) is executed is defined by

* The setHeartBeatDuration set by the controller
* plus the allowedLatency set by the robot

With these the robot expects the next heartbeat message after duration+latency seconds, if no heartbeat message arrives in time the callback will be called. The paramater of the callback gived the elapsed time since the last received message.


## Controlling Different Robot Parts (Defining Controllable Frames)

There is mostly only one function to send a command, e.g. setTwistCommand(), while some robots have multiple parts that can be moved with a twist command (e.g. they can have a mobile platform and a arm).

In these cases, the "header" part of the command can be used to set the target frame of the command.
When time synchronization of the command is important, a "ComplexAction" can be used to send the commands.

When the robot should be usable for generic user interfaces (that use the RobotController library), the initControllableFrames() function should be used to report the availabe commands. 


## Defining Actions

Actions are funtions of the robot that it can finish on its own.
This can be simple switches (light on) or autonomous behaviors (return to docking station).
They devided into "simple" and "complex" actions. Where thsi only defines 

### Simple Actions

Simple actions are name-based and contain only a float value as command. They can be used as a trigger or set a single value (e.g. "stop", "maximumSpeed", "controlMode").

When the robot should be usable for generic user interfaces (that use the RobotController library), the initSimpleActions() function should be used to report the availabe actions, the type (NamedValue) setting is only important for generated GUIs, not for the content of the message.

### Complex Actions

Complex actions are also name-baed and contain a array of Twists and/or Poses. These action can be used for more complex behaviors. (e.g. "AreaExploration", "FollowTrajectory", "SyncronousDualArmTwistControl") 

When the robot should be usable for generic user interfaces (that use the RobotController library), the initComplexActions() function should be used to report the availabe actions, the type (NamedValue) setting is only important for generated GUIs, not for the content of the message.


## Tools

### ZeroMQ Proxy

While other transports might only support single one to one connections, the ZeroMQ Transport supports a one to many connections.
Each instance of RobotController connects to the ControlledRobot instance on the robot.
This also means all telemetry is send to all connected RobotController instances and thus using more Bandwidth. 
This can be a problem for Robots with limited bandwidth (e.g. wirelessly connected).

For these cases, RRC contains a ZeroMQ proxy implementation which allows to have onle one connection to the robot and to connect other instances of RobotController to the proxy (`$> robot_remote_control-zmq_proxy`). This way the telemetry is only sent once on the robot-proxy connection.

### Statistics

RRC has an Statistics module for the telemetry on both sided of the communication.

The cmake define "RRC_STATISTICS" toggles the availability and calculation of the statistics.

You can use the `getStatistics()` function to get the data. The indes is based on the TelemetryMessageType enum in src/MessageTypes.hpp, where the index 0 (NO_TELEMETRY_DATA) contains the global statistic.

### Command Line Interface

RRC comes with an command line interface for testing purposes (`$> robot_controller`). 

It establishes a connection to the robot and provides basic requests and controls for the robot.
It can also be used to access the statistics module to check which telemetry at which rates are actually send by the robot.

(please see the CLI readme)[CLI.md]







