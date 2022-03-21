# Robot Remote Control (rrc)

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


## Defining Controllable Frames
Work in progress

## Defining Actions
Work in progress




