# Tutorial

When you set the cmake option BUILD_TUTORIAL, binaries of tis tutorial are generated in the build folder (but not installed)

## Controlling a Simple Robot

### Robot side

To control a robot, you need to implement the robot side by using robot_remote_control::ControlledRobot.
To instanciate the class, you need to select and configure your transport implementation fist (new Transports can easily be added).
By default, you should use the ZeroMQ transport (TransportZmq).

```cpp
    TransportSharedPtr commands = TransportSharedPtr(new TransportZmq("tcp://*:7001", TransportZmq::REP));
    TransportSharedPtr telemetry = TransportSharedPtr(new TransportZmq("tcp://*:7002", TransportZmq::PUB));
    robot_remote_control::ControlledRobot robot(commands, telemetry);
```

Commands and telemetry are using different connections (ports), because commands are always replied, temeletry does not wait for a reply.
Also, you can choose to use different transports for cammnds ans telemetry (as commands aprobably more important than telemetry)

Next, the receive thread must be started:

```cpp
    robot.startUpdateThread(0);
```

Then, static robot data should be initialized (available functions are starting with "init"), this data can be requested by the RobotController on demand

```cpp
    robot.initRobotName("TestRobot");
```

We also need local variables to hamdle commands and telemetry, these are based on protobuf, their definition can be found in the .protobuf file in the /src/Types folder of this repo.

```cpp
    // commands to receive
    robot_remote_control::Twist twistcommand;
    // robot data
    robot_remote_control::Twist twist;
```

To get used with protobuf, you can use this tutorial: https://protobuf.dev/getting-started/cpptutorial/. Protobuf will generate functions into the types to make the contents printable: void PrintDebugString() will directly print the contents to console, std::string DebugString() and td::string ShortDebugString() will return a string that can be printed manually/elsewhere.

**Protobuf will not print or submit "default values", e.g. when a number value is 0, nothing is printed.**


Finally we need a main loop where we receive commands and send telemetry:

```cpp
    while (true) {
        // receive 
        if (robot.getTwistCommand(&twistcommand)) {
            twistcommand.PrintDebugString();
            // we don't have real robot data, so we just write the value back
            twist = twistcommand;
        }
        // here would be the interaction with the robot software
        
        // write command back (in lack of real data)
        robot.setCurrentTwist(twist);
        usleep(100000);
    }
```

The code is available [here](./tutorial1_ControlledRobot.cpp).

When the BUILD_TUTORIAL cmake option was on, it can be executed with `./build/tutorial/tutorial1_ControlledRobot_bin`

When executed, you can use the `robot_controller` command line interface to test your implementation:

The `robot_controller` binary will connect to the running RobotController implementation and gives you console-like access to functions.

* `help` commannd gived you a list of available commands
* Hitting the `TAB` key gived you auto-completions and helps with command parameters.
* `stats` gives you an approximate of bandwith used per type ans their frequency

When connected to the tutorial application, you can use

* `setTwistCommand 1 2 3 4 5 6` to send a command 
    * your code uses `twistcommand.PrintDebugString()` to print the command
    * sets the twist telemetry to the command value
* `getCurrentTwist` prints the most recent twist telemetry


### Controller side

Normally, you don't want to control a robot using the command line, so there is also a class for the controlling side.

The setup is similar to the robot side, but the transport parameters are different.

```cpp
    TransportSharedPtr commands = TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7001", TransportZmq::REQ));
    TransportSharedPtr telemetry = TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7002", TransportZmq::SUB));
    robot_remote_control::RobotController controller(commands, telemetry);
```

After starting the receive thread, you can request static data:

```cpp
    robot_remote_control::RobotName robotname;
    controller.requestRobotName(&robotname);

    printf("connected to:\n");
    robotname.PrintDebugString();
```

And then fill a command protobuf class and send it to the robot


```cpp
// commands to send
    robot_remote_control::Twist twist_command;

    // robot data
    robot_remote_control::Twist twist_telemetry;

    int offset = 0;
    while (true) {

        // generate some command
        // write access to complex types have to use the mutable pointer in protobuf
        twist_command.mutable_angular()->set_x(offset+1);
        twist_command.mutable_angular()->set_y(offset+2);
        twist_command.mutable_angular()->set_z(offset+3);
        twist_command.mutable_linear()->set_x(offset+4);
        twist_command.mutable_linear()->set_y(offset+5);
        twist_command.mutable_linear()->set_z(offset+6);
        ++offset;
        // send the command
        controller.setTwistCommand(twist_command);

        controller.getCurrentTwist(&twist_telemetry);
        twist_telemetry.PrintDebugString();

        usleep(1000000);
    }
```

The code is available [here](./tutorial1_RobotController.cpp).

When the BUILD_TUTORIAL cmake option was on, it can be executed with `./build/tutorial/tutorial1_RobotController_bin`
When you also start the robot side `./build/tutorial/tutorial1_ControlledRobot_bin`, you can see both sides communicate.

## Connection Management

You might have noticed that the robot name may stay empty, when you start the controller side first, and the robot side was not started.

RRC with ZMQ will auto-connect and reconnect in the background, normally this is fine, but the static data might to be re-requested.
Also, the robot and your controller program (GUI) should know that a connection was lost.

To do this, the controller can request periodic pings from the robot.

**Without calling setHeartBeatDuration(), connection management is disabled**

```cpp
    // set Heartbeat to one 1/2 second
    controller.setHeartBeatDuration(0.5);

    controller.setupDisconnectedCallback([](const float &elapsed){
        printf("no lost connection since %.2f seconds\n", elapsed);
    });

    controller.setupConnectedCallback([](){
        printf("connected\n");
    });
    
    // wait for connection
    while (!controller.isConnected()) {
        printf("waiting for robot\n");
        usleep(1000000);
    }
```

When the controller activates the connection management, the callbacks on the robots side also become active:
The callback for connection losses, in this example allows 100ms of later arrival, due to differences in latency between heartbeat commands.
The elapsed time may be used to have different stages of escalation (slow down, stop, etc.). When there are multiple connections to this robots with different heartbeats in rare occations the logner heartbeat is used (connection loss (hight freq) right after the low freq time was send)

The setupHeartbeatCallbackInterval() defines the minimum time between executing the callback (even though more hearbeat messages are missing)
The Heartbeat may be prolonged by the controller during operation to allow the robot autonomous operation without connection.

```cpp
    robot.setupDisconnectedCallback(0.1, [](const float &elapsed){
        printf("no heartbeat since %.2f seconds\n", elapsed);
    });

    robot.setupConnectedCallback([](){
        printf("controller connected\n");
    });

    robot.setupHeartbeatCallbackInterval(2);
```

## Callbacks

## Channels

## Simple Actions

## Complex Actions

## Joints

## Robot Model and Files

## Permissions
