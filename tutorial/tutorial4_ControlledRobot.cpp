#include "ControlledRobot.hpp"

#include <iostream>
#include <unistd.h>

#include "Transports/TransportZmq.hpp"

using robot_remote_control::TransportSharedPtr;
using robot_remote_control::TransportZmq;

int main(int argc, char** argv)
{
    TransportSharedPtr commands = TransportSharedPtr(new TransportZmq("tcp://*:7001", TransportZmq::REP));
    TransportSharedPtr telemetry = TransportSharedPtr(new TransportZmq("tcp://*:7002", TransportZmq::PUB));
    robot_remote_control::ControlledRobot robot(commands, telemetry);

    // start receive thread
    robot.startUpdateThread(0);

    // set a robot name
    robot.initRobotName("TestRobot");

    // connection manegement
    robot.setupDisconnectedCallback(0.1, [](const float &elapsed){
        printf("no heartbeat since %.2f seconds\n", elapsed);
    });

    robot.setupConnectedCallback([](){
        printf("controller connected\n");
    });

    
    robot.setDefaultChannelName(robot_remote_control::CURRENT_TWIST, "default channel");
    int twist_2nd_channel = robot.addChannel(robot_remote_control::CURRENT_TWIST, "another channel");

    // commands to receive
    robot_remote_control::Twist twist_command;

    // robot data
    robot_remote_control::Twist twist_telemetry, twist_telemetry2;

    twist_telemetry2.mutable_header()->set_frame("2nd value");

    robot.addCommandReceivedCallback(robot_remote_control::TWIST_COMMAND, [&]() {
        robot.getTwistCommand(&twist_command);
        twist_command.PrintDebugString();
        // we don't have real robot data, so we just write the value back
        twist_telemetry = twist_command;
    });

    while (true) {

        // here would be the interaction with the robot software
        
        // write command back (in lack of real data)
        robot.setCurrentTwist(twist_telemetry);
        // write other data on other channel
        robot.setCurrentTwist(twist_telemetry2, twist_2nd_channel);

        usleep(100000);
    }
    return 0;
}


