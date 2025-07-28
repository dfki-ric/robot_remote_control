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

    // set a callback for connection losses, allow 100ms of later arrival
    // (due to differences in latency between heartbeat commands)
    // the elapsed time may be used to have different stages of escalation
    // when there are multiple connections to this robots with different heartbeats
    // in rare occations the logner heartbeat is used (connection loss (hight freq) right after the low freq time was send)
    robot.setupDisconnectedCallback(0.1, [](const float &elapsed){
        printf("no heartbeat since %.2f seconds\n", elapsed);
    });

    robot.setupConnectedCallback([](){
        printf("controller connected\n");
    });


    // commands to receive
    robot_remote_control::Twist twist_command;

    // robot data
    robot_remote_control::Twist twist_telemetry;

    while (true) {

        // receive 
        if (robot.getTwistCommand(&twist_command)) {
            twist_command.PrintDebugString();
            
            // we don't have real robot data, so we just write the value back
            twist_telemetry = twist_command;
        }

        // here would be the interaction with the robot software
        
        // write command back (in lack of real data)
        robot.setCurrentTwist(twist_telemetry);

        usleep(100000);
    }
    return 0;
}


