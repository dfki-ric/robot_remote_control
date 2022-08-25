
#include "ControlledRobot.hpp"

#include <unistd.h>
#include <iostream>

#include "Transports/TransportZmq.hpp"

using robot_remote_control::TransportSharedPtr;
using robot_remote_control::TransportZmq;


int main(int argc, char** argv)
{
    TransportSharedPtr commands = TransportSharedPtr(new TransportZmq("tcp://*:7001", TransportZmq::REP));
    TransportSharedPtr telemetry = TransportSharedPtr(new TransportZmq("tcp://*:7002", TransportZmq::PUB));
    robot_remote_control::ControlledRobot robot(commands, telemetry);

    robot.startUpdateThread(0);

    // set a callback for connection losses, allow 100ms of later arrival
    // (due to differences in latency between heartbeat commands)
    // the elapsed time may be used to have different stages of escalation
    // when there are multiple connections to this robots with different heartbeats
    // in rare occations the logner heartbeat is used (connection loss (hight freq) right after the low freq time was send)
    robot.setupHeartbeatCallback(0.1, [](const float &elapsed){
        printf("no heartbeat since %.2f seconds\n", elapsed);
    });

    
    // for requests to work, you need a valid connection:
    // only works when heartbeats are set up
    while (!robot.isConnected()) {
        printf("waiting for connection\n");
        usleep(100000);
    }

    robot.addCommandReceivedCallback(robot_remote_control::TARGET_POSE_COMMAND, []() {
        // WARNING: this callback run in the reveive thread, you should not use this to access data, only to notify other threads
        //printf("Pose Command Callback\n");
    });


    while (true) {

        //do nothing

        usleep(100000);
    }


    return 0;
}


