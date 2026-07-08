
#include "ControlledRobot.hpp"

#include <unistd.h>
#include <iostream>

#include "Transports/ZeroMQ/TransportZmq.hpp"

using robot_remote_control::TransportSharedPtr;
using robot_remote_control::TransportZmq;


int main(int argc, char** argv)
{
    TransportSharedPtr commands = TransportSharedPtr(new TransportZmq("tcp://*:7001", TransportZmq::REP));
    TransportSharedPtr telemetry = TransportSharedPtr(new TransportZmq("tcp://*:7002", TransportZmq::PUB));
    robot_remote_control::ControlledRobot robot(commands, telemetry);


 
    robot.startUpdateThread(10);

    // set a callback for connection losses, allow 100ms of later arrival
    // (due to differences in latency between heartbeat commands)
    // the elapsed time may be used to have different stages of escalation
    // when there are multiple connections to this robots with different heartbeats
    // in rare occations the logner heartbeat is used (connection loss (hight freq) right after the low freq time was send)

    bool running = true;
    int counter = 0;
    // robot.setupHeartbeatCallback(0.5, [&](const float &elapsed){
    //     printf("test finished\n");
    //     if counter 
    //     running=false;
    // });

    robot.waitForConnection();

    robot.addCommandReceivedCallback(robot_remote_control::TARGET_POSE_COMMAND, []() {
        // WARNING: this callback run in the reveive thread, you should not use this to access data, only to notify other threads
        //printf("Pose Command Callback\n");
    });

    robot_remote_control::Pose pose;
    
    robot.addCommandReceivedCallback(robot_remote_control::TARGET_POSE_COMMAND, [&](){
        ++counter;
        if (counter == 2000) {
             printf("switching scheduler\n");
             if (!robot.setUpdateThreadPriority(99, SCHED_FIFO)) {
                 printf("error setting thread priority\n");
            };
        }
        if (counter == 4000) {
             printf("switching scheduler\n");
             if (!robot.setUpdateThreadPriority(99, SCHED_RR)) {
                 printf("error setting thread priority\n");
            };
        }
        if (counter>=6000) {
            // exit(0);
            running=false;
        }
    });

    while(running) {
        usleep(100000);
    }

}


