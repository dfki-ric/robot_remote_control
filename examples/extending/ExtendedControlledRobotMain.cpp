
#include "ExtendedControlledRobot.hpp"

#include <unistd.h>
#include <iostream>

#include "Transports/TransportZmq.hpp"

using robot_remote_control::TransportSharedPtr;
using robot_remote_control::TransportZmq;


int main(int argc, char** argv) {
    TransportSharedPtr commandsconnection = TransportSharedPtr(new TransportZmq("tcp://*:7001", TransportZmq::REP));
    TransportSharedPtr telemetryconnection = TransportSharedPtr(new TransportZmq("tcp://*:7002", TransportZmq::PUB));
    robot_remote_control::ExtendedControlledRobot robot(commandsconnection, telemetryconnection);

    robot.startUpdateThread(100);

    robot_remote_control::RobotName name;
    name.set_value("ExtendedTestRobot");
    robot.initRobotName(name);


    myrobot::NewControlMessage control;
    myrobot::NewTelemetryMessage telemetry;


    while (true) {
        
        robot.getNewControlMessage(&control);
        std::cout << "recv:" << std::endl;
        control.PrintDebugString();

        std::cout << "send:" << std::endl;
        telemetry.set_sequence_no(control.sequence_no());
        robot.setNewTelemetryMessage(telemetry);
        telemetry.PrintDebugString();

        usleep(100000);
    }


    return 0;
}


