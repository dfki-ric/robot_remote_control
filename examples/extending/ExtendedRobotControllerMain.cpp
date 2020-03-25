#include <unistd.h>
#include <iostream>
#include "ExtendedRobotController.hpp"
#include "Transports/TransportZmq.hpp"



using robot_remote_control::TransportSharedPtr;
using robot_remote_control::TransportZmq;

int main(int argc, char** argv) {
    TransportSharedPtr commandsconnection = TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7001", TransportZmq::REQ));
    TransportSharedPtr telemetryconnection = TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7002", TransportZmq::SUB));
    robot_remote_control::ExtendedRobotController controller(commandsconnection, telemetryconnection);

    // consruct type
    myrobot::NewControlMessage control;
    control.set_msg("testmessage");
    control.set_sequence_no(0);

    myrobot::NewTelemetryMessage telemetry;
    
    controller.startUpdateThread(100);

    float x = 0;
    int iteration = 0;


    robot_remote_control::RobotName name;
    controller.requestRobotName(&name);

    std::cout << "connected to " << name.value() << std::endl;

    while (true) {
        control.set_sequence_no(iteration++);
        controller.setNewControlMessage(control);

        controller.getNewTelemetryMessage(&telemetry);
        std::cout << "get" << std::endl;
        telemetry.PrintDebugString();

        controller.requestNewTelemetryMessage(&telemetry);
        std::cout << "request" << std::endl;
        telemetry.PrintDebugString();



        usleep(1000000);
    }

    return 0;
}
