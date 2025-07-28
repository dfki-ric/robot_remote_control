#include "RobotController.hpp"

#include <iostream>
#include <unistd.h>

#include "Transports/TransportZmq.hpp"

using robot_remote_control::TransportSharedPtr;
using robot_remote_control::TransportZmq;

int main(int argc, char** argv)
{
    TransportSharedPtr commands = TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7001", TransportZmq::REQ));
    TransportSharedPtr telemetry = TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7002", TransportZmq::SUB));
    robot_remote_control::RobotController controller(commands, telemetry);

    // start receive thread
    controller.startUpdateThread(0);

    robot_remote_control::RobotName robotname;
    controller.requestRobotName(&robotname);

    printf("connected to:\n");
    robotname.PrintDebugString();

    // set Heartbeat to one 1/2 second
    controller.setHeartBeatDuration(2);

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

    // commands to send
    robot_remote_control::Twist twist_command;

    // robot data
    robot_remote_control::Twist twist_telemetry, twist_telemetry2;

    // print new data using callback instaed of main loop
    controller.addTelemetryReceivedCallback<robot_remote_control::Twist>(robot_remote_control::CURRENT_TWIST, [&](const robot_remote_control::Twist& twist) { 
        std::string tele = twist.ShortDebugString();
        printf("channel0: %s\n", tele.c_str());
    });

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

        if (controller.getCurrentTwist(&twist_telemetry2, true, 1)) {
            std::string tele = twist_telemetry2.ShortDebugString();
            printf("channel1: %s\n", tele.c_str());
        }

        // send the command
        controller.setTwistCommand(twist_command);

        usleep(100000);
    }

    return 0;
}
