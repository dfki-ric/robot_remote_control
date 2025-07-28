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

        if (controller.getCurrentTwist(&twist_telemetry)){
            twist_telemetry.PrintDebugString();
        }

        usleep(1000000);
    }

    return 0;
}
