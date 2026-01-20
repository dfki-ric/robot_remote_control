#include <iostream>
#include "RobotController.hpp"
#include "Transports/ZeroMQ/TransportZmq.hpp"
#include <unistd.h>
#include <chrono>


using robot_remote_control::TransportSharedPtr;
using robot_remote_control::TransportZmq;

int main(int argc, char** argv) {
    TransportSharedPtr commands = TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7001", TransportZmq::REQ));
    TransportSharedPtr telemetry = TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7002", TransportZmq::SUB));
    robot_remote_control::RobotController controller(commands, telemetry);

    // construct types
    robot_remote_control::Pose pose;

    // fill
    pose.mutable_position()->set_x(0);
    pose.mutable_position()->set_y(0);
    pose.mutable_position()->set_z(0);

    pose.mutable_orientation()->set_x(0);
    pose.mutable_orientation()->set_y(0);
    pose.mutable_orientation()->set_z(0);
    pose.mutable_orientation()->set_w(1);

    
    controller.startUpdateThread(10);

    float x = 0;

    robot_remote_control::WrenchState wstate;

    // set Heartbeat to one second
    controller.setHeartBeatDuration(1);

    controller.waitForConnection();

    
    for (int i = 0; i<=6000;++i){
    // while (true) {
        auto start = std::chrono::high_resolution_clock::now();
        controller.setTargetPose(pose);
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << std::endl;
    }
    return 0;
}
