#include <iostream>
#include "RobotController.hpp"
#include "Transports/TransportZmq.hpp"
#include <unistd.h>


using robot_remote_control::TransportSharedPtr;
using robot_remote_control::TransportZmq;

int main(int argc, char** argv) {
    TransportSharedPtr commands = TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7001", TransportZmq::REQ));
    TransportSharedPtr telemetry = TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7002", TransportZmq::SUB));
    robot_remote_control::RobotController controller(commands, telemetry);

    // consruct type
    robot_remote_control::Position position;
    robot_remote_control::Orientation orientation;
    robot_remote_control::Pose pose;

    // fill
    position.set_x(0);
    position.set_y(0);
    position.set_z(0);

    orientation.set_x(0);
    orientation.set_y(0);
    orientation.set_z(0);
    orientation.set_w(1);

    *(pose.mutable_position()) = position;
    *(pose.mutable_orientation()) = orientation;

    robot_remote_control::Pose currentpose;
    robot_remote_control::JointState jointstate;

    robot_remote_control::Twist twistcommand;
    twistcommand.mutable_angular()->set_z(0.1);
    twistcommand.mutable_linear()->set_x(0.1);


    controller.startUpdateThread(100);

    float x = 0;

    while (true) {
        controller.setTargetPose(pose);
        // controller.setTwistCommand(twistcommand);

        // receive pending telemetry
        // controller.update();

        // in production, the get function should use a while loop
        // while (controller.getCurrentPose(currentpose)){do stuff}
        int buffersize_pose = controller.getCurrentPose(&currentpose);
        int buffersize_joint = controller.getCurrentJointState(&jointstate);

        pose.mutable_position()->set_x(x);
        x += 0.01;


        printf("%.2f %.2f %.2f\n", currentpose.position().x(), currentpose.position().y(), currentpose.position().z());

        printf("got %i joints\n", jointstate.name_size());
        google::protobuf::RepeatedPtrField<std::string> names = jointstate.name();


        printf("buffer sizes %i, %i\n", buffersize_pose, buffersize_joint);


        usleep(10000);
    }

    return 0;
}
