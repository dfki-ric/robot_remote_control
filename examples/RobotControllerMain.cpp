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

    // construct types
    robot_remote_control::Position position;
    robot_remote_control::Orientation orientation;
    robot_remote_control::Pose pose;
    robot_remote_control::Transforms transforms;

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

    robot_remote_control::WrenchState wstate;

    // set Heartbeat to one second
    controller.setHeartBeatDuration(1);

    controller.addTelemetryReceivedCallback<robot_remote_control::Pose>(robot_remote_control::CURRENT_POSE, [](const size_t buffersize, const robot_remote_control::Pose &pose) {
        // WARNING: this callback rund in the reveive thread, you cannot use this to access data, only to notify other threads
        printf("Current Pose callback: %s\n", pose.ShortDebugString().c_str());
    });


    while (true) {

        controller.setTargetPose(pose);
        pose.mutable_position()->set_x(x);
        x += 0.01;
        
        // controller.setTwistCommand(twistcommand);

        // receive pending telemetry
        // controller.update();

        // in production, the get function should use a while loop
        // while (controller.getCurrentPose(currentpose)){do stuff}
        while (controller.getCurrentPose(&currentpose)) {
            //read all poses from the buffer
        }
        bool new_joint = controller.getCurrentJointState(&jointstate);



        //printf("%.2f %.2f %.2f\n", currentpose.position().x(), currentpose.position().y(), currentpose.position().z());
        printf("Current Pose: %s\n", currentpose.ShortDebugString().c_str());

        printf("got %i joints\n", jointstate.name_size());
        google::protobuf::RepeatedPtrField<std::string> names = jointstate.name();


        robot_remote_control::SimpleSensor sens;
        controller.getSimpleSensor(1, &sens);

        // read only one element (no matter how much available)
        if (controller.getCurrentTransforms(&transforms)) {
            transforms.PrintDebugString();
        }

        robot_remote_control::PermissionRequest permreq;
        if (controller.getPermissionRequest(&permreq)) {
            permreq.PrintDebugString();
            robot_remote_control::Permission permission;
            permission.set_requestuid(permreq.requestuid());
            if (permreq.description() == "test2") {
                permission.set_granted(false);
            } else {
                permission.set_granted(true);
            }
            controller.setPermission(permission);
        }

        // read all remaining and only print if new
        bool hasnewWrenchState = false;
        while (controller.getCurrentWrenchState(&wstate)) { hasnewWrenchState=true; }
        if (hasnewWrenchState) {
            wstate.PrintDebugString();
        }


        printf("latency %f seconds\n", controller.getHeartBreatRoundTripTime()/2.0);


        usleep(10000);
    }

    return 0;
}
