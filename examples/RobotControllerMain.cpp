#include <iostream>
#include "RobotController.hpp"
#include <Transports/ZeroMQ/TransportZmq.hpp>
// #include "Transports/WebSocket/TransportWebSocket.hpp"
// #include "Transports/Http/TransportHttp.hpp"
#include <unistd.h>


using robot_remote_control::TransportSharedPtr;
using robot_remote_control::TransportZmq;
// using robot_remote_control::TransportWebSocket;
// using robot_remote_control::TransportHttp;

int main(int argc, char** argv) {
    TransportSharedPtr commands = TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7001", TransportZmq::REQ));
    TransportSharedPtr telemetry = TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7002", TransportZmq::SUB));

    // TransportSharedPtr commands = TransportSharedPtr(new TransportWebSocket(TransportWebSocket::CLIENT_TEXT, 7001, "localhost"));
    // TransportSharedPtr telemetry = TransportSharedPtr(new TransportWebSocket(TransportWebSocket::CLIENT_TEXT, 7002, "localhost"));
    
    // TransportSharedPtr commands = TransportSharedPtr(new TransportHttp("http://localhost:7001", TransportHttp::CLIENT));

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


    controller.startUpdateThread(10);

    float x = 0;

    robot_remote_control::WrenchState wstate;

    // set Heartbeat to one second
    controller.setHeartBeatDuration(1);

    controller.setupDisconnectedCallback([](const float &elapsed){
        printf("no lost connection since %.2f seconds\n", elapsed);
    });

    controller.setupConnectedCallback([](){
        printf("connected\n");
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
        if (controller.getCurrentPose(&currentpose), true) {
            //read all poses from the buffer
            printf("Current Pose: %s\n", currentpose.ShortDebugString().c_str());
        }

        bool new_joint = controller.getCurrentJointState(&jointstate);



        //printf("%.2f %.2f %.2f\n", currentpose.position().x(), currentpose.position().y(), currentpose.position().z());
        
        if (new_joint) {
            printf("got %i joints\n", jointstate.name_size());
        }
        google::protobuf::RepeatedPtrField<std::string> names = jointstate.name();


        robot_remote_control::SimpleSensor sens;
        controller.getSimpleSensor(&sens);

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


        printf("latency %f seconds\n", controller.getHeartBeatRoundTripTime()/2.0);


        usleep(10000);
    }

    return 0;
}
