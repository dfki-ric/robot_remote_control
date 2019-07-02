#include <iostream>
#include <interaction-library-controlled_robot/RobotController.hpp>
#include <interaction-library-controlled_robot/Transports/TransportZmq.hpp>
#include <unistd.h>


using namespace interaction;

int main(int argc, char** argv)
{
    TransportSharedPtr commands = TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7001",TransportZmq::REQ));
    TransportSharedPtr telemetry = TransportSharedPtr(new TransportZmq("tcp://127.0.0.1:7002",TransportZmq::SUB));
    RobotController controller(commands,telemetry);
    
    //consruct type
    Position position;
    Orientation orientation;
    Pose pose;

    //fill
    position.set_x(0);
    position.set_y(0);
    position.set_z(0);

    orientation.set_x(0);
    orientation.set_y(0);
    orientation.set_z(0);
    orientation.set_w(1);
    
    *(pose.mutable_position()) = position;
    *(pose.mutable_orientation()) = orientation;        

    interaction::Pose currentpose;
    interaction::JointState jointstate;

    interaction::Twist twistcommand;
    twistcommand.mutable_angular()->set_z(0.1);
    twistcommand.mutable_linear()->set_x(0.1);
    


    float x = 0;

    while (true){
        controller.setTargetPose(pose);
        //controller.setTwistCommand(twistcommand);

        //receive pending telemetry
        controller.updateTelemetry();
        int buffersize_pose = controller.getCurrentPose(currentpose);
        int buffersize_joint = controller.getCurrentJointState(jointstate);

        pose.mutable_position()->set_x(x);
        x += 0.01;

        

        printf("%.2f %.2f %.2f\n",currentpose.position().x(),currentpose.position().y(),currentpose.position().z());

        printf("got %i joints\n",jointstate.name_size());
        google::protobuf::RepeatedPtrField<std::string> names = jointstate.name();

        printf("buffer sizes %i, %i\n",buffersize_pose, buffersize_joint);


        usleep(10000);
    }
    



    return 0;
}
