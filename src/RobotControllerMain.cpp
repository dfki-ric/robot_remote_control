#include <iostream>
#include <interaction-library-controlled_robot/RobotController.hpp>
#include <interaction-library-controlled_robot/Transports/TransportZmq.hpp>
#include <unistd.h>

int main(int argc, char** argv)
{
    std::shared_ptr<interaction::Transport> commands = std::shared_ptr<interaction::Transport>(new interaction::TransportZmq("tcp://127.0.0.1:7001",interaction::TransportZmq::REQ));
    std::shared_ptr<interaction::Transport> telemetry = std::shared_ptr<interaction::Transport>(new interaction::TransportZmq("tcp://127.0.0.1:7002",interaction::TransportZmq::SUB));
    interaction::RobotController controller(commands,telemetry);
    
    //consruct type
    interaction::Position position;
    interaction::Orientation orientation;
    interaction::Pose pose;

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

    interaction::Twist twistcommand;
    twistcommand.mutable_angular()->set_z(0.1);
    twistcommand.mutable_linear()->set_x(0.1);
    


    float x = 0;

    while (true){
        //controller.setTargetPose(pose);
        controller.setTwistCommand(twistcommand);
        currentpose = controller.getCurrentPose();

        pose.mutable_position()->set_x(x);
        x += 0.01;

        printf("%.2f %.2f %.2f\n",currentpose.position().x(),currentpose.position().y(),currentpose.position().z());




        usleep(100000);
    }
    



    return 0;
}
