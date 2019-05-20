#include <iostream>
#include <interaction-library-controlled_robot/RobotController.hpp>

#include <unistd.h>

int main(int argc, char** argv)
{
    interaction::RobotController controller("tcp://127.0.0.1:7001");
    
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

    float x = 0;

    while (true){
        controller.setTargetPose(pose);
        currentpose = controller.getCurrentPose();

        pose.mutable_position()->set_x(x);
        x += 0.01;

        printf("%.2f %.2f %.2f\n",currentpose.position().x(),currentpose.position().y(),currentpose.position().z());

        usleep(1000000);
    }
    



    return 0;
}
