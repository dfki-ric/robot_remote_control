#include <iostream>
#include <interaction-library-controlled_robot/ControlledRobot.hpp>

int main(int argc, char** argv)
{
    interaction::ControlledRobot robot("tcp://*:7001");
    

    while (true){
        robot.update();

        //fake-write requested pose to curretn pose
        robot.setCurrentPose(robot.getTargetPose());
    }


    return 0;
}
