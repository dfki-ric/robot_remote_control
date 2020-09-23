#pragma once

#include "RobotController/RobotController.hpp"

namespace robot_remote_control{

/**
 * @brief hides the cpp protobuf datatypes from cython
 * 
 */
class RobotControllerWrapper {
 public:
    RobotControllerWrapper(RobotController* controller):controller(controller){}

    bool getCurrentPose(std::string* data) {
        Pose pos;

        int res = controller->getCurrentPose(&pos);
        pos.PrintDebugString();
        printf("bufsize %i\n",res);

        if (res) {
            printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
        }else{
            printf("no data\n");
        }
        pos.SerializeToString(data);
        return res;
    }

 private:
    RobotController* controller;

};

}  // namespace robot_remote_control