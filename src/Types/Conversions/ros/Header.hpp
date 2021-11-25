#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <std_msgs/Header.h>


namespace robot_remote_control {
namespace RosConversion {

    static void convert(const std_msgs::Header &from, robot_remote_control::TimeStamp *to) {
        to->set_secs(from.stamp.sec);
        to->set_nsecs(from.stamp.nsec);
    }


}  // namespace RosConversion
}  // namespace robot_remote_control
