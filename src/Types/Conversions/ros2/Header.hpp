#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <std_msgs/msg/header.hpp>


namespace robot_remote_control {
namespace RosConversion {

    // static void convert(const std_msgs::msg::Header &from, robot_remote_control::Header *to) {
    //     to->mutable_timestamp()->set_secs(from.stamp.sec);
    //     to->mutable_timestamp()->set_nsecs(from.stamp.nanosec);
    // }


    static void convert(const std_msgs::msg::Header &from, robot_remote_control::Header *to) {
        to->set_frame(from.frame_id);
        to->mutable_timestamp()->set_secs(from.stamp.sec);
        to->mutable_timestamp()->set_nsecs(from.stamp.nanosec);
        
    }

    static void convert(const robot_remote_control::Header &from, std_msgs::msg::Header *to) {
        to->frame_id = from.frame();
        to->stamp.sec = from.timestamp().secs();
        to->stamp.nanosec = from.timestamp().nsecs();
    }

}  // namespace RosConversion
}  // namespace robot_remote_control
