#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <std_msgs/Header.h>


namespace robot_remote_control {
namespace RosConversion {

    static void convert(const std_msgs::Header &from, robot_remote_control::TimeStamp *to) {
        to->set_secs(from.stamp.sec);
        to->set_nsecs(from.stamp.nsec);
    }

    static convert(const ros::Time& from, robot_remote_control::TimeStamp *to) {
        to->set_secs(from.sec);
        to->set_nsecs(from.nsec);
    }

    static void convert(const std_msgs::Header &from, robot_remote_control::Header *to) {
        to->set_seq(from.seq);
        to->set_frame(from.frame_id);
        convert(from.stamp, to->mutable_timestamp());
    }

    static void convert(const robot_remote_control::Header &from, std_msgs::Header *to) {
        to->seq = from.seq();
        to->frame_id = from.frame();
        to->stamp.sec = from.timestamp().secs();
        to->stamp.nsec = from.timestamp().nsecs();
    }

}  // namespace RosConversion
}  // namespace robot_remote_control
