#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <nav_msgs/msg/odometry.hpp>
#include "../Header.hpp"
#include "../geometry_msgs/Pose.hpp"
#include "../geometry_msgs/Twist.hpp"

namespace robot_remote_control {
namespace RosConversion {

    static void convert(const nav_msgs::msg::Odometry &from, robot_remote_control::Pose* to) {
        convert(from.header, to->mutable_header());
        convert(from.pose.pose, to);
    }

    static void convert(const nav_msgs::msg::Odometry &from, robot_remote_control::Odometry* to) {
        convert(from.header, to->mutable_header());
        convert(from.pose, to->mutable_pose());
        convert(from.twist, to->mutable_twist());
        to->set_child_frame_id(from.child_frame_id);
    }

}  // namespace RosConversion
}  // namespace robot_remote_control