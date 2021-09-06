#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <nav_msgs/Odometry.h>

namespace robot_remote_control {
namespace RosConversion {

    static void convert(const nav_msgs::Odometry &from, robot_remote_control::Pose* to) {
        to->mutable_position()->set_x(from.pose.pose.position.x);
        to->mutable_position()->set_y(from.pose.pose.position.y);
        to->mutable_position()->set_z(from.pose.pose.position.z);

        to->mutable_orientation()->set_x(from.pose.pose.orientation.x);
        to->mutable_orientation()->set_y(from.pose.pose.orientation.y);
        to->mutable_orientation()->set_z(from.pose.pose.orientation.z);
        to->mutable_orientation()->set_w(from.pose.pose.orientation.w);
    }

}  // namespace RosConversion
}  // namespace robot_remote_control
