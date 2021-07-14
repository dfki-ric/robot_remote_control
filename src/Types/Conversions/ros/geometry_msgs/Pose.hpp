#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <geometry_msgs/PoseStamped.h>

namespace robot_remote_control {
namespace RosConversion {

    inline static void convert(const robot_remote_control::Pose &from, geometry_msgs::PoseStamped *to ) {
        to->header.stamp = ros::Time::now();
        to->header.frame_id = "";
        to->pose.position.x = from.position().x();
        to->pose.position.y = from.position().y();
        to->pose.position.z = from.position().z();

        to->pose.orientation.x = from.orientation().x();
        to->pose.orientation.y = from.orientation().y();
        to->pose.orientation.z = from.orientation().z();
        to->pose.orientation.w = from.orientation().w();
    }

    inline static void convert(const robot_remote_control::Pose &from, geometry_msgs::Pose *to ) {
        to->position.x = from.position().x();
        to->position.y = from.position().y();
        to->position.z = from.position().z();

        to->orientation.x = from.orientation().x();
        to->orientation.y = from.orientation().y();
        to->orientation.z = from.orientation().z();
        to->orientation.w = from.orientation().w();
    }

}  // namespace RockConversion
}  // namespace robot_remote_control
