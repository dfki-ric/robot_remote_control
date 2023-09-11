#pragma once

#include <string>

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "../Header.hpp"
#include "Point.hpp"
#include "Quaternion.hpp"

namespace robot_remote_control {
namespace RosConversion {

    inline static void convert(const robot_remote_control::Pose &from, geometry_msgs::msg::Pose *to ) {
        convert(from.position(), &to->position);
        convert(from.orientation(), &to->orientation);
    }
    
    inline static void convert(const robot_remote_control::Pose &from, geometry_msgs::msg::PoseStamped *to) {
        convert(from.header(), &to->header);
        convert(from, &to->pose);
    }

    inline static void convert(const geometry_msgs::msg::Pose &from, robot_remote_control::Pose *to ) {
        convert(from.position, to->mutable_position());
        convert(from.orientation, to->mutable_orientation());
    }
    
    inline static void convert(const geometry_msgs::msg::PoseStamped &from, robot_remote_control::Pose *to ) {
        convert(from.header, to->mutable_header());
        convert(from.pose, to);
    }
}  // namespace RosConversion
}  // namespace robot_remote_control
