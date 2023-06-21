#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include "../Header.hpp"
#include "../geometry_msgs/Pose.hpp"
#include <nav_msgs/msg/path.hpp>

namespace robot_remote_control {
namespace RosConversion {

    static void convert(const nav_msgs::msg::Path &from, robot_remote_control::Poses* to) {

        convert(from.header, to->mutable_header());

        if(from.header.frame_id.empty()) {
            to->set_frame("world");
        }

        for (const auto & pose : from.poses) {
            convert(pose, to->add_poses());
        }
    }

    static void convert(const robot_remote_control::Poses &from, nav_msgs::msg::Path* to) {
        for (unsigned int i = 0; i < from.poses_size(); i++) {
            convert(from.poses(i), &to->poses[i]);
        }
    }

}  // namespace RosConversion
}  // namespace robot_remote_control
