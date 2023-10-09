#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include "../Header.hpp"

namespace robot_remote_control {
namespace RosConversion {

    static void convert(const sensor_msgs::msg::JointState &from, robot_remote_control::JointState* to) {
        convert(from.header, to->mutable_header());

        for (int i = 0; i < from.name.size(); ++i) {
            to->add_name(from.name[i]);
            if (from.position.size()) {
                to->add_position(from.position[i]);
            }
            if (from.velocity.size()) {
                to->add_velocity(from.velocity[i]);
            }
            if (from.effort.size()) {
                to->add_effort(from.effort[i]);
            }
        }
    }

    static void convert(const robot_remote_control::JointState &from, sensor_msgs::msg::JointState* to) {
        convert(from.header(), &to->header);

        for (int i = 0; i < from.name_size(); ++i) {
            to->name.push_back(from.name(i));
            if (from.position_size()) {
                to->position.push_back(from.position(i));
            }
            if (from.velocity_size()) {
                to->velocity.push_back(from.velocity(i));
            }
            if (from.effort_size()) {
                to->effort.push_back(from.effort(i));
            }
        }
    }

}  // namespace RosConversion
}  // namespace robot_remote_control
