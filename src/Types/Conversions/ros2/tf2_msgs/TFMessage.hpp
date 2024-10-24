#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include "../Header.hpp"

namespace robot_remote_control {
namespace RosConversion {

    static void convert(const robot_remote_control::Transforms &from, tf2_msgs::msg::TFMessage *to) {
        for (int i = 0; i < from.transform_size(); i++) {
            const robot_remote_control::Transform &rrc_tf = from.transform(i);
            geometry_msgs::msg::TransformStamped tf;
            convert(rrc_tf, &tf);
            to->transforms.push_back(tf);
        }
    }


}  // namespace RosConversion
}  // namespace robot_remote_control
