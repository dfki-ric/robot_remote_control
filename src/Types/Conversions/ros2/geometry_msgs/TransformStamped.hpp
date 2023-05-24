#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

namespace robot_remote_control {
namespace RosConversion {

    static void convert(const geometry_msgs::msg::TransformStamped &from, robot_remote_control::Pose *to) {
        convert(from.header, to->mutable_header());

        geometry_msgs::msg::Vector3 pos = from.transform.translation;
        robot_remote_control::Position *rrc_pos = to->mutable_position();
        rrc_pos->set_x(pos.x);
        rrc_pos->set_y(pos.y);
        rrc_pos->set_z(pos.z);

        geometry_msgs::msg::Quaternion quat = from.transform.rotation;
        robot_remote_control::Orientation *rrc_quat = to->mutable_orientation();
        rrc_quat->set_x(quat.x);
        rrc_quat->set_y(quat.y);
        rrc_quat->set_z(quat.z);
        rrc_quat->set_w(quat.w);
    }


}  // namespace RosConversion
}  // namespace robot_remote_control
