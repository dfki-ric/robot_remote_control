#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include "../Header.hpp"

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

    static void convert(const robot_remote_control::Pose &from, geometry_msgs::msg::TransformStamped *to) {
        convert(from.header(), &to->header);

        robot_remote_control::Position pos = from.position();
        to->transform.translation.x = pos.x();
        to->transform.translation.y = pos.y();
        to->transform.translation.z = pos.z();

        robot_remote_control::Orientation quat = from.orientation();
        to->transform.rotation.x = quat.x();
        to->transform.rotation.y = quat.y();
        to->transform.rotation.z = quat.z();
        to->transform.rotation.w = quat.w();
    }

    static void convert(const robot_remote_control::Transform &from, geometry_msgs::msg::TransformStamped *to) {
        convert(from.header(), &to->header);

        robot_remote_control::Pose pose = from.transform();
        convert(pose, to);

        to->header.frame_id = from.from();
        to->child_frame_id = from.to();
    }


}  // namespace RosConversion
}  // namespace robot_remote_control
