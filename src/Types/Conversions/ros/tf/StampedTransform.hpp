#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <tf/transform_datatypes.h>
#include "../Header.hpp"

namespace robot_remote_control {
namespace RosConversion {

    static void convert(const tf::StampedTransform &from, robot_remote_control::Pose *to) {
        convert(from.header, to->mutable_header());

        tf::Vector3 pos = from.getOrigin();
        robot_remote_control::Position *rrc_pos = to->mutable_position();
        rrc_pos->set_x(pos.getX());
        rrc_pos->set_y(pos.getY());
        rrc_pos->set_z(pos.getZ());

        tf::Quaternion quat = from.getRotation();
        robot_remote_control::Orientation *rrc_quat = to->mutable_orientation();
        rrc_quat->set_x(quat.getX());
        rrc_quat->set_y(quat.getY());
        rrc_quat->set_z(quat.getZ());
        rrc_quat->set_w(quat.getW());
    }


}  // namespace RosConversion
}  // namespace robot_remote_control
