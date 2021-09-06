#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <geometry_msgs/Quaternion.h>

namespace robot_remote_control {
namespace RosConversion {

    inline static void convert(const robot_remote_control::Orientation &from, geometry_msgs::Quaternion *to ) {
        to->x = from.x();
        to->y = from.y();
        to->z = from.z();
        to->w = from.w();
    }

    inline static void convert(const geometry_msgs::Quaternion &from, robot_remote_control::Orientation *to ) {
        to->set_x(from.x);
        to->set_y(from.y);
        to->set_z(from.z);
        to->set_w(from.w);
    }

}  // namespace RosConversion
}  // namespace robot_remote_control
