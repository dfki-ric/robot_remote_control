#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <geometry_msgs/Vector3.h>

namespace robot_remote_control {
namespace RosConversion {

    inline static void convert(const robot_remote_control::Vector3 &from, geometry_msgs::Vector3 *to ) {
        to->x = from.x();
        to->y = from.y();
        to->z = from.z();
    }

    inline static void convert(const geometry_msgs::Vector3 &from, robot_remote_control::Vector3 *to ) {
        to->set_x(from.x);
        to->set_y(from.y);
        to->set_z(from.z);
    }

}  // namespace RockConversion
}  // namespace robot_remote_control
