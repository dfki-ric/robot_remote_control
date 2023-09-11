#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <geometry_msgs/WrenchStamped.h>
#include "../Header.hpp"

namespace robot_remote_control {
namespace RosConversion {

    static void convert(const geometry_msgs::WrenchStamped &from, robot_remote_control::WrenchState* to) {
        robot_remote_control::Wrench *pb_wrench = to->add_wrenches();
        convert(from.header, pb_wrench->mutable_header());

        pb_wrench->mutable_force()->set_x(from.wrench.force.x);
        pb_wrench->mutable_force()->set_y(from.wrench.force.y);
        pb_wrench->mutable_force()->set_z(from.wrench.force.z);

        pb_wrench->mutable_torque()->set_x(from.wrench.torque.x);
        pb_wrench->mutable_torque()->set_y(from.wrench.torque.y);
        pb_wrench->mutable_torque()->set_z(from.wrench.torque.z);
    }


}  // namespace RosConversion
}  // namespace robot_remote_control
