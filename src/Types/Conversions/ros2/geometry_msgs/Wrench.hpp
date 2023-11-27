#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <geometry_msgs/msg/wrench_stamped.h>
#include "../Header.hpp"

namespace robot_remote_control {
namespace RosConversion {

    static void convert(const geometry_msgs::msg::WrenchStamped &from, robot_remote_control::WrenchState* to) {
        robot_remote_control::Wrench *pb_wrench = to->add_wrenches();
        convert(from.header, pb_wrench->mutable_header());

        pb_wrench->mutable_force()->set_x(from.wrench.force.x);
        pb_wrench->mutable_force()->set_y(from.wrench.force.y);
        pb_wrench->mutable_force()->set_z(from.wrench.force.z);

        pb_wrench->mutable_torque()->set_x(from.wrench.torque.x);
        pb_wrench->mutable_torque()->set_y(from.wrench.torque.y);
        pb_wrench->mutable_torque()->set_z(from.wrench.torque.z);
    }

    static void convert(const robot_remote_control::WrenchState &from, geometry_msgs::msg::WrenchStamped* to) {
        convert(from.wrenches(0).header(), &to->header);

        to->wrench.force.x = from.wrenches(0).force().x();
        to->wrench.force.y = from.wrenches(0).force().y();
        to->wrench.force.z = from.wrenches(0).force().z();

        to->wrench.torque.x = from.wrenches(0).torque().x();
        to->wrench.torque.y = from.wrenches(0).torque().y();
        to->wrench.torque.z = from.wrenches(0).torque().z();
    }

}  // namespace RosConversion
}  // namespace robot_remote_control
