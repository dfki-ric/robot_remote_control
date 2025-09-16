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

    static void convert(const robot_remote_control::WrenchState &from, geometry_msgs::msg::WrenchStamped* to, size_t index = 0) {
        convert(from.wrenches(index).header(), &to->header);

        to->wrench.force.x = from.wrenches(index).force().x();
        to->wrench.force.y = from.wrenches(index).force().y();
        to->wrench.force.z = from.wrenches(index).force().z();

        to->wrench.torque.x = from.wrenches(index).torque().x();
        to->wrench.torque.y = from.wrenches(index).torque().y();
        to->wrench.torque.z = from.wrenches(index).torque().z();
    }

}  // namespace RosConversion
}  // namespace robot_remote_control
