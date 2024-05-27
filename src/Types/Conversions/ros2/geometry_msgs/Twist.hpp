#pragma once

#include <string>
#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "../Header.hpp"

namespace robot_remote_control {
namespace RosConversion {

    inline static void convert(const robot_remote_control::Twist &from, geometry_msgs::msg::TwistStamped *to) {
        convert(from.header(), &to->header);

        to->twist.angular.x = from.angular().x();
        to->twist.angular.y = from.angular().y();
        to->twist.angular.z = from.angular().z();

        to->twist.linear.x = from.linear().x();
        to->twist.linear.y = from.linear().y();
        to->twist.linear.z = from.linear().z();
    }

    inline static void convert(const robot_remote_control::Twist &from, geometry_msgs::msg::Twist *to ) {
        to->angular.x = from.angular().x();
        to->angular.y = from.angular().y();
        to->angular.z = from.angular().z();

        to->linear.x = from.linear().x();
        to->linear.y = from.linear().y();
        to->linear.z = from.linear().z();
    }

    inline static void convert(const geometry_msgs::msg::TwistStamped &from, robot_remote_control::Twist *to) {
        convert(from.header, to->mutable_header());

        to->mutable_angular()->set_x(from.twist.angular.x);
        to->mutable_angular()->set_y(from.twist.angular.y);
        to->mutable_angular()->set_z(from.twist.angular.z);

        to->mutable_linear()->set_x(from.twist.linear.x);
        to->mutable_linear()->set_y(from.twist.linear.y);
        to->mutable_linear()->set_z(from.twist.linear.z);
    }

    inline static void convert(const geometry_msgs::msg::Twist &from, robot_remote_control::Twist *to ) {
        to->mutable_angular()->set_x(from.angular.x);
        to->mutable_angular()->set_y(from.angular.y);
        to->mutable_angular()->set_z(from.angular.z);

        to->mutable_linear()->set_x(from.linear.x);
        to->mutable_linear()->set_y(from.linear.y);
        to->mutable_linear()->set_z(from.linear.z);
    }

}  // namespace RosConversion
}  // namespace robot_remote_control
