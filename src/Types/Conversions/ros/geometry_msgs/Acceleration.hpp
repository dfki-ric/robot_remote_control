#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <geometry_msgs/AccelStamped.h>

namespace robot_remote_control {
namespace RosConversion {
        inline static void convert(const robot_remote_control::Acceleration &from, geometry_msgs::AccelStamped *to ) {
             to->header.stamp = ros::Time::now();
             to->accel.angular.x = from.angular().x();
             to->accel.angular.y = from.angular().y();
             to->accel.angular.z = from.angular().z();

             to->accel.linear.x = from.linear().x();
             to->accel.linear.y = from.linear().y();
             to->accel.linear.z = from.linear().z();
        }

        inline static void convert(const robot_remote_control::Twist &from, geometry_msgs::Accel *to ) {
             to->angular.x = from.angular().x();
             to->angular.y = from.angular().y();
             to->angular.z = from.angular().z();

             to->linear.x = from.linear().x();
             to->linear.y = from.linear().y();
             to->linear.z = from.linear().z();
        }
    }  // namespace RockConversion
}  // namespace robot_remote_control
