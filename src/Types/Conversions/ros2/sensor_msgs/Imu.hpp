#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <sensor_msgs/msg/imu.hpp>

#include "../geometry_msgs/Quaternion.hpp"
#include "../geometry_msgs/Vector3.hpp"
#include "../Header.hpp"

namespace robot_remote_control {
namespace RosConversion {

    static void convert(const robot_remote_control::IMU &from, sensor_msgs::msg::Imu* to) {
        convert(from.header(), &to->header);
        convert(from.orientation(), &to->orientation);
        convert(from.gyro(), &to->angular_velocity);
        convert(from.acceleration(), &to->linear_acceleration);
    }

    static void convert(const sensor_msgs::msg::Imu &from, robot_remote_control::IMU* to) {
        convert(from.header, to->mutable_header());
        convert(from.orientation, to->mutable_orientation());
        convert(from.angular_velocity, to->mutable_gyro());
        convert(from.linear_acceleration, to->mutable_acceleration());
    }

}  // namespace RosConversion
}  // namespace robot_remote_control
