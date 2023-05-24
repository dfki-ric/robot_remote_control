#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
// deprecated as of ROS2 Foxy
#include <sensor_msgs/msg/point_cloud.hpp>

#include "../Header.hpp"

namespace robot_remote_control {
namespace RosConversion {

    static void convert(const sensor_msgs::msg::PointCloud &from, robot_remote_control::PointCloud* to) {
        convert(from.header, to->mutable_header());

        for (auto &point : from.points) {
            robot_remote_control::Position* pos = to->add_points();
            pos->set_x(point.x);
            pos->set_y(point.y);
            pos->set_z(point.z);
        }
    }

}  // namespace RosConversion
}  // namespace robot_remote_control
