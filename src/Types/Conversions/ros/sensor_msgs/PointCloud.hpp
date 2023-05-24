#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <sensor_msgs/PointCloud.h>

namespace robot_remote_control {
namespace RosConversion {

    static void convert(const sensor_msgs::PointCloud &from, robot_remote_control::PointCloud* to) {
        for (auto &point : from.points) {
            robot_remote_control::Position* pos = to->add_points();
            pos->set_x(point.x);
            pos->set_y(point.y);
            pos->set_z(point.z);
        }
    }

}  // namespace RosConversion
}  // namespace robot_remote_control
