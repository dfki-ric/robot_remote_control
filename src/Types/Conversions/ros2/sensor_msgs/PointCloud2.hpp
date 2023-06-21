#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "PointCloud.hpp"
#include "../Header.hpp"

namespace robot_remote_control {
namespace RosConversion {

    static void convert(const sensor_msgs::msg::PointCloud2 &pointcloud, robot_remote_control::PointCloud* to) {
        convert(pointcloud.header, to->mutable_header());

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(pointcloud, "z");
        
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            robot_remote_control::Position* pos = to->add_points();
            pos->set_x(*iter_x);
            pos->set_y(*iter_y);
            pos->set_z(*iter_z);
        }
    }

}  // namespace RosConversion
}  // namespace robot_remote_control
