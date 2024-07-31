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

    static void convert(const robot_remote_control::PointCloud &pointcloud, sensor_msgs::msg::PointCloud2* to) {
        convert(pointcloud.header(), &to->header);

        sensor_msgs::PointCloud2Modifier modifier(*to);
        modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                            "z", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(pointcloud.points_size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(*to, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*to, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*to, "z");

        for (int i = 0; i < pointcloud.points_size(); ++i, ++iter_x, ++iter_y, ++iter_z) {
            *iter_x = pointcloud.points(i).x();
            *iter_y = pointcloud.points(i).y();
            *iter_z = pointcloud.points(i).z();
        }
    }

}  // namespace RosConversion
}  // namespace robot_remote_control
