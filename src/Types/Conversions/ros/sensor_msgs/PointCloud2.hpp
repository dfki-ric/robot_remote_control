#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include "PointCloud.hpp"

namespace robot_remote_control {
namespace RosConversion {

    static void convert(const sensor_msgs::PointCloud2 &pointcloud, robot_remote_control::PointCloud* to) {
        sensor_msgs::PointCloud pcloud;
        sensor_msgs::convertPointCloud2ToPointCloud(pointcloud, pcloud);
        convert(pcloud, to);
    }
    
    static void convert(const robot_remote_control::PointCloud &pointcloud, sensor_msgs::PointCloud2* to) {
        convert(pointcloud.header(), &to->header);


        sensor_msgs::PointCloud2Modifier modifier(*to);
        modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::PointField::FLOAT32,
                                            "y", 1, sensor_msgs::PointField::FLOAT32,
                                            "z", 1, sensor_msgs::PointField::FLOAT32);
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
