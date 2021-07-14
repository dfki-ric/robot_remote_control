#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "PointCloud.hpp"

namespace robot_remote_control {
namespace RosConversion {

    static void convert(const sensor_msgs::PointCloud2Ptr &pointcloud, robot_remote_control::PointCloud* to) {
        sensor_msgs::PointCloud pcloud;
        sensor_msgs::convertPointCloud2ToPointCloud(*pointcloud, pcloud);
        convert(pcloud, to);
    }

}  // namespace RockConversion
}  // namespace robot_remote_control
