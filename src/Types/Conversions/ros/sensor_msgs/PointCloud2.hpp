#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include "PointCloud.hpp"

namespace robot_remote_control {
namespace RosConversion {

    static void convert(const sensor_msgs::PointCloud2 &from, robot_remote_control::PointCloud* to) {
        convert(from.header, to->mutable_header());
        // assume pointcloud either has
        // * no additional channels
        // * single float rgb channel
        // * single float intensity channel
        // find out about the situation
        std::string extra_channel = "";
        for (auto const& field : from.fields) {
            // probably no cloud will have both
            if (field.name == "rgb" || field.name == "intensity") {
                extra_channel = field.name;
                // got one, can't handle others
                // TODO: check type (assume float32 for now)
                break;
            }
        }
        // resize and assign points
        auto n_points = from.height * from.width;
        // resize and assign additional channel (if available)
        // if not available, perform no-op instead of adding anything
        std::function<void(float)> add_channel_point = [](float){};
        if (extra_channel == "rgb") {
            // only act on rgb channel for now
            // code should work with intensity channel just as good
            auto* channel = to->add_channels();
            channel->set_name(extra_channel);
            channel->mutable_values()->Reserve(n_points);
            add_channel_point =
                [channel](float p) {
                    // append value to the first channel (which would be this)
                    // assume that we will only ever deal with a single extra channel
                    channel->add_values(p);
                };
        }
        else {
            // re-assign extra channel to end up with a valid iterator
            // iterator will not be read by add_channel_point function in this case
            extra_channel = "x";
        }
        // reserve enough space beforehand to avoid reallocation
        to->mutable_points()->Reserve(n_points);
        for (
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(from, "x"), iter_y(from, "y"), iter_z(from, "z"), iter_extra(from, extra_channel);
            iter_x != iter_x.end();
            ++iter_x, ++iter_y, ++iter_z, ++iter_extra
            )
        {
            auto* p = to->add_points();
            p->set_x(*iter_x);
            p->set_y(*iter_y);
            p->set_z(*iter_z);
            add_channel_point(*iter_extra);
        }
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
