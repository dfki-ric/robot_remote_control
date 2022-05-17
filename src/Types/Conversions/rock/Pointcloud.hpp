#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <string>
#include <base/samples/Pointcloud.hpp>
#include "Time.hpp"

namespace robot_remote_control {
namespace RockConversion {

    inline static void convert(base::samples::Pointcloud rock_type, PointCloud *rrc_type, const std::string frame = "world") {
        int counter = 0;

        rrc_type->mutable_points()->Reserve(rock_type.points.size());
        for (auto &point : rock_type.points) {
            robot_remote_control::Position *p = rrc_type->add_points();
            //robot_remote_control::Position* p = rrc_type->mutable_points(counter);
            p->set_x(point[0]);
            p->set_y(point[1]);
            p->set_z(point[2]);
            ++counter;
        }
        // std::vector<base::Vector4d> colors;
        counter = 0;
        if (rock_type.colors.size()) {
            rrc_type->mutable_channels()->Reserve(rock_type.colors.size());
            for (auto &color : rock_type.colors) {
                robot_remote_control::ChannelFloat *channel = rrc_type->add_channels();
                //robot_remote_control::ChannelFloat *channel = rrc_type->mutable_channels(counter);
                channel->set_name("color_rgba");
                channel->add_values(color[0]);
                channel->add_values(color[1]);
                channel->add_values(color[2]);
                channel->add_values(color[3]);
                ++counter;
            }
        }
        rrc_type->mutable_header()->set_frame(frame);
        convert(rock_type.time, rrc_type->mutable_header()->mutable_timestamp());
    }

    inline static void convert(const PointCloud& rrc_type, base::samples::Pointcloud *rock_type) {
        rock_type->points.clear();
        rock_type->colors.clear();

        convert(rrc_type.header().timestamp(), &(rock_type->time));

        rock_type->points.reserve(rrc_type.points().size());
        for (auto &point : rrc_type.points()) {
            base::Point rock_point;
            rock_point[0] = point.x();
            rock_point[1] = point.y();
            rock_point[2] = point.z();
            rock_type->points.push_back(rock_point);
        }

        if (rrc_type.channels_size()) {
            for (auto &channel : rrc_type.channels()) {
                if (channel.name() == "color_rgba") {
                    rock_type->colors.reserve(channel.values().size()/4);
                    int index = 0;
                    base::Vector4d rock_color;
                    for (auto &color : channel.values()) {
                        int colorindex = index % 4;
                        rock_color[colorindex] = color;
                        if (colorindex == 3) {
                            rock_type->colors.push_back(rock_color);
                        }
                    }
                }
            }
        }
    }

}  // namespace RockConversion
}  // namespace robot_remote_control
