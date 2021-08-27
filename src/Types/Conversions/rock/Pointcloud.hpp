#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <base/samples/Pointcloud.hpp>

namespace robot_remote_control {
namespace RockConversion {

    inline static void convert(base::samples::Pointcloud rock_type, PointCloud *rrc_type, const std::string frame = "world") {
        for (auto &point : rock_type.points) {
            robot_remote_control::Position* p = rrc_type->add_points();
            p->set_x(point[0]);
            p->set_y(point[1]);
            p->set_z(point[2]);
        }
        // std::vector<base::Vector4d> colors;
        if (rock_type.colors.size()) {
            for (auto &color : rock_type.colors) {
                robot_remote_control::ChannelFloat *channel = rrc_type->add_channels();
                channel->set_name("color_rgba");
                channel->add_values(color[0]);
                channel->add_values(color[1]);
                channel->add_values(color[2]);
                channel->add_values(color[3]);
            }
        }
        rrc_type->set_frame(frame);
    }

}  // namespace RockConversion
}  // namespace robot_remote_control
