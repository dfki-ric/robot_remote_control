#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <string>
#include <base/samples/Pointcloud.hpp>
#include "Time.hpp"
#include <cinttypes>


namespace robot_remote_control {
namespace RockConversion {

    // helper class to pack/unpack rgb to/from float
    // binary format explained here, too:
    // https://pointclouds.org/documentation/structpcl_1_1_point_x_y_z_r_g_b.html#details
    // http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/ChannelFloat32.html
    struct rgb_converter {
        uint8_t b, g, r, a;
        rgb_converter(uint8_t r, uint8_t g, uint8_t b) : b(b), g(g), r(r), a(0.0) {}
        rgb_converter(float const x) {
            static_assert(sizeof(rgb_converter)==sizeof x, "type sizes don't match");
            std::memcpy(this, &x, sizeof x);
        }
        float to_float() const {
            float x;
            std::memcpy(&x, this, sizeof x);
            return x;
        }
    };

    inline static void convert(base::samples::Pointcloud rock_type, PointCloud *rrc_type, const std::string frame = "") {
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
            // TODO: is this really how this should work?
            //       one channel for each point in the data set?
            //       also space is reserved only for 1/4 of the expected data??
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
                    // TODO: this should be adjusted
                    //       don't transmit color pixels with four float values each
                    //       also, please don't have one channel per point/pixel
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
                else if (channel.name() == "rgb") {
                    // rock pointclouds color values are stored as double in range [0, 1.0]
                    // scale to convert uint8 to double
                    const double rgb_scale = 1.0/255.0;
                    rock_type->colors.reserve(channel.values().size());
                    for (auto const& color : channel.values()) {
                        // convert float to rgb tuple
                        // rescale from uint8 to double
                        rgb_converter rgb_convert{color};
                        rock_type->colors.emplace_back(
                                rgb_convert.r*rgb_scale,
                                rgb_convert.g*rgb_scale,
                                rgb_convert.b*rgb_scale,
                                1.0);
                    }
                }
            }
        }
    }

}  // namespace RockConversion
}  // namespace robot_remote_control
