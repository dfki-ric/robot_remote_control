#pragma once

#include <algorithm>

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <sensor_msgs/Image.h>

#include "../Header.hpp"

namespace robot_remote_control {
namespace RosConversion {

    static void convert(const robot_remote_control::Image &from, sensor_msgs::Image* to) {
        convert(from.header(), &(to->header));
        to->height = from.height();
        to->width = from.width();
        // TODO: translate encodings (throw errors if selected one is not compatible, e.g., MODE_PNG?)
        // compressed images use other message type: sensor_msgs::CompressedImage 
        to->encoding = from.encoding();
        to->is_bigendian = from.is_bigendian();
        to->step = from.step();
        auto const& data = from.data();
        to->data.reserve(data.size());
        std::copy(data.begin(), data.end(), to->data.begin());
    }

    static void convert(const sensor_msgs::Image &from, robot_remote_control::Image* to) {
        convert(from.header, to->mutable_header());
        to->mutable_height() = from.height;
        to->mutable_width() = from.width;
        // TODO: translate encodings (throw errors if selected one is not compatible, e.g., MODE_PNG?)
        // compressed images use other message type: sensor_msgs::CompressedImage 
        to->mutable_encoding() = from.encoding;
        to->mutable_is_bigendian() = from.is_bigendian;
        to->mutable_step() = from.step;
        auto const& data = from.data;
        to->mutable_data().reserve(data.size());
        std::copy(data.begin(), data.end(), to->mutable_data().begin());
    }

}  // namespace RosConversion
}  // namespace robot_remote_control
