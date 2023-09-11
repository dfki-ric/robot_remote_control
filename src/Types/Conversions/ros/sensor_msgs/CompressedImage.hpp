#pragma once

#include <algorithm>

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <sensor_msgs/CompressedImage.h>

#include "../Header.hpp"

namespace robot_remote_control {
namespace RosConversion {

    static void convert(const robot_remote_control::Image &from, sensor_msgs::CompressedImage* to) {
        convert(from.header(), &(to->header));
        // TODO: translate encodings (throw errors if selected one is not compatible, e.g., MODE_PNG?)
        // compressed images use other message type: sensor_msgs::CompressedImage 
        auto const& encoding = from.encoding();
        if (encoding == "MODE_JPEG") {
            to->format = "jpg";
        }
        else if (encoding == "MODE_PNG") {
            to->format = "png";
        }
        else {
            // encoding unsupported, leave original value as hint, as to how to interpret the data
            to->format = encoding;
        }
        auto const& data = from.data();
        to->data.resize(data.size());
        std::copy(data.begin(), data.end(), to->data.begin());
    }

    static void convert(const sensor_msgs::CompressedImage &from, robot_remote_control::Image* to) {
        convert(from.header, to->mutable_header());
        // TODO: translate encodings (throw errors if selected one is not compatible, e.g., MODE_PNG?)
        // compressed images use other message type: sensor_msgs::CompressedImage 
        auto const& format = from.format;
        if (format.find("jpeg") != std::string::npos) {
            to->set_encoding("MODE_JPEG");
        }
        else if (format.find("png") != std::string::npos) {
            to->set_encoding("MODE_PNG");
        }
        else {
            // encoding unsupported, leave original value as hint, as to how to interpret the data
            to->set_encoding(format);
        }
        auto const& data = from.data;
        to->mutable_data()->resize(data.size());
        std::copy(data.begin(), data.end(), to->mutable_data()->begin());
    }

}  // namespace RosConversion
}  // namespace robot_remote_control
