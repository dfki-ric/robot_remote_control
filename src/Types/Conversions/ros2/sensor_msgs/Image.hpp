#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <sensor_msgs/msg/image.hpp>
#include <boost/bimap.hpp>
#include <string>

#include "../Header.hpp"

namespace robot_remote_control {
namespace RosConversion {

    // https://stackoverflow.com/a/31841462
    template <typename L, typename R>
    boost::bimap<L, R>
    make_bimap(std::initializer_list<typename boost::bimap<L, R>::value_type> list)
    {
        return boost::bimap<L, R>(list.begin(), list.end());
    }
    static const boost::bimap<std::string, std::string> encodings = make_bimap<std::string, std::string>({
        {"MODE_BAYER_RGGB", "bayer_rggb8"},
        {"MODE_BAYER_BGGR", "bayer_bggr8"},
        {"MODE_BAYER_GBRG", "bayer_gbrg8"},
        {"MODE_BAYER_GRBG", "bayer_grbg8"},
        {"MODE_GRAYSCALE", "mono8"},
        {"MODE_RGB", "rgb8"},
        {"MODE_BGR", "bgr8"}
    });

    static void convert(const robot_remote_control::Image &from, sensor_msgs::msg::Image* to) {
        convert(from.header(), &(to->header));
        to->height = from.height();
        to->width = from.width();
        if (encodings.left.find(from.encoding()) != encodings.left.end()) {
            // encoding supported
            to->encoding = encodings.left.at(from.encoding());
        } else {
            // encoding unsupported, leave original value as hint, as to how to interpret the data
            to->encoding = from.encoding();
        }
        to->is_bigendian = from.is_bigendian();
        to->step = from.step();
        auto const& data = from.data();
        to->data.resize(data.size());
        std::copy(data.begin(), data.end(), to->data.begin());
    }

    static void convert(const sensor_msgs::msg::Image &from, robot_remote_control::Image* to) {
        convert(from.header, to->mutable_header());
        to->set_height(from.height);
        to->set_width(from.width);
        if (encodings.right.find(from.encoding) != encodings.right.end()) {
            // encoding supported
            to->set_encoding(encodings.right.at(from.encoding));
        } else {
            // encoding unsupported, leave original value as hint, as to how to interpret the data
            to->set_encoding(from.encoding);
        }
        to->set_is_bigendian(from.is_bigendian);
        to->set_step(from.step);
        to->mutable_data()->resize(from.data.size());
        std::copy(from.data.begin(), from.data.end(), to->mutable_data()->begin());
    }

}  // namespace RosConversion
}  // namespace robot_remote_control
