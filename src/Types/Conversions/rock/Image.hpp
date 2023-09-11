#pragma once

#include "Time.hpp"
#include <string>
#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <base/samples/Frame.hpp>
#include <boost/bimap.hpp>


namespace robot_remote_control {
namespace RockConversion {

    // https://stackoverflow.com/a/31841462
    template <typename L, typename R>
    boost::bimap<L, R>
    make_bimap(std::initializer_list<typename boost::bimap<L, R>::value_type> list)
    {
        return boost::bimap<L, R>(list.begin(), list.end());
    }

    static const boost::bimap<std::string, base::samples::frame::frame_mode_t> encodings = make_bimap<std::string, base::samples::frame::frame_mode_t>({
            {"MODE_UNDEFINED", base::samples::frame::MODE_UNDEFINED},
            {"MODE_BAYER", base::samples::frame::MODE_BAYER},
            {"MODE_BAYER_RGGB", base::samples::frame::MODE_BAYER_RGGB},
            {"MODE_BAYER_BGGR", base::samples::frame::MODE_BAYER_BGGR},
            {"MODE_BAYER_GBRG", base::samples::frame::MODE_BAYER_GBRG},
            {"MODE_BAYER_GRBG", base::samples::frame::MODE_BAYER_GRBG},
            {"MODE_GRAYSCALE", base::samples::frame::MODE_GRAYSCALE},
            {"MODE_UYVY", base::samples::frame::MODE_UYVY},
            {"MODE_RGB", base::samples::frame::MODE_RGB},
            {"MODE_BGR", base::samples::frame::MODE_BGR},
            {"MODE_RGB32", base::samples::frame::MODE_RGB32},
            {"MODE_PJPG", base::samples::frame::MODE_PJPG},
            {"MODE_JPEG", base::samples::frame::MODE_JPEG},
            {"MODE_PNG", base::samples::frame::MODE_PNG}
    });

    // inline static void convert(const IMU &rrc_type, base::samples::IMUSensors* rock_type) {
    //     convert(rrc_type.acceleration(), &(rock_type->acc));
    //     convert(rrc_type.gyro(), &(rock_type->gyro));
    //     convert(rrc_type.mag(), &(rock_type->mag));
    // }

    inline static void convert(const base::samples::frame::Frame &rock_type, Image *rrc_type) {
        convert(rock_type.time, rrc_type->mutable_header()->mutable_timestamp());
        rrc_type->set_height(rock_type.getHeight());
        rrc_type->set_width(rock_type.getWidth());

        auto const encoding_mode = encodings.right.find(rock_type.getFrameMode());
        std::string encoding = encoding_mode != encodings.right.end() ? encoding_mode->second : "MODE_UNDEFINED";
        rrc_type->set_encoding(encoding);

        rrc_type->set_step(rock_type.getRowSize());
        rrc_type->set_is_bigendian(true);
        rrc_type->set_data(std::string(reinterpret_cast<const char*>(rock_type.image.data()), rock_type.getNumberOfBytes()));
    }

    inline static void convert(Image const &rrc_type, base::samples::frame::Frame *rock_type) {
        convert(rrc_type.header().timestamp(), &rock_type->time);
        auto const encoding_mode = encodings.left.find(rrc_type.encoding());
        base::samples::frame::frame_mode_t encoding = encoding_mode != encodings.left.end()
                ? encoding_mode->second : base::samples::frame::MODE_UNDEFINED;

        rock_type->init(rrc_type.width(), rrc_type.height(), 8, encoding, 0, rrc_type.data().size());

//        rrc_type->set_step(rock_type.getRowSize());
//        rrc_type->set_is_bigendian(true);
        rock_type->setImage(rrc_type.data().data(), rrc_type.data().size());
    }

}  // namespace RockConversion
}  // namespace robot_remote_control
