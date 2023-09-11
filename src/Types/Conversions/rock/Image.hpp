#pragma once

#include <string>
#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <base/samples/Frame.hpp>
#include "Time.hpp"

namespace robot_remote_control {
namespace RockConversion {

    // inline static void convert(const IMU &rrc_type, base::samples::IMUSensors* rock_type) {
    //     convert(rrc_type.acceleration(), &(rock_type->acc));
    //     convert(rrc_type.gyro(), &(rock_type->gyro));
    //     convert(rrc_type.mag(), &(rock_type->mag));
    // }

    inline static void convert(const base::samples::frame::Frame &rock_type, Image *rrc_type) {
        convert(rock_type.time, rrc_type->mutable_header()->mutable_timestamp());
        rrc_type->set_height(rock_type.getHeight());
        rrc_type->set_width(rock_type.getWidth());

        std::string encoding;
        switch (rock_type.getFrameMode()) {
            case base::samples::frame::MODE_UNDEFINED: encoding = "MODE_UNDEFINED"; break;
            case base::samples::frame::MODE_BAYER: encoding = "MODE_BAYER"; break;
            case base::samples::frame::MODE_BAYER_RGGB: encoding = "MODE_BAYER_RGGB"; break;
            case base::samples::frame::MODE_BAYER_BGGR: encoding = "MODE_BAYER_BGGR"; break;
            case base::samples::frame::MODE_BAYER_GBRG: encoding = "MODE_BAYER_GBRG"; break;
            case base::samples::frame::MODE_BAYER_GRBG: encoding = "MODE_BAYER_GRBG"; break;
            case base::samples::frame::MODE_GRAYSCALE: encoding = "MODE_GRAYSCALE"; break;
            case base::samples::frame::MODE_UYVY: encoding = "MODE_UYVY"; break;
            case base::samples::frame::MODE_RGB: encoding = "MODE_RGB"; break;
            case base::samples::frame::MODE_BGR: encoding = "MODE_BGR"; break;
            case base::samples::frame::MODE_RGB32: encoding = "MODE_RGB32"; break;
            case base::samples::frame::MODE_PJPG: encoding = "MODE_PJPG"; break;
            case base::samples::frame::MODE_JPEG: encoding = "MODE_JPEG"; break;
            case base::samples::frame::MODE_PNG: encoding = "MODE_PNG"; break;
        }
        rrc_type->set_encoding(encoding);

        rrc_type->set_step(rock_type.getRowSize());
        rrc_type->set_is_bigendian(true);
        rrc_type->set_data(std::string(reinterpret_cast<const char*>(rock_type.image.data()), rock_type.getNumberOfBytes()));
    }

    inline static void convert(Image const &rrc_type, base::samples::frame::Frame *rock_type) {
        // non-functional implementation for CI until https://github.com/dfki-ric/robot_remote_control/pull/37 is merged
    }
        

}  // namespace RockConversion
}  // namespace robot_remote_control
