#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <base/Time.hpp>

namespace robot_remote_control {
namespace RockConversion {

    inline static void convert(const base::Time &rock_type, TimeStamp *rrc_type) {
        timeval time = rock_type.toTimeval();
        rrc_type->set_secs(time.tv_sec);
        rrc_type->set_nsecs(time.tv_usec * 1000);
    }

    inline static void convert(const TimeStamp &rrc_type, base::Time *rock_type) {
        *rock_type = base::Time::fromSeconds(rrc_type.secs(), rrc_type.nsecs() / 1000);
    }


}  // namespace RockConversion
}  // namespace robot_remote_control
