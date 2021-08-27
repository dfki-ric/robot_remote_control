#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <base/Time.hpp>

namespace robot_remote_control {
namespace RockConversion {

    inline static void convert(base::Time rock_type, TimeStamp *rrc_type) {
        timeval time = rock_type.toTimeval();
        rrc_type->set_secs(time.tv_sec);
        rrc_type->set_nsecs(time.tv_usec * 1000);
    }

}  // namespace RockConversion
}  // namespace robot_remote_control
