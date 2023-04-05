#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <base/samples/Wrenches.hpp>
#include "Time.hpp"
#include "Eigen.hpp"

namespace robot_remote_control {
namespace RockConversion {


   inline static void convert(const WrenchState &rrc_type, base::samples::Wrenches* rock_type) {
        rock_type->clear();
        convert(rrc_type.header(), &(rock_type->time));

        rock_type->names.reserve(rrc_type.wrenches().size());
        rock_type->elements.reserve(rrc_type.wrenches().size());
        for (int i = 0; i < rrc_type.wrenches().size(); i++) {
            try {
                base::Wrench wrench;
                convert(rrc_type.wrenches(i).force(), &wrench.force);
                convert(rrc_type.wrenches(i).torque(), &wrench.torque);
                rock_type->names.push_back(rrc_type.wrenches(i).header().frame());
                rock_type->elements.push_back(wrench);
            } catch (base::NamedVector<base::Wrench>::InvalidName &e) {
                std::cout << "Name not found" << std::endl;
            }
        }
    }

    inline static void convert(const base::samples::Wrenches &rock_type, WrenchState *rrc_type) {
        // Clear last state
        rrc_type->clear_wrenches();
        convert(rock_type.time, rrc_type->mutable_header());

        rrc_type->mutable_wrenches()->Reserve(rock_type.size());
        for (unsigned int i = 0; i < rock_type.size(); i++) {
            Wrench* rrc_wrench = rrc_type->add_wrenches();
            rrc_wrench->mutable_header()->set_frame(rock_type.names[i]);

            convert(rock_type.time, rrc_wrench->mutable_header());
            convert(rock_type.elements[i].force, rrc_wrench->mutable_force());
            convert(rock_type.elements[i].torque, rrc_wrench->mutable_torque());
        }
    }

}  // namespace RockConversion
}  // namespace robot_remote_control
