#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <base/samples/Joints.hpp>
#include "Time.hpp"


namespace robot_remote_control {
namespace RockConversion {

    inline static void convert(const JointState &rrc_type, base::samples::Joints* rock_type) {
        rock_type->clear();
        convert(rrc_type.header(), &(rock_type->time));
        // init all states
        for (int i = 0; i < rrc_type.name().size(); i++) {
            try {
                rock_type->names.push_back(rrc_type.name(i));

                base::JointState joint;
                if (rrc_type.position().size()) {
                    joint.position = rrc_type.position(i);
                }
                if (rrc_type.velocity().size()) {
                    joint.speed = rrc_type.velocity(i);
                }
                if (rrc_type.effort().size()) {
                    joint.effort = rrc_type.effort(i);
                }
                if (rrc_type.acceleration().size()) {
                    joint.acceleration = rrc_type.acceleration(i);
                }
                rock_type->elements.push_back(joint);
            } catch (base::NamedVector<base::JointState>::InvalidName &e) {
                std::cout << "Name not found" << std::endl;
            }
        }
    }

    inline static void convert(const JointCommand &rrc_type, base::samples::Joints* rock_type) {
        rock_type->clear();
        convert(rrc_type.header(), &(rock_type->time));
        // init all states
        for (int i = 0; i < rrc_type.name().size(); i++) {
            try {
                rock_type->names.push_back(rrc_type.name(i));

                base::JointState joint;
                if (rrc_type.position().size()) {
                    joint.position = rrc_type.position(i);
                }
                if (rrc_type.velocity().size()) {
                    joint.speed = rrc_type.velocity(i);
                }
                if (rrc_type.effort().size()) {
                    joint.effort = rrc_type.effort(i);
                }
                if (rrc_type.acceleration().size()) {
                    joint.acceleration = rrc_type.acceleration(i);
                }
                rock_type->elements.push_back(joint);
            } catch (base::NamedVector<base::JointState>::InvalidName &e) {
                std::cout << "Name not found" << std::endl;
            }
        }
    }

    inline static void convert(const base::samples::Joints &rock_type, JointState *rrc_type) {
        convert(rock_type.time, rrc_type->mutable_header());
        // Clear last state
        rrc_type->clear_name();
        rrc_type->clear_position();
        rrc_type->clear_velocity();
        rrc_type->clear_effort();

        for (unsigned int i = 0; i < rock_type.size(); i++) {
            rrc_type->add_name(rock_type.names[i]);
            rrc_type->add_position(rock_type.elements[i].position);
            rrc_type->add_velocity(rock_type.elements[i].speed);
            rrc_type->add_effort(rock_type.elements[i].effort);
        }
    }

}  // namespace RockConversion
}  // namespace robot_remote_control
