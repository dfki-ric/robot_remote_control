#pragma once

#include <robot_remote_control/Types/RobotRemoteControl.pb.h>
#include <maps/grid/MLSMap.hpp>
#include <string>
#include <limits>

#include "Eigen.hpp"
#include "Time.hpp"

namespace robot_remote_control {
namespace RockConversion {

    inline static void convert(const maps::grid::Vector2ui &rock_type, Vector2 *rrc_type) {
        rrc_type->set_x(rock_type.x());
        rrc_type->set_y(rock_type.y());
    }

    inline static void convert(const Vector2 &rrc_type, maps::grid::Vector2ui *rock_type) {
        rock_type->x() = rrc_type.x();
        rock_type->y() = rrc_type.y();
    }

    /**
     * @brief this converter converts into a simplification of the MLS, only consisting the lowest level
     * Thus converting the MLS into a simple heightmap
     * 
     * @param rock_type 
     * @param rrc_type 
     */
    inline static void convert(const maps::grid::MLSMapKalman &rock_type, SimpleSensor *rrc_type, const std::string &name = "") {
        rrc_type->set_name(name);

        maps::grid::Vector2ui num_cell = rock_type.getNumCells();
        convert(num_cell, rrc_type->mutable_size());
        convert((base::Vector2d)rock_type.getResolution(), rrc_type->mutable_scale());
        convert(rock_type.translation(), rrc_type->mutable_local_offset());
        convert(rock_type.rotation(), rrc_type->mutable_local_orientation());


        // reserve space for all value entries
        rrc_type->mutable_value()->Reserve(num_cell.x() * num_cell.y());
        for (size_t y = 0; y < num_cell.y(); y++) {
            for (size_t x = 0; x < num_cell.x(); x++) {
                const maps::grid::MLSMapKalman::CellType &list = rock_type.at(x, y);
                if (list.size()) {
                    // only the the fitst SurfacePatch(Base) (Iterator) from the list
                    auto patch = list.begin();
                    // set the only the lowest data point, but it's maximum value
                    // when a block is the lowest entry, we want to use the top of it
                    rrc_type->mutable_value()->Set(x+y*num_cell.x(), patch->getMax());
                } else {
                    rrc_type->mutable_value()->Set(x+y*num_cell.x(), std::numeric_limits<float>::quiet_NaN());
                    //rrc_type->add_value(std::numeric_limits<float>::quiet_NaN());
                }
            }
        }
    }

    inline static void convert(const SimpleSensor &rrc_type, maps::grid::MLSMapKalman *rock_type) {
        maps::grid::Vector2ui num_cell;
        convert(rrc_type.size(), &num_cell);
        rock_type->resize(num_cell);

        base::Vector2d res;
        convert(rrc_type.scale(), &res);
        rock_type->setResolution(res);

        rrc_type->mutable_value()->Reserve(num_cell.x() * num_cell.y());
        for (size_t y = 0; y < num_cell.y(); y++) {
            for (size_t x = 0; x < num_cell.x(); x++) {
                maps::grid::SurfacePatch<maps::grid::MLSConfig::KALMAN> patch(rrc_type.value(x+y*num_cell.x()), 0);  // set 0 variance
                rock_type->at(x, y).insert(patch);
            }
        }

        base::Vector3d translation;
        convert(rrc_type.local_offset(), &translation);
        rock_type->translate(translation);
    }

    /**
     * @brief this converter converts into a simplification of the MLS, only consisting the lowest level
     * Thus converting the MLS into a simple heightmap
     * 
     * @param rock_type 
     * @param rrc_type 
     */
    inline static void convert(const maps::grid::MLSMapSloped &rock_type, SimpleSensor *rrc_type, const std::string &name = "") {
        rrc_type->set_name(name);

        maps::grid::Vector2ui num_cell = rock_type.getNumCells();
        convert(num_cell, rrc_type->mutable_size());
        convert((base::Vector2d)rock_type.getResolution(), rrc_type->mutable_scale());
        convert(rock_type.translation(), rrc_type->mutable_local_offset());
        convert(rock_type.rotation(), rrc_type->mutable_local_orientation());

        // reserve space for all value entries
        rrc_type->mutable_value()->Reserve(num_cell.x() * num_cell.y());
        for (size_t y = 0; y < num_cell.y(); y++) {
            for (size_t x = 0; x < num_cell.x(); x++) {
                const auto &list = rock_type.at(x, y);
                if (list.size()) {
                    // only the the fitst SurfacePatch(Base) (Iterator) from the list
                    auto patch = list.begin();
                    // set the only the lowest data point, but it's maximum value
                    // when a block is the lowest entry, we want to use the top of it
                    //rrc_type->mutable_value()->Set(x+y*num_cell.x(), patch->getMax());
                    rrc_type->add_value(patch->getMax());
                } else {
                    //rrc_type->mutable_value()->Set(x+y*num_cell.x(), std::numeric_limits<float>::quiet_NaN());
                    rrc_type->add_value(std::numeric_limits<float>::quiet_NaN());
                }
            }
        }
    }

    // inline static void convert(const SimpleSensor &rrc_type, maps::grid::MLSMapSloped *rock_type) {
    //     maps::grid::Vector2ui num_cell;
    //     convert(rrc_type.size(), &num_cell);
    //     rock_type->resize(num_cell);

    //     base::Vector2d res;
    //     convert(rrc_type.scale(), &res);
    //     rock_type->setResolution(res);

    //     for (size_t y = 0; y < num_cell.y(); y++) {
    //         for (size_t x = 0; x < num_cell.x(); x++) {
    //             float value = rrc_type.value(x+y*num_cell.x());
    //             if (value != std::numeric_limits<float>::quiet_NaN()) {
    //                 maps::grid::SurfacePatch<maps::grid::MLSConfig::SLOPE> patch(Eigen::Vector3f(1, 1, value), 0);  // set 0 variance
    //                 rock_type->mergePatch(maps::grid::Index(x, y), patch);
    //             }
    //         }
    //     }
    // }

    /**
     * @brief this converter converts into a simplification of the MLS, only consisting the lowest level
     * Thus converting the MLS into a simple heightmap
     * 
     * @param rock_type 
     * @param rrc_type 
     */
    inline static void convert(const maps::grid::MLSMapPrecalculated &rock_type, SimpleSensor *rrc_type, const std::string &name = "") {
        rrc_type->set_name(name);

        maps::grid::Vector2ui num_cell = rock_type.getNumCells();
        convert(num_cell, rrc_type->mutable_size());
        convert((base::Vector2d)rock_type.getResolution(), rrc_type->mutable_scale());
        convert(rock_type.translation(), rrc_type->mutable_local_offset());
        convert(rock_type.rotation(), rrc_type->mutable_local_orientation());

        // reserve space for all value entries
        rrc_type->mutable_value()->Reserve(num_cell.x() * num_cell.y());
        for (size_t y = 0; y < num_cell.y(); y++) {
            for (size_t x = 0; x < num_cell.x(); x++) {
                const auto &list = rock_type.at(x, y);
                if (list.size()) {
                    // only the the fitst SurfacePatch(Base) (Iterator) from the list
                    auto patch = list.begin();
                    // set the only the lowest data point, but it's maximum value
                    // when a block is the lowest entry, we want to use the top of it
                    //rrc_type->mutable_value()->Set(x+y*num_cell.x(), patch->getMax());
                    rrc_type->add_value(patch->getMax());
                } else {
                    //rrc_type->mutable_value()->Set(x+y*num_cell.x(), std::numeric_limits<float>::quiet_NaN());
                    rrc_type->add_value(std::numeric_limits<float>::quiet_NaN());
                }
            }
        }
    }

    // inline static void convert(const SimpleSensor &rrc_type, maps::grid::MLSMapPrecalculated *rock_type) {
    //     maps::grid::Vector2ui num_cell;
    //     convert(rrc_type.size(), &num_cell);
    //     rock_type->resize(num_cell);

    //     base::Vector2d res;
    //     convert(rrc_type.scale(), &res);
    //     rock_type->setResolution(res);

    //     for (size_t y = 0; y < num_cell.y(); y++) {
    //         for (size_t x = 0; x < num_cell.x(); x++) {
    //             float value = rrc_type.value(x+y*num_cell.x());
    //             if (value != std::numeric_limits<float>::quiet_NaN()) {
    //                 maps::grid::SurfacePatch<maps::grid::MLSConfig::PRECALCULATED> patch(Eigen::Vector3f(1, 1, value), Eigen::Vector3f(0, 0, 1), value, value);  // set 0 variance
    //                 rock_type->mergePatch(maps::grid::Index(x, y), patch);
    //                 // rock_type->at().insert(patch, config);
    //             }
    //         }
    //     }
    // }

}  // namespace RockConversion
}  // namespace robot_remote_control
