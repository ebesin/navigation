/*
 * @Author       : dwayne
 * @Date         : 2023-04-15
 * @LastEditTime : 2023-04-15
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */


#include "collision_checker.hpp"

namespace guided_hybrid_a_star {
GridCollisionChecker::GridCollisionChecker(nav2_costmap_2d::Costmap2D* costmap,
                                           unsigned int                angle_size)
    : FootprintCollisionChecker(costmap)
{
    // Convert number of regular bins into angles
    float bin_size = 2 * M_PI / static_cast<float>(angle_size);
    angles_.reserve(angle_size);
    for (unsigned int i = 0; i != angle_size; i++) {
        angles_.push_back(bin_size * i);
    }
}

void GridCollisionChecker::setFootprint(const nav2_costmap_2d::Footprint& footprint)
{
    // No change, no updates required
    if (footprint == unoriented_footprint_) {
        return;
    }

    oriented_footprints_.reserve(angles_.size());
    double                    sin_th, cos_th;
    geometry_msgs::msg::Point new_pt;
    const unsigned int        footprint_size = footprint.size();

    // Precompute the orientation bins for checking to use
    for (unsigned int i = 0; i != angles_.size(); i++) {
        sin_th = sin(angles_[i]);
        cos_th = cos(angles_[i]);
        nav2_costmap_2d::Footprint oriented_footprint;
        oriented_footprint.reserve(footprint_size);

        for (unsigned int j = 0; j < footprint_size; j++) {
            new_pt.x = footprint[j].x * cos_th - footprint[j].y * sin_th;
            new_pt.y = footprint[j].x * sin_th + footprint[j].y * cos_th;
            oriented_footprint.push_back(new_pt);
        }

        oriented_footprints_.push_back(oriented_footprint);
    }

    unoriented_footprint_ = footprint;
}

bool GridCollisionChecker::inCollision(const float& x,
                                       const float& y,
                                       const float& angle_bin,
                                       const bool&  traverse_unknown)
{
    // Check to make sure cell is inside the map
    if (outsideRange(costmap_->getSizeInCellsX(), x) ||
        outsideRange(costmap_->getSizeInCellsY(), y)) {
        return false;
    }

    // Assumes setFootprint already set
    double wx, wy;

    footprint_cost_ = costmap_->getCost(static_cast<unsigned int>(x), static_cast<unsigned int>(y));

    if (footprint_cost_ == nav2_costmap_2d::NO_INFORMATION && traverse_unknown) {
        return false;
    }

    // if occupied or unknown and not to traverse unknown space
    return static_cast<double>(footprint_cost_) > nav2_costmap_2d::MAX_NON_OBSTACLE;
}

bool GridCollisionChecker::inCollision(const unsigned int& i, const bool& traverse_unknown)
{
    footprint_cost_ = costmap_->getCost(i);
    if (footprint_cost_ == nav2_costmap_2d::NO_INFORMATION && traverse_unknown) {
        return false;
    }

    // if occupied or unknown and not to traverse unknown space
    return footprint_cost_ >= nav2_costmap_2d::MAX_NON_OBSTACLE;
}

float GridCollisionChecker::getCost()
{
    // Assumes inCollision called prior
    return static_cast<float>(footprint_cost_);
}

bool GridCollisionChecker::outsideRange(const unsigned int& max, const float& value)
{
    return value < 0.0f || value > max;
}

}   // namespace guided_hybrid_a_star
