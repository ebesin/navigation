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
GridCollisionChecker::GridCollisionChecker(nav2_costmap_2d::Costmap2D *costmap,
                                           unsigned int angle_size)
    : FootprintCollisionChecker(costmap) {
  // Convert number of regular bins into angles
  float bin_size = 2 * M_PI / static_cast<float>(angle_size);
  angles_.reserve(angle_size);
  for (unsigned int i = 0; i != angle_size; i++) {
    angles_.push_back(bin_size * i);
  }
}

void GridCollisionChecker::setFootprint(
    const nav2_costmap_2d::Footprint &footprint, const bool &radius,
    const double &possible_inscribed_cost) {
  possible_inscribed_cost_ = possible_inscribed_cost;
  footprint_is_radius_ = radius;

  // Use radius, no caching required
  if (radius) {
    return;
  }

  // No change, no updates required
  if (footprint == unoriented_footprint_) {
    return;
  }

  oriented_footprints_.reserve(angles_.size());
  double sin_th, cos_th;
  geometry_msgs::msg::Point new_pt;
  const unsigned int footprint_size = footprint.size();

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

void GridCollisionChecker::setFootprint(
    const nav2_costmap_2d::Footprint &footprint) {
  // No change, no updates required
  if (footprint == unoriented_footprint_) {
    return;
  }

  oriented_footprints_.reserve(angles_.size());
  double sin_th, cos_th;
  geometry_msgs::msg::Point new_pt;
  const unsigned int footprint_size = footprint.size();

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

bool GridCollisionChecker::inCollision(const float &x, const float &y,
                                       const float &angle_bin,
                                       const bool &traverse_unknown) {
  // Check to make sure cell is inside the map
  if (outsideRange(costmap_->getSizeInCellsX(), x) ||
      outsideRange(costmap_->getSizeInCellsY(), y)) {
    return false;
  }

  // Assumes setFootprint already set
  double wx, wy;
  costmap_->mapToWorld(static_cast<double>(x), static_cast<double>(y), wx, wy);
  if (!footprint_is_radius_) {
    // if footprint, then we check for the footprint's points, but first see
    // if the robot is even potentially in an inscribed collision
    footprint_cost_ = costmap_->getCost(static_cast<unsigned int>(x),
                                        static_cast<unsigned int>(y));

    if (footprint_cost_ < possible_inscribed_cost_) {
      return false;
    }

    // If its inscribed, in collision, or unknown in the middle,
    // no need to even check the footprint, its invalid
    if (footprint_cost_ == nav2_costmap_2d::NO_INFORMATION &&
        !traverse_unknown) {
      return true;
    }

    if (static_cast<double>(footprint_cost_) >
        nav2_costmap_2d::MAX_NON_OBSTACLE) {
      return true;
    }

    // if possible inscribed, need to check actual footprint pose.
    // Use precomputed oriented footprints are done on initialization,
    // offset by translation value to collision check
    geometry_msgs::msg::Point new_pt;
    const nav2_costmap_2d::Footprint &oriented_footprint =
        oriented_footprints_[angle_bin];
    nav2_costmap_2d::Footprint current_footprint;
    current_footprint.reserve(oriented_footprint.size());
    for (unsigned int i = 0; i < oriented_footprint.size(); ++i) {
      new_pt.x = wx + oriented_footprint[i].x;
      new_pt.y = wy + oriented_footprint[i].y;
      current_footprint.push_back(new_pt);
    }

    footprint_cost_ = footprintCost(current_footprint);

    if (footprint_cost_ == nav2_costmap_2d::NO_INFORMATION &&
        traverse_unknown) {
      return false;
    }

    // if occupied or unknown and not to traverse unknown space
    return footprint_cost_ >= nav2_costmap_2d::MAX_NON_OBSTACLE;
  } else {
    footprint_cost_ = costmap_->getCost(static_cast<unsigned int>(x),
                                        static_cast<unsigned int>(y));

    if (footprint_cost_ == nav2_costmap_2d::NO_INFORMATION &&
        traverse_unknown) {
      return false;
    }

    // if occupied or unknown and not to traverse unknown space
    return static_cast<double>(footprint_cost_) >
           nav2_costmap_2d::MAX_NON_OBSTACLE;
  }
}

bool GridCollisionChecker::inCollision(const unsigned int &i,
                                       const bool &traverse_unknown) {
  footprint_cost_ = costmap_->getCost(i);
  if (footprint_cost_ == nav2_costmap_2d::NO_INFORMATION && traverse_unknown) {
    return false;
  }

  // if occupied or unknown and not to traverse unknown space
  return footprint_cost_ >= nav2_costmap_2d::MAX_NON_OBSTACLE;
}

float GridCollisionChecker::getCost() {
  // Assumes inCollision called prior
  return static_cast<float>(footprint_cost_);
}

bool GridCollisionChecker::outsideRange(const unsigned int &max,
                                        const float &value) {
  return value < 0.0f || value > max;
}

}  // namespace guided_hybrid_a_star
