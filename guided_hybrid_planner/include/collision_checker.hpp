/*
 * @Author       : dwayne
 * @Date         : 2023-04-15
 * @LastEditTime : 2023-04-16
 * @Description  :
 *
 * Copyright (c) 2023 by dwayne, All Rights Reserved.
 */

#ifndef COLLISION_CHECKER_CPP_
#define COLLISION_CHECKER_CPP_

#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"

namespace guided_hybrid_a_star {
class GridCollisionChecker : public nav2_costmap_2d::FootprintCollisionChecker<
                                 nav2_costmap_2d::Costmap2D*> {
 public:
  GridCollisionChecker(nav2_costmap_2d::Costmap2D* costmap,
                       unsigned int angle_size);

  /**
   * @description  : 设置Footprint
   * @return        {*}
   */
  void setFootprint(const nav2_costmap_2d::Footprint& footprint);

  void setFootprint(const nav2_costmap_2d::Footprint& footprint,
                    const bool& radius, const double& possible_inscribed_cost);

  /**
   * @description  : 判断位于(x,y)处的车是否有碰撞
   * @return        {*}
   */
  bool inCollision(const float& x, const float& y, const float& theta,
                   const bool& traverse_unknown);

  /**
   * @description  : 判断位于index 为 i 的点是否有碰撞
   * @param         {unsigned int&} i:
   * @param         {bool&} traverse_unknown:
   * @return        {*}
   */
  bool inCollision(const unsigned int& i, const bool& traverse_unknown);

  float getCost();

  std::vector<float>& getPrecomputedAngles() { return angles_; }

 private:
  bool outsideRange(const unsigned int& max, const float& value);

 protected:
  std::vector<nav2_costmap_2d::Footprint> oriented_footprints_;
  nav2_costmap_2d::Footprint unoriented_footprint_;
  double footprint_cost_;
  bool footprint_is_radius_;
  double possible_inscribed_cost_{-1};
  std::vector<float> angles_;
};
}  // namespace guided_hybrid_a_star

#endif
