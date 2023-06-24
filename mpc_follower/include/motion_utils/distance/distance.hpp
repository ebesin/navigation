/*
 * @Author       : dwayne
 * @Date         : 2023-06-23
 * @LastEditTime : 2023-06-23
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef MOTION_UTILS__DISTANCE__DISTANCE_HPP_
#define MOTION_UTILS__DISTANCE__DISTANCE_HPP_

#include <boost/optional.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <tuple>
#include <vector>

namespace motion_utils {
boost::optional<double> calcDecelDistWithJerkAndAccConstraints(const double current_vel,
                                                               const double target_vel,
                                                               const double current_acc,
                                                               const double acc_min,
                                                               const double jerk_acc,
                                                               const double jerk_dec);

}   // namespace motion_utils

#endif   // MOTION_UTILS__DISTANCE__DISTANCE_HPP_
