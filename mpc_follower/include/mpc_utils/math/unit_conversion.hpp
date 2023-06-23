/*
 * @Author       : dwayne
 * @Date         : 2023-06-21
 * @LastEditTime : 2023-06-21
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef MPC_UTILS__MATH__UNIT_CONVERSION_HPP_
#define MPC_UTILS__MATH__UNIT_CONVERSION_HPP_

#include "mpc_utils/math/constants.hpp"

namespace mpc_utils {
constexpr double deg2rad(const double deg)
{
    return deg * pi / 180.0;
}
constexpr double rad2deg(const double rad)
{
    return rad * 180.0 / pi;
}

constexpr double kmph2mps(const double kmph)
{
    return kmph * 1000.0 / 3600.0;
}
constexpr double mps2kmph(const double mps)
{
    return mps * 3600.0 / 1000.0;
}
}   // namespace mpc_utils

#endif   // TIER4_AUTOWARE_UTILS__MATH__UNIT_CONVERSION_HPP_
