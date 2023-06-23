/*
 * @Author       : dwayne
 * @Date         : 2023-06-21
 * @LastEditTime : 2023-06-21
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef MPC_UTILS__MATH__NORMALIZATION_HPP_
#define MPC_UTILS__MATH__NORMALIZATION_HPP_

#include "mpc_utils/math/constants.hpp"

#include <cmath>

namespace mpc_utils {
inline double normalizeDegree(const double deg, const double min_deg = -180)
{
    const auto max_deg = min_deg + 360.0;

    const auto value = std::fmod(deg, 360.0);
    if (min_deg <= value && value < max_deg) {
        return value;
    }

    return value - std::copysign(360.0, value);
}

inline double normalizeRadian(const double rad, const double min_rad = -pi)
{
    const auto max_rad = min_rad + 2 * pi;

    const auto value = std::fmod(rad, 2 * pi);
    if (min_rad <= value && value < max_rad) {
        return value;
    }

    return value - std::copysign(2 * pi, value);
}

}   // namespace mpc_utils

#endif   // MPC_UTILS__MATH__NORMALIZATION_HPP_
