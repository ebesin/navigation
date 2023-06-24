/*
 * @Author       : dwayne
 * @Date         : 2023-06-23
 * @LastEditTime : 2023-06-23
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */
// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mpc_utils/math/trigonometry.hpp"

#include "mpc_utils/math/constants.hpp"
#include "mpc_utils/math/sin_table.hpp"

#include <cmath>

namespace mpc_utils {

float sin(float radian)
{
    float  degree = radian * (180.f / static_cast<float>(mpc_utils::pi)) * (discrete_arcs_num_360 / 360.f);
    size_t idx = (static_cast<int>(std::round(degree)) % discrete_arcs_num_360 + discrete_arcs_num_360) % discrete_arcs_num_360;

    float mul = 1.f;
    if (discrete_arcs_num_90 <= idx && idx < 2 * discrete_arcs_num_90) {
        idx = 2 * discrete_arcs_num_90 - idx;
    }
    else if (2 * discrete_arcs_num_90 <= idx && idx < 3 * discrete_arcs_num_90) {
        mul = -1.f;
        idx = idx - 2 * discrete_arcs_num_90;
    }
    else if (3 * discrete_arcs_num_90 <= idx && idx < 4 * discrete_arcs_num_90) {
        mul = -1.f;
        idx = 4 * discrete_arcs_num_90 - idx;
    }

    return mul * g_sin_table[idx];
}

float cos(float radian)
{
    return sin(radian + static_cast<float>(mpc_utils::pi) / 2.f);
}

}   // namespace mpc_utils
