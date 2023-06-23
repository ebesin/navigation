/*
 * @Author       : dwayne
 * @Date         : 2023-06-21
 * @LastEditTime : 2023-06-21
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef MPC_UTILS__MATH__SIN_TABLE_HPP_
#define MPC_UTILS__MATH__SIN_TABLE_HPP_

#include <cstddef>

namespace mpc_utils {

constexpr size_t   sin_table_size        = 32769;
constexpr size_t   discrete_arcs_num_90  = 32768;
constexpr size_t   discrete_arcs_num_360 = 131072;
extern const float g_sin_table[sin_table_size];

}   // namespace mpc_utils

#endif   // MPC_UTILS__MATH__SIN_TABLE_HPP_
