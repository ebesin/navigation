/*
 * @Author       : dwayne
 * @Date         : 2023-06-21
 * @LastEditTime : 2023-06-24
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef TIER4_AUTOWARE_UTILS__ROS__DEBUG_TRAITS_HPP_
#define TIER4_AUTOWARE_UTILS__ROS__DEBUG_TRAITS_HPP_

#include <tier4_debug_msgs/msg/bool_stamped.hpp>
#include <tier4_debug_msgs/msg/float32_multi_array_stamped.hpp>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>
#include <tier4_debug_msgs/msg/float64_multi_array_stamped.hpp>
#include <tier4_debug_msgs/msg/float64_stamped.hpp>
#include <tier4_debug_msgs/msg/int32_multi_array_stamped.hpp>
#include <tier4_debug_msgs/msg/int32_stamped.hpp>
#include <tier4_debug_msgs/msg/int64_multi_array_stamped.hpp>
#include <tier4_debug_msgs/msg/int64_stamped.hpp>
#include <tier4_debug_msgs/msg/string_stamped.hpp>

#include <type_traits>

namespace mpc_utils::debug_traits {
template<typename T> struct is_debug_message : std::false_type
{};

template<> struct is_debug_message<tier4_debug_msgs::msg::BoolStamped> : std::true_type
{};

template<> struct is_debug_message<tier4_debug_msgs::msg::Float32MultiArrayStamped> : std::true_type
{};

template<> struct is_debug_message<tier4_debug_msgs::msg::Float32Stamped> : std::true_type
{};

template<> struct is_debug_message<tier4_debug_msgs::msg::Float64MultiArrayStamped> : std::true_type
{};

template<> struct is_debug_message<tier4_debug_msgs::msg::Float64Stamped> : std::true_type
{};

template<> struct is_debug_message<tier4_debug_msgs::msg::Int32MultiArrayStamped> : std::true_type
{};

template<> struct is_debug_message<tier4_debug_msgs::msg::Int32Stamped> : std::true_type
{};

template<> struct is_debug_message<tier4_debug_msgs::msg::Int64MultiArrayStamped> : std::true_type
{};

template<> struct is_debug_message<tier4_debug_msgs::msg::Int64Stamped> : std::true_type
{};

template<> struct is_debug_message<tier4_debug_msgs::msg::StringStamped> : std::true_type
{};
}   // namespace mpc_utils::debug_traits

#endif   // TIER4_AUTOWARE_UTILS__ROS__DEBUG_TRAITS_HPP_
