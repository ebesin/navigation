/*
 * @Author       : dwayne
 * @Date         : 2023-06-21
 * @LastEditTime : 2023-06-23
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef TIER4_AUTOWARE_UTILS__ROS__DEBUG_PUBLISHER_HPP_
#define TIER4_AUTOWARE_UTILS__ROS__DEBUG_PUBLISHER_HPP_

#include "mpc_utils/ros/debug_traits.hpp"

#include <rclcpp/publisher_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosidl_runtime_cpp/traits.hpp>

#include <memory>
#include <string>
#include <unordered_map>

namespace mpc_utils {
namespace debug_publisher {
template<class T_msg,
         class T,
         std::enable_if_t<mpc_utils::debug_traits::is_debug_message<T_msg>::value, std::nullptr_t> = nullptr>
T_msg toDebugMsg(const T& data, const rclcpp::Time& stamp)
{
    T_msg msg;
    msg.stamp = stamp;
    msg.data  = data;
    return msg;
}
}   // namespace debug_publisher

class DebugPublisher
{
public:
    explicit DebugPublisher(rclcpp::Node* node, const char* ns)
        : node_(node)
        , ns_(ns)
    {}

    template<class T, std::enable_if_t<rosidl_generator_traits::is_message<T>::value, std::nullptr_t> = nullptr>
    void publish(const std::string& name, const T& data, const rclcpp::QoS& qos = rclcpp::QoS(1))
    {
        if (pub_map_.count(name) == 0) {
            pub_map_[name] = node_->create_publisher<T>(std::string(ns_) + "/" + name, qos);
        }

        std::dynamic_pointer_cast<rclcpp::Publisher<T>>(pub_map_.at(name))->publish(data);
    }

    template<class T_msg, class T, std::enable_if_t<!rosidl_generator_traits::is_message<T>::value, std::nullptr_t> = nullptr>
    void publish(const std::string& name, const T& data, const rclcpp::QoS& qos = rclcpp::QoS(1))
    {
        publish(name, debug_publisher::toDebugMsg<T_msg>(data, node_->now()), qos);
    }

private:
    rclcpp::Node*                                                           node_;
    const char*                                                             ns_;
    std::unordered_map<std::string, std::shared_ptr<rclcpp::PublisherBase>> pub_map_;
};
}   // namespace mpc_utils

#endif   // TIER4_AUTOWARE_UTILS__ROS__DEBUG_PUBLISHER_HPP_
