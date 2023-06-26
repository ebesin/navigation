/*
 * @Author       : dwayne
 * @Date         : 2023-06-26
 * @LastEditTime : 2023-06-26
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#include "rclcpp/rclcpp.hpp"
#include "sim_ackermann.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimAckermann>("sim_ackermann"));
    rclcpp::shutdown();
    return 0;
}