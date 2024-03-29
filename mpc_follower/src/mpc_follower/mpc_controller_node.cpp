/*
 * @Author       : dwayne
 * @Date         : 2023-06-28
 * @LastEditTime : 2023-06-29
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#include "3rd/backward.hpp"
#include "mpc_follower/mpc_controller.hpp"
namespace backward {
backward::SignalHandling sh;
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // auto node = ;
    rclcpp::spin(std::make_shared<MpcController>("mpc_controller"));
    rclcpp::shutdown();
    return 0;
}