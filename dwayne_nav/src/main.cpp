/*
 * @Author       : iPEK
 * @Date         : 2023-03-11
 * @LastEditTime : 2023-03-11
 * @Description  : 
 * 
 * Copyright (c) 2023 by iPEK, All Rights Reserved. 
 */
#include "base_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<base_node::BaseNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
