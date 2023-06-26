/*
 * @Author       : dwayne
 * @Date         : 2023-04-08
 * @LastEditTime : 2023-06-26
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#include "3rd/backward.hpp"
#include "guided_hybrid_astar_flow.hpp"
#include "rclcpp/rclcpp.hpp"

namespace backward {
backward::SignalHandling sh;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<guided_hybrid_a_star::GuidedHybridAstarFlow>("guided_hybrid_planner_flow");
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
