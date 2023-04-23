/*
 * @Author       : dwayne
 * @Date         : 2023-04-13
 * @LastEditTime : 2023-04-18
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */


#include "3rd/backward.hpp"
#include "guided_hybrid_astar.hpp"
#include "gtest/gtest.h"
#include <iostream>


namespace backward {
backward::SignalHandling sh;
}

using namespace guided_hybrid_a_star;

TEST(guided_hybrid_astar, test_getAngleIndex)
{
    std::cout << "test_getAngleIndex...." << std::endl;
    SearchInfo                              serch_info;
    guided_hybrid_a_star::GuidedHybridAStar astar(10, 10, 70, serch_info);
    // EXPECT_TRUE(astar.getAngleIndex(1.2));
    EXPECT_EQ(astar.getAngleIndex(0), 0);
    EXPECT_EQ(astar.getAngleIndex(3.14), 34);
    EXPECT_EQ(astar.getAngleIndex(6.28), 69);
    EXPECT_EQ(astar.getAngleIndex(6.29), 0);
}

TEST(guided_hybrid_astar, test_state2Index)
{
    std::cout << "test_state2Index...." << std::endl;
    SearchInfo                              serch_info;
    guided_hybrid_a_star::GuidedHybridAStar astar(10, 10, 70, serch_info);
    // EXPECT_TRUE(astar.getAngleIndex(1.2));
    // EXPECT_EQ(astar.state2Index(Vec3d(1, 1, 0)), Vec3i(1, 1, 0));
    // EXPECT_EQ(astar.state2Index(Vec3d(0.2, 2.5, 0)), Vec3i(0, 2, 0));
    // EXPECT_EQ(astar.state2Index(Vec3d(1, 1, 0)), Vec3i(1, 1, 0));
    // EXPECT_EQ(astar.state2Index(Vec3d(1, 1, 6.28)), Vec3i(1, 1, 69));
    // EXPECT_EQ(astar.state2Index(Vec3d(1, 1, 6.29)), Vec3i(1, 1, 0));
}

TEST(guided_hybrid_astar, test_initMotion)
{
    std::cout << "test_initMotion...." << std::endl;
    SearchInfo serch_info;
    serch_info.max_steer_angle          = M_PI / 6;
    serch_info.wheel_base               = 1;
    serch_info.steer_angle_segment_size = 10;
    guided_hybrid_a_star::GuidedHybridAStar astar(10, 10, 70, serch_info);
    astar.initMotion();
}

TEST(guided_hybrid_astar, test_getFrontNeighbor)
{
    std::cout << "test_getFrontNeighbor...." << std::endl;
    SearchInfo serch_info;
    serch_info.max_steer_angle          = M_PI / 6;
    serch_info.wheel_base               = 3;
    serch_info.steer_angle_segment_size = 10;
    guided_hybrid_a_star::GuidedHybridAStar astar(10, 10, 70, serch_info);
    astar.initMotion();
    // guided_hybrid_a_star::StateNode current_node(300);
    std::shared_ptr<guided_hybrid_a_star::StateNode> current_node =
        std::make_shared<guided_hybrid_a_star::StateNode>(300);
    Vec3d current_state(4.3, 6.2, M_PI);
    current_node->setState(current_state);
    std::shared_ptr<guided_hybrid_a_star::StateNode> next_node =
        std::make_shared<guided_hybrid_a_star::StateNode>();
    EXPECT_TRUE(astar.getFrontNeighbor(current_node, 2, next_node));
    std::cout << "next_node_x: " << next_node->getState()[0]
              << " next_node_y: " << next_node->getState()[1]
              << " next_node_theta: " << next_node->getState()[2] << std::endl;

    EXPECT_TRUE(astar.getFrontNeighbor(current_node, 3, next_node));
    std::cout << "next_node_x: " << next_node->getState()[0]
              << " next_node_y: " << next_node->getState()[1]
              << " next_node_theta: " << next_node->getState()[2] << std::endl;

    EXPECT_TRUE(astar.getFrontNeighbor(current_node, 6, next_node));
    std::cout << "next_node_x: " << next_node->getState()[0]
              << " next_node_y: " << next_node->getState()[1]
              << " next_node_theta: " << next_node->getState()[2] << std::endl;

    EXPECT_TRUE(astar.getBackNeighbor(current_node, 6, next_node));
    std::cout << "next_node_x: " << next_node->getState()[0]
              << " next_node_y: " << next_node->getState()[1]
              << " next_node_theta: " << next_node->getState()[2] << std::endl;
}

TEST(guided_hybrid_astar, test_computeDistanceHeuristicCost)
{
    std::cout << "test_computeDistanceHeuristicCost...." << std::endl;
    SearchInfo serch_info;
    serch_info.max_steer_angle          = M_PI / 6;
    serch_info.wheel_base               = 1;
    serch_info.steer_angle_segment_size = 10;
    serch_info.shot_distance            = 3;
    guided_hybrid_a_star::GuidedHybridAStar          astar(10, 10, 70, serch_info);
    std::shared_ptr<guided_hybrid_a_star::StateNode> current_node =
        std::make_shared<guided_hybrid_a_star::StateNode>();
    current_node->setState(Vec3d(1.0, 1.0, 0));
    std::shared_ptr<guided_hybrid_a_star::StateNode> target_node =
        std::make_shared<guided_hybrid_a_star::StateNode>();

    Color::Modifier blue(Color::FG_BLUE);

    current_node->setState(Vec3d(1.0, 1.0, M_PI));
    target_node->setState(Vec3d(2.0, 1.0, M_PI));
    std::cout << blue << Flag::Modifier("DistanceHeuristicCost")
              << "computeDistanceHeuristicCost from 1.0, 1.0, M_PI) to (2.0, 1.0, M_PI): "
              << astar.computeDistanceHeuristicCost(current_node, target_node) << std::endl;

    current_node->setState(Vec3d(1.0, 1.0, 0));
    target_node->setState(Vec3d(2.0, 1.0, 0));
    std::cout << blue << Flag::Modifier("DistanceHeuristicCost")
              << "computeDistanceHeuristicCost from (1.0, 1.0, 0) to (2.0, 1.0, 0): "
              << astar.computeDistanceHeuristicCost(current_node, target_node) << std::endl;

    current_node->setState(Vec3d(1.0, 1.0, 0));
    target_node->setState(Vec3d(5.0, 1.0, 0));
    std::cout << blue << Flag::Modifier("DistanceHeuristicCost")
              << "computeDistanceHeuristicCost from (1.0, 1.0, 0) to (5.0, 1.0, 0): "
              << astar.computeDistanceHeuristicCost(current_node, target_node) << std::endl;
}

TEST(guided_hybrid_astar, test_share_ptr)
{
    GuidedHybridAStar::StateNodePtr              node = std::make_shared<StateNode>(10);
    std::vector<GuidedHybridAStar::StateNodePtr> nodevec;
    nodevec.push_back(node);
    node = std::make_shared<StateNode>(20);
    std::cout << Color::Modifier(Color::FG_BLUE) << Flag::Modifier("test_share_ptr")
              << "nodevec[0]:" << nodevec[0]->getIndex() << std::endl;
}


int main(int argc, char* argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}