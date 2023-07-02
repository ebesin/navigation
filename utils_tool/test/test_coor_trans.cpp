/*
 * @Author       : dwayne
 * @Date         : 2023-06-29
 * @LastEditTime : 2023-06-29
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#include "3rd/backward.hpp"
#include "coor_trans.hpp"
#include "gtest/gtest.h"

namespace backward {
backward::SignalHandling sh;
}

TEST(utils_tool, test_getYawFromQuaternion)
{
    std::cout << "test_getYawFromQuaternion...." << std::endl;
    std::cout << "angle of [1.0, 0.0]: " << utils_tool::getHeadingFromVector(1.0, 0.0) << std::endl;
    // EXPECT_TRUE(astar.getAngleIndex(1.2));
    // EXPECT_EQ(astar.getAngleIndex(0), 0);
    // EXPECT_EQ(astar.getAngleIndex(3.14), 34);
    // EXPECT_EQ(astar.getAngleIndex(6.28), 69);
    // EXPECT_EQ(astar.getAngleIndex(6.29), 0);
}


int main(int argc, char* argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}