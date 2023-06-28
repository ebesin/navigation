/*
 * @Author       : dwayne
 * @Date         : 2023-06-28
 * @LastEditTime : 2023-06-28
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */
#pragma once

#include "coor_trans.hpp"
#include "mpc_msgs/msg/trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>

namespace utils_tool {
using mpc_msgs::msg::Trajectory;
class PathGenerator : public rclcpp::Node
{
private:
    /* data */
    rclcpp::Publisher<Trajectory>::SharedPtr trajectory_publisher_;
    Trajectory                               traj_;
    std::string                              traj_type_;
    double                                   resulation_;
    double                                   velocity_;

public:
    PathGenerator(/* args */ std::string name);
    ~PathGenerator();

    /**
     * @description  : 完善路径信息（包括航向、转角、速度等）
     * @return        {*}
     */
    void perfectPath(Trajectory& traj, double velocity);

    /**
     * @description  : 生成sin曲线路径
     * @return        {*}
     */
    void generateSinCurve();

    /**
     * @description  : 生成直线路径
     * @param         {double} length: 路径长度
     * @param         {double} heading:路径方向
     * @return        {*}
     */
    void generateLineCurve(double length, double heading);

    /**
     * @description  : 生成圆形路径
     * @return        {*}
     */
    void generateCircleCurve();
};
}   // namespace utils_tool