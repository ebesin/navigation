/*
 * @Author       : dwayne
 * @Date         : 2023-06-28
 * @LastEditTime : 2023-06-30
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */
#pragma once

#include "coor_trans.hpp"
#include "mpc_msgs/msg/trajectory.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
namespace utils_tool {
using mpc_msgs::msg::Trajectory;
class PathGenerator : public rclcpp::Node
{
private:
    /* data */
    rclcpp::Publisher<Trajectory>::SharedPtr          trajectory_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    Trajectory                                        traj_;
    std::string                                       traj_type_;
    double                                            resulation_;
    double                                            velocity_;
    double                                            wheel_base_;

    double path_length_;
    /*路径方向，仅对直线有用*/
    double path_heading_;

public:
    PathGenerator(/* args */ std::string name);
    ~PathGenerator();

    /**
     * @description  : 完善路径信息（转角、速度等）
     * @return        {*}
     */
    //todo 目前只考虑了前进，后续需要考虑倒车
    void perfectPath(Trajectory& traj, double velocity, double wheel_base);

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