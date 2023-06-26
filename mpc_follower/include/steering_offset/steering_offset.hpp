/*
 * @Author       : dwayne
 * @Date         : 2023-06-24
 * @LastEditTime : 2023-06-24
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */
#pragma once

#include <geometry_msgs/msg/twist.hpp>

#include <cmath>
#include <deque>
#include <vector>

class SteeringOffsetEstimator
{
public:
    SteeringOffsetEstimator(double wheelbase, double average_num, double vel_thres, double steer_thres, double offset_limit);
    ~SteeringOffsetEstimator() = default;

    double getOffset() const;
    void   updateOffset(const geometry_msgs::msg::Twist& twist, const double steering);

private:
    // parameters
    double wheelbase_              = 3.0;
    size_t average_num_            = 1000;
    double update_vel_threshold_   = 8.0;
    double update_steer_threshold_ = 0.05;
    double offset_limit_           = 0.02;

    // results
    std::deque<double> steering_offset_storage_;
    double             steering_offset_ = 0.0;
};