/*
 * @Author       : dwayne
 * @Date         : 2023-06-26
 * @LastEditTime : 2023-06-26
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */


#include "bicycle_kinematics.hpp"

using namespace sim_robot;

BicycleKinematics::BicycleKinematics(double wheel_base, rclcpp::Duration min_sim_time)
    : wheel_base_(wheel_base)
    , min_sim_time_(min_sim_time)
{}

void BicycleKinematics::calKinematics(BicycleKinematics::State&           current_state,
                                      const BicycleKinematics::CtrlInput& u,
                                      const rclcpp::Duration              sim_time)
{
    rclcpp::Duration time = rclcpp::Duration::from_nanoseconds(0);
    while (time < sim_time) {
        double time_interval = min_sim_time_.seconds();
        current_state        = current_state + State(current_state.x_ * cos(current_state.phi_) * time_interval,
                                              current_state.y_ * sin(current_state.phi_) * time_interval,
                                              u.v / wheel_base_ * tan(u.delta) * time_interval);
        time                 = time + min_sim_time_;
    }
    double time_interval = (sim_time - (time - sim_time)).seconds();
    current_state        = current_state + State(current_state.x_ * cos(current_state.phi_) * time_interval,
                                          current_state.y_ * sin(current_state.phi_) * time_interval,
                                          u.v / wheel_base_ * tan(u.delta) * time_interval);
}