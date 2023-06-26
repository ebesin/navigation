/*
 * @Author       : dwayne
 * @Date         : 2023-06-26
 * @LastEditTime : 2023-06-26
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#include "bicycle_kinematics.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class SimAckermann : public rclcpp::Node
{
private:
    /* data */
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr      odom_publisher_;
    rclcpp::TimerBase::SharedPtr                               pub_timer;

    bool is_init_ = false;

    std::shared_ptr<sim_robot::BicycleKinematics::State> current_state_ptr_;
    geometry_msgs::msg::Twist::SharedPtr                 current_cmd_ptr_;
    std::shared_ptr<sim_robot::BicycleKinematics>        simulator_ptr_;

    rclcpp::Time current_time_;
    rclcpp::Time last_time_;

    /*parameters*/
    double      origin_x_;         //初始点x
    double      origin_y_;         //初始点y
    double      origin_phi_;       //初始航向
    double      pub_period_;       //机器人状态发布间隔
    double      min_sim_time_;     //最小仿真时间
    double      wheel_base_;       //机器人轴距
    std::string cmd_sub_topic_;    //控制指令接收话题名
    std::string odom_pub_toipc_;   //里程计发布话题名

    /**
     * @description  : 声明参数
     * @return        {*}
     */
    void declareParameter();

    /**
     * @description  : 控制指令回调
     * @param         {SharedPtr} cmd_vel:
     * @return        {*}
     */
    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel);

    /**
     * @description  : 定时器回调
     * @return        {*}
     */
    void timerCallback();

    /**
     * @description  : 通过航向获得四元数
     * @param         {double} yaw:
     * @return        {*}
     */
    geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw);


public:
    SimAckermann(std::string name);
    ~SimAckermann();
};