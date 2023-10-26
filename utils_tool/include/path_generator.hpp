/*
 * @Author       : dwayne
 * @Date         : 2023-06-28
 * @LastEditTime : 2023-07-02
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
class PathGenerator : public rclcpp::Node {
private:
  /* data */
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  Trajectory traj_;
  nav_msgs::msg::Path path_;
  std::string traj_type_;
  double resulation_;
  double velocity_;
  double wheel_base_;
  double cycle_;
  double amplitude_;

  double path_length_;
  /*路径方向，仅对直线有用*/
  double path_heading_;

  /*运动方向 1-->前进  -1-->后退*/
  int direction_;

public:
  PathGenerator(/* args */ std::string name);
  ~PathGenerator();

  /**
   * @description  : 完善路径信息（转角、速度等）
   * @return        {*}
   */
  // todo 目前只考虑了前进，后续需要考虑倒车
  void perfectPath(Trajectory &traj, double velocity, double wheel_base,
                   int direction);

  /**
   * @description  : 生成sin曲线路径
   * @param         {double} length: 沿x轴的长度
   * @param         {double} heading: x轴方向
   * @param         {double} cycle: 周期
   * @param         {double} amplitude: 幅度
   * @return        {*}
   */
  void generateSinCurve(double length, double heading, double cycle,
                        double amplitude, int direction);

  /**
   * @description  : 生成直线路径
   * @param         {double} length: 路径长度
   * @param         {double} heading:路径方向
   * @return        {*}
   */
  void generateLineCurve(double length, double heading, int direction);

  /**
   * @description  : 生成圆形路径
   * @return        {*}
   */
  void generateCircleCurve();

  /**
   * @description  : 定时器回调
   * @return        {*}
   */
  void timerCallback();

  /**
   * @description  : 开启定时器
   * @return        {*}
   */
  void startTimer();
};
} // namespace utils_tool