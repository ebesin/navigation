/*
 * @Author       : dwayne
 * @Date         : 2023-06-21
 * @LastEditTime : 2023-06-21
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef MPC_LATERAL_CONTROLLER__STEERING_PREDICTOR_HPP_
#define MPC_LATERAL_CONTROLLER__STEERING_PREDICTOR_HPP_

#include "rclcpp/rclcpp.hpp"

#include "mpc_msgs/msg/ackermann_lateral_command.hpp"

#include <memory>
#include <vector>

using mpc_msgs::msg::AckermannLateralCommand;

class SteeringPredictor
{
public:
    SteeringPredictor(const double steer_tau, const double steer_delay);
    ~SteeringPredictor() = default;

    /**
   * @brief Calculate the predicted steering based on the given vehicle model.
   * @return The predicted steering angle.
   */
    double calcSteerPrediction();

    /**
   * @brief Store the steering command in the buffer.
   * @param steer The steering command to be stored.
   */
    void storeSteerCmd(const double steer);

private:
    rclcpp::Logger           m_logger = rclcpp::get_logger("mpc_steer_predictor");
    rclcpp::Clock::SharedPtr m_clock  = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

    // The previously predicted steering value.
    double m_steer_prediction_prev = 0.0;

    // Previous computation time.
    rclcpp::Time m_time_prev = rclcpp::Time(0, 0, RCL_ROS_TIME);

    // time constant for the steering dynamics
    double m_steer_tau;

    // time delay for the steering dynamics
    double m_input_delay;

    // Buffer of sent control commands.
    std::vector<AckermannLateralCommand> m_ctrl_cmd_vec;

    /**
   * @brief Get the sum of all steering commands over the given time range.
   * @param t_start The start time of the range.
   * @param t_end The end time of the range.
   * @param time_constant The time constant for the sum calculation.
   * @return The sum of the steering commands.
   */
    double getSteerCmdSum(const rclcpp::Time& t_start, const rclcpp::Time& t_end, const double time_constant) const;

    /**
   * @brief Set previously calculated steering
   */
    void setPrevResult(const double& steering);
};

#endif   // MPC_LATERAL_CONTROLLER__STEERING_PREDICTOR_HPP_
