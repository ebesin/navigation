/*
 * @Author       : dwayne
 * @Date         : 2023-06-19
 * @LastEditTime : 2023-06-20
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#pragma once
#include <chrono>
#include <deque>
#include <iostream>
#include <limits>
#include <unistd.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/utils.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.h>

#include "mpc_follower/lowpass_filter.h"
#include "mpc_follower/mpc_trajectory.h"
#include "mpc_follower/mpc_utils.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <mpc_msgs/mpc_msgs/msg/control_command_stamped.hpp>
#include <mpc_msgs/mpc_msgs/msg/lane.hpp>
#include <mpc_msgs/mpc_msgs/msg/vehicle_status.hpp>

#include "mpc_follower/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.h"


#include "nav2_util/lifecycle_node.hpp"


class MPCFollower : public nav2_util::LifecycleNode
{
    MPCFollower(const std::string& name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~MPCFollower();

    /**
   * @brief Configure node
   */
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;

    /**
   * @brief Activate node
   */
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

    /**
   * @brief Deactivate node
   */
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;

    /**
   * @brief Cleanup node
   */
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;

    /**
   * @brief shutdown node
   */
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;


private:
    //控制指令发布话题
    rclcpp_lifecycle::LifecyclePublisher<mpc_msgs::msg::ControlCommandStamped>::SharedPtr steer_vel_ctrl_cmd_publisher_;
    //twist发布话题
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_publisher_;
    //参考线订阅话题
    rclcpp::Subscription<mpc_msgs::msg::Lane>::SharedPtr ref_path_subscriber_;
    //当前位姿订阅话题
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr current_pose_subscriber_;
    //当前车辆状态订阅话题
    rclcpp::Subscription<mpc_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscriber_;
    //控制定时器
    rclcpp::TimerBase::SharedPtr control_timer_;
    //reference line
    MPCTrajectory ref_traj_;
    //需要跟踪的航点
    mpc_msgs::msg::Lane current_waypoints_;
    //车辆模型
    std::shared_ptr<VehicleModelInterface> vehicle_model_ptr_;

    std::string         output_interface_;    //!< @brief output command type
    std::deque<double>  input_buffer_;        //!< @brief control input (mpc_output) buffer for delay time conpemsation
    Butterworth2dFilter lpf_steering_cmd_;    //!< @brief lowpass filter for steering command
    Butterworth2dFilter lpf_lateral_error_;   //!< @brief lowpass filter for lateral error to calculate derivatie
    Butterworth2dFilter lpf_yaw_error_;       //!< @brief lowpass filter for heading error to calculate derivatie
    mpc_msgs::msg::Lane current_waypoints_;   //!< @brief current waypoints to be followed
    std::shared_ptr<VehicleModelInterface> vehicle_model_ptr_;    //!< @brief vehicle model for MPC
    std::string                            vehicle_model_type_;   //!< @brief vehicle model type for MPC
    // std::shared_ptr<QPSolverInterface>     qpsolver_ptr_;         //!< @brief qp solver for MPC
    std::string        output_interface_;   //!< @brief output command type
    std::deque<double> input_buffer_;       //!< @brief control input (mpc_output) buffer for delay time conpemsation


    /* parameters for control*/
    double ctrl_period_;                //!< @brief control frequency [s]
    double steering_lpf_cutoff_hz_;     //!< @brief cutoff frequency of lowpass filter for steering command [Hz]
    double admisible_position_error_;   //!< @brief stop MPC calculation when lateral error is large than this value [m]
    double admisible_yaw_error_deg_;    //!< @brief stop MPC calculation when heading error is large than this value [deg]
    double steer_lim_deg_;              //!< @brief steering command limit [rad]
    double wheelbase_;                  //!< @brief vehicle wheelbase length [m] to convert steering angle to angular velocity

    /* parameters for path smoothing */
    bool   enable_path_smoothing_;        //< @brief flag for path smoothing
    bool   enable_yaw_recalculation_;     //< @brief flag for recalculation of yaw angle after resampling
    int    path_filter_moving_ave_num_;   //< @brief param of moving average filter for path smoothing
    int    path_smoothing_times_;         //< @brief number of times of applying path smoothing filter
    int    curvature_smoothing_num_;      //< @brief point-to-point index distance used in curvature calculation
    double traj_resample_dist_;           //< @brief path resampling interval [m]

    struct MPCParam
    {
        int    prediction_horizon;                        //< @brief prediction horizon step
        double prediction_sampling_time;                  //< @brief prediction horizon period
        double weight_lat_error;                          //< @brief lateral error weight in matrix Q
        double weight_heading_error;                      //< @brief heading error weight in matrix Q
        double weight_heading_error_squared_vel_coeff;    //< @brief heading error * velocity weight in matrix Q
        double weight_steering_input;                     //< @brief steering error weight in matrix R
        double weight_steering_input_squared_vel_coeff;   //< @brief steering error * velocity weight in matrix R
        double weight_lat_jerk;                           //< @brief lateral jerk weight in matrix R
        double weight_terminal_lat_error;                 //< @brief terminal lateral error weight in matrix Q
        double weight_terminal_heading_error;             //< @brief terminal heading error weight in matrix Q
        double zero_ff_steer_deg;                         //< @brief threshold that feed-forward angle becomes zero
        double delay_compensation_time;                   //< @brief delay time for steering input to be compensated
    };

    MPCParam mpc_param_;   // for mpc design parameter
    struct VehicleStatus
    {
        std_msgs::msg::Header     header;           //< @brief header
        geometry_msgs::msg::Pose  pose;             //< @brief vehicle pose
        geometry_msgs::msg::Twist twist;            //< @brief vehicle velocity
        double                    tire_angle_rad;   //< @brief vehicle tire angle
    };

    VehicleStatus vehicle_status_;   //< @brief vehicle status

    double steer_cmd_prev_;          //< @brief steering command calculated in previous period
    double lateral_error_prev_;      //< @brief previous lateral error for derivative
    double yaw_error_prev_;          //< @brief previous lateral error for derivative

    /* flags */
    bool my_position_ok_;   //< @brief flag for validity of current pose
    bool my_velocity_ok_;   //< @brief flag for validity of current velocity
    bool my_steering_ok_;   //< @brief flag for validity of steering angle


    /**
   * @brief compute and publish control command for path follow with a constant control period
   */
    void timerCallback();

    /**
   * @brief set current_waypoints_ with receved message
   */
    void callbackRefPath(const mpc_msgs::msg::Lane::ConstPtr&);

    /**
   * @brief set vehicle_status_.pose with receved message
   */
    void callbackPose(const geometry_msgs::msg::PoseStamped::ConstPtr&);


    /**
   * @brief set vehicle_status_.twist and vehicle_status_.tire_angle_rad with receved message
   */
    void callbackVehicleStatus(const mpc_msgs::msg::VehicleStatus& msg);

    /**
   * @brief publish control command calculated by MPC
   * @param [in] vel_cmd velocity command [m/s] for vehicle control
   * @param [in] acc_cmd acceleration command [m/s2] for vehicle control
   * @param [in] steer_cmd steering angle command [rad] for vehicle control
   * @param [in] steer_vel_cmd steering angle speed [rad/s] for vehicle control
   */
    void publishControlCommands(const double& vel_cmd,
                                const double& acc_cmd,
                                const double& steer_cmd,
                                const double& steer_vel_cmd);

    /**
   * @brief publish control command as geometry_msgs/TwistStamped type
   * @param [in] vel_cmd velocity command [m/s] for vehicle control
   * @param [in] omega_cmd angular velocity command [rad/s] for vehicle control
   */
    void publishTwist(const double& vel_cmd, const double& omega_cmd);

    /**
   * @brief publish control command as autoware_msgs/ControlCommand type
   * @param [in] vel_cmd velocity command [m/s] for vehicle control
   * @param [in] acc_cmd acceleration command [m/s2] for vehicle control
   * @param [in] steer_cmd steering angle command [rad] for vehicle control
   */
    void publishCtrlCmd(const double& vel_cmd, const double& acc_cmd, const double& steer_cmd);

    /**
   * @brief calculate control command by MPC algorithm
   * @param [out] vel_cmd velocity command
   * @param [out] acc_cmd acceleration command
   * @param [out] steer_cmd steering command
   * @param [out] steer_vel_cmd steering rotation speed command
   */
    bool calculateMPC(double& vel_cmd, double& acc_cmd, double& steer_cmd, double& steer_vel_cmd);


    /* debug */
    bool show_debug_info_;   //!< @brief flag to display debug info

    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr
        debug_filtered_traj_publisher_;                            //!< @brief publisher for debug info
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr
        debug_predicted_traj_publisher_;                           //!< @brief publisher for debug info
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr
        debug_values_publisher_;                                   //!< @brief publisher for debug info
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr
        debug_mpc_calc_time_publisher_;                            //!< @brief publisher for debug info
    rclcpp::Subscription<mpc_msgs::msg::VehicleStatus>::SharedPtr
                                     estimate_twist_subscriber_;   //!< @brief subscriber for /estimate_twist for debug
    geometry_msgs::msg::TwistStamped estimate_twist_;              //!< @brief received /estimate_twist for debug

    /**
   * @brief convert MPCTraj to visualizaton marker for visualization
   */
    void convertTrajToMarker(const MPCTrajectory&             traj,
                             visualization_msgs::msg::Marker& markers,
                             std::string                      ns,
                             double                           r,
                             double                           g,
                             double                           b,
                             double                           z);

    /**
   * @brief callback for estimate twist for debug
   */
    void callbackEstimateTwist(const geometry_msgs::msg::TwistStamped& msg) { estimate_twist_ = msg; }
};
