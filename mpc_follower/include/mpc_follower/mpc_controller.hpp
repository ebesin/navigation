/*
 * @Author       : dwayne
 * @Date         : 2023-06-24
 * @LastEditTime : 2023-06-30
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */
#pragma once

#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mpc_follower/mpc.hpp"
#include "mpc_follower/mpc_trajectory.hpp"
#include "mpc_follower/mpc_utils.hpp"
#include "mpc_follower/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.hpp"
#include "qp_solver/qp_solver_osqp.hpp"
#include "qp_solver/qp_solver_unconstr_fast.hpp"

#include "mpc_msgs/msg/ackermann_lateral_command.hpp"
#include "mpc_msgs/msg/operation_mode_state.hpp"
#include "mpc_msgs/msg/steering_report.hpp"
#include "mpc_msgs/msg/trajectory.hpp"
#include "mpc_msgs/msg/vehicle_odometry.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "steering_offset/steering_offset.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tier4_debug_msgs/msg/float32_multi_array_stamped.hpp"
#include "tier4_debug_msgs/msg/float32_stamped.hpp"
#include "tier4_debug_msgs/msg/multi_array_layout.hpp"

#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_util/lifecycle_node.hpp"

using mpc_msgs::msg::AckermannLateralCommand;
using mpc_msgs::msg::Trajectory;
using mpc_msgs::msg::SteeringReport;
using nav_msgs::msg::Odometry;
// using tier4_debug_msgs::msg::Float32MultiArrayStamped;
using tier4_debug_msgs::msg::Float32Stamped;

class MpcController : public rclcpp::Node
{
public:
    MpcController(const std::string& name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~MpcController();


    struct InputData
    {
        mpc_msgs::msg::Trajectory                      current_trajectory;
        nav_msgs::msg::Odometry                        current_odometry;
        mpc_msgs::msg::SteeringReport                  current_steering;
        geometry_msgs::msg::AccelWithCovarianceStamped current_accel;
        mpc_msgs::msg::OperationModeState              current_operation_mode;
    };

    struct LateralSyncData
    {
        bool is_steer_converged{false};
    };


    struct LateralOutput
    {
        mpc_msgs::msg::AckermannLateralCommand control_cmd;
        LateralSyncData                        sync_data;
    };

    //     /**
    //    * @brief Configure node
    //    */
    void configure();

    //     /**
    //    * @brief Activate node
    //    */
    //     nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

    //     /**
    //    * @brief Deactivate node
    //    */
    //     nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;

    //     /**
    //    * @brief Cleanup node
    //    */
    //     nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;

    //     /**
    //    * @brief shutdown node
    //    */
    //     nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

private:
    rclcpp::TimerBase::SharedPtr timer_control_;
    double                       timeout_thr_sec_;

    rclcpp::Subscription<Trajectory>::SharedPtr                                     ref_path_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                        odometry_subscriber_;
    rclcpp::Subscription<mpc_msgs::msg::SteeringReport>::SharedPtr                  steering_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr accel_subscriber_;
    rclcpp::Subscription<mpc_msgs::msg::OperationModeState>::SharedPtr              operation_mode_subscriber_;
    rclcpp::Publisher<mpc_msgs::msg::AckermannLateralCommand>::SharedPtr            control_cmd_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr              debug_marker_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr                         cmd_publisher_;


    void callbackTimerControl();
    void onTrajectory(const mpc_msgs::msg::Trajectory::SharedPtr);
    void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void onSteering(const mpc_msgs::msg::SteeringReport::SharedPtr msg);
    void onAccel(const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg);

    mpc_msgs::msg::Trajectory::SharedPtr                      current_trajectory_ptr_;
    nav_msgs::msg::Odometry::SharedPtr                        current_odometry_ptr_;
    mpc_msgs::msg::SteeringReport::SharedPtr                  current_steering_ptr_;
    geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr current_accel_ptr_;
    mpc_msgs::msg::OperationModeState::SharedPtr              current_operation_mode_ptr_;


    rclcpp::Publisher<Trajectory>::SharedPtr                                      predicted_traj_publisher_;
    rclcpp::Publisher<tier4_debug_msgs::msg::Float32MultiArrayStamped>::SharedPtr debug_values_publisher_;
    rclcpp::Publisher<Float32Stamped>::SharedPtr                                  steer_offset_publisher_;


    //!< @brief parameters for path smoothing
    TrajectoryFilteringParam trajectory_filtering_param_;

    // Ego vehicle speed threshold to enter the stop state.
    double stop_state_entry_ego_speed_;

    // Target vehicle speed threshold to enter the stop state.
    double stop_state_entry_target_speed_;

    // Convergence threshold for steering control.
    double converged_steer_rad_;

    // max mpc output change threshold for 1 sec
    double mpc_converged_threshold_rps_;

    // Time duration threshold to check if the trajectory shape has changed.
    double new_traj_duration_time_;

    // Distance threshold to check if the trajectory shape has changed.
    double new_traj_end_dist_;

    // Flag indicating whether to keep the steering control until it converges.
    bool keep_steer_control_until_converged_;

    // trajectory buffer for detecting new trajectory
    std::deque<Trajectory> trajectory_buffer_;

    MPC mpc_;

    // Check is mpc output converged
    bool is_mpc_history_filled_{false};

    // store the last mpc outputs for 1 sec
    std::vector<std::pair<AckermannLateralCommand, rclcpp::Time>> mpc_steering_history_{};

    // set the mpc steering output to history
    void setSteeringToHistory(const AckermannLateralCommand& steering);

    // check if the mpc steering output is converged
    bool isMpcConverged();

    // measured kinematic state
    Odometry current_kinematic_state_;

    SteeringReport current_steering_;   // Measured steering information.

    Trajectory current_trajectory_;     // Current reference trajectory for path following.

    double steer_cmd_prev_ = 0.0;       // MPC output in the previous period.

    // Flag indicating whether the previous control command is initialized.
    bool is_ctrl_cmd_prev_initialized_ = false;

    // Previous control command for path following.
    AckermannLateralCommand ctrl_cmd_prev_;

    //  Flag indicating whether the first trajectory has been received.
    bool has_received_first_trajectory_ = false;

    // Threshold distance for the ego vehicle in nearest index search.
    double ego_nearest_dist_threshold_;

    // Threshold yaw for the ego vehicle in nearest index search.
    double ego_nearest_yaw_threshold_;

    // Flag indicating whether auto steering offset removal is enabled.
    bool enable_auto_steering_offset_removal_;

    // Steering offset estimator for offset compensation.
    std::shared_ptr<SteeringOffsetEstimator> steering_offset_;


    /**
   * @brief Initialize the timer
   * @param period_s Control period in seconds.
   */
    void initTimer(double period_s);

    /**
   * @brief Create the vehicle model based on the provided parameters.
   * @param wheelbase Vehicle's wheelbase.
   * @param steer_lim Steering command limit.
   * @param steer_tau Steering time constant.
   * @return Pointer to the created vehicle model.
   */
    std::shared_ptr<VehicleModelInterface> createVehicleModel(const double wheelbase,
                                                              const double steer_lim,
                                                              const double steer_tau);


    /**
   * @brief Create the quadratic problem solver interface.
   * @return Pointer to the created QP solver interface.
   */
    std::shared_ptr<QPSolverInterface> createQPSolverInterface();

    /**
   * @brief Create the steering offset estimator for offset compensation.
   * @param wheelbase Vehicle's wheelbase.
   * @return Pointer to the created steering offset estimator.
   */
    std::shared_ptr<SteeringOffsetEstimator> createSteerOffsetEstimator(const double wheelbase);

    /**
   * @brief Check if all necessary data is received and ready to run the control.
   * @param input_data Input data required for control calculation.
   * @return True if the data is ready, false otherwise.
   */
    bool isReady(const InputData& input_data);

    /**
   * @brief Compute the control command for path following with a constant control period.
   * @param input_data Input data required for control calculation.
   * @return Lateral output control command.
   */
    LateralOutput run(MpcController::InputData const& input_data);

    geometry_msgs::msg::Twist run2(MpcController::InputData const& input_data);

    /**
   * @brief Set the current trajectory using the received message.
   * @param msg Received trajectory message.
   */
    void setTrajectory(const Trajectory& msg);

    /**
   * @brief Check if the received data is valid.
   * @return True if the data is valid, false otherwise.
   */
    bool checkData() const;

    /**
   * @brief Create the control command.
   * @param ctrl_cmd Control command to be created.
   * @return Created control command.
   */
    AckermannLateralCommand createCtrlCmdMsg(const AckermannLateralCommand& ctrl_cmd);

    /**
   * @brief Publish the predicted future trajectory.
   * @param predicted_traj Predicted future trajectory to be published.
   */
    void publishPredictedTraj(Trajectory& predicted_traj) const;

    /**
   * @brief Publish diagnostic message.
   * @param diagnostic Diagnostic message to be published.
   */
    void publishDebugValues(tier4_debug_msgs::msg::Float32MultiArrayStamped& diagnostic) const;

    /**
   * @brief Get the stop control command.
   * @return Stop control command.
   */
    AckermannLateralCommand getStopControlCommand() const;

    /**
   * @brief Get the control command applied before initialization.
   * @return Initial control command.
   */
    AckermannLateralCommand getInitialControlCommand() const;

    /**
   * @brief Check if the ego car is in a stopped state.
   * @return True if the ego car is stopped, false otherwise.
   */
    bool isStoppedState() const;

    /**
   * @brief Check if the trajectory has a valid value.
   * @param traj Trajectory to be checked.
   * @return True if the trajectory is valid, false otherwise.
   */
    bool isValidTrajectory(const Trajectory& traj) const;

    /**
   * @brief Check if the trajectory shape has changed.
   * @return True if the trajectory shape has changed, false otherwise.
   */
    bool isTrajectoryShapeChanged() const;

    /**
   * @brief Check if the steering control is converged and stable now.
   * @param cmd Steering control command to be checked.
   * @return True if the steering control is converged and stable, false otherwise.
   */
    bool isSteerConverged(const AckermannLateralCommand& cmd) const;

    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr m_set_param_res;

    /**
   * @brief Declare MPC parameters as ROS parameters to allow tuning on the fly.
   */
    void declareMPCparameters();


    boost::optional<InputData> createInputData(rclcpp::Clock& clock);

    /**
   * @brief Callback function called when parameters are changed outside of the node.
   * @param parameters Vector of changed parameters.
   * @return Result of the parameter callback.
   */
    rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter>& parameters);

    // template<typename... Args> inline void info_throttle(Args&&... args)
    // {
    //     RCLCPP_INFO_THROTTLE(get_logger(), get_clock(), 5000, args...);
    // }

    // template<typename... Args> inline void warn_throttle(Args&&... args)
    // {
    //     RCLCPP_WARN_THROTTLE(get_logger(), get_clock(), 5000, args...);
    // }
};