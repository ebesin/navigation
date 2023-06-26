/*
 * @Author       : dwayne
 * @Date         : 2023-06-20
 * @LastEditTime : 2023-06-23
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#include "mpc_follower/mpc_follower_node.hpp"s
#define DEBUG_INFO(...)            \
    {                              \
        if (show_debug_info_) {    \
            ROS_INFO(__VA_ARGS__); \
        }                          \
    }

MPCFollower::MPCFollower(const std::string& name, const rclcpp::NodeOptions& options)
    : nav2_util::LifecycleNode(name, "", options)
    , my_position_ok_(false)
    , my_velocity_ok_(false)
    , my_steering_ok_(false)
{
    RCLCPP_INFO(get_logger(), "Creating MPCFollower...");

    declare_parameter("show_debug_info", rclcpp::ParameterValue(false));
    declare_parameter("ctrl_period", rclcpp::ParameterValue(true));
    declare_parameter("enable_path_smoothing", rclcpp::ParameterValue(false));
    declare_parameter("enable_yaw_recalculation", rclcpp::ParameterValue(false));
    declare_parameter("path_filter_moving_ave_num", rclcpp::ParameterValue(35));
    declare_parameter("path_smoothing_times", rclcpp::ParameterValue(1));
    declare_parameter("curvature_smoothing_num", rclcpp::ParameterValue(35));
    declare_parameter("traj_resample_dist", rclcpp::ParameterValue(0.1));
    declare_parameter("admisible_position_error", rclcpp::ParameterValue(5.0));
    declare_parameter("admisible_yaw_error_deg", rclcpp::ParameterValue(90.0));
    declare_parameter("output_interface", rclcpp::ParameterValue("all"));


    /* mpc parameters */
    declare_parameter("mpc_prediction_horizon", rclcpp::ParameterValue(70));
    declare_parameter("mpc_prediction_sampling_time", rclcpp::ParameterValue(0.1));
    declare_parameter("mpc_weight_lat_error", rclcpp::ParameterValue(1.0));
    declare_parameter("mpc_weight_heading_error", rclcpp::ParameterValue(0.0));
    declare_parameter("mpc_weight_heading_error_squared_vel_coeff", rclcpp::ParameterValue(0.3));
    declare_parameter("mpc_weight_steering_input", rclcpp::ParameterValue(1.0));
    declare_parameter("mpc_weight_steering_input_squared_vel_coeff", rclcpp::ParameterValue(0.25));
    declare_parameter("mpc_weight_lat_jerk", rclcpp::ParameterValue(0.0));
    declare_parameter("mpc_weight_terminal_lat_error", rclcpp::ParameterValue(1.0));
    declare_parameter("mpc_weight_terminal_heading_error", rclcpp::ParameterValue(0.1));
    declare_parameter("mpc_zero_ff_steer_deg", rclcpp::ParameterValue(2.0));
    declare_parameter("delay_compensation_time", rclcpp::ParameterValue(0.0));

    declare_parameter("steer_lim_deg", rclcpp::ParameterValue(35.0));
    declare_parameter("vehicle_model_wheelbase", rclcpp::ParameterValue(2.9));

    /* vehicle model setup */
    declare_parameter("vehicle_model_type", rclcpp::ParameterValue("kinematics_no_delay"));

    /* lowpass filter */
    declare_parameter("steering_lpf_cutoff_hz", rclcpp::ParameterValue(3.0));
    declare_parameter("error_deriv_lpf_curoff_hz", rclcpp::ParameterValue(5.0));

    /**/
    declare_parameter("out_twist_name", rclcpp::ParameterValue("twist_raw"));
    declare_parameter("out_vehicle_cmd_name", rclcpp::ParameterValue("ctrl_raw"));
    declare_parameter("in_waypoints_name", rclcpp::ParameterValue("base_waypoints"));
    declare_parameter("in_selfpose_name", rclcpp::ParameterValue("current_pose"));
    declare_parameter("in_vehicle_status_name", rclcpp::ParameterValue("vehicle_status"));
}


nav2_util::CallbackReturn MPCFollower::on_configure(const rclcpp_lifecycle::State& state)
{
    RCLCPP_INFO(get_logger(), "Configuring");
    auto node = shared_from_this();

    node->get_parameter("show_debug_info", show_debug_info_);
    node->get_parameter("ctrl_period", ctrl_period_);
    node->get_parameter("enable_path_smoothing", enable_path_smoothing_);
    node->get_parameter("enable_yaw_recalculation", enable_yaw_recalculation_);
    node->get_parameter("path_filter_moving_ave_num", path_filter_moving_ave_num_);
    node->get_parameter("path_smoothing_times", path_smoothing_times_);
    node->get_parameter("curvature_smoothing_num", curvature_smoothing_num_);
    node->get_parameter("traj_resample_dist", traj_resample_dist_);
    node->get_parameter("admisible_position_error", admisible_position_error_);
    node->get_parameter("admisible_yaw_error_deg", admisible_yaw_error_deg_);
    node->get_parameter("output_interface", output_interface_);

    /* mpc parameters */
    node->get_parameter("mpc_prediction_horizon", mpc_param_.prediction_horizon);
    node->get_parameter("mpc_prediction_sampling_time", mpc_param_.prediction_sampling_time);
    node->get_parameter("mpc_weight_lat_error", mpc_param_.weight_lat_error);
    node->get_parameter("mpc_weight_heading_error", mpc_param_.weight_heading_error);
    node->get_parameter("mpc_weight_heading_error_squared_vel_coeff", mpc_param_.weight_heading_error_squared_vel_coeff);
    node->get_parameter("mpc_weight_steering_input", mpc_param_.weight_steering_input);
    node->get_parameter("mpc_weight_steering_input_squared_vel_coeff", mpc_param_.weight_steering_input_squared_vel_coeff);
    node->get_parameter("mpc_weight_lat_jerk", mpc_param_.weight_lat_jerk);
    node->get_parameter("mpc_weight_terminal_lat_error", mpc_param_.weight_terminal_lat_error);
    node->get_parameter("mpc_weight_terminal_heading_error", mpc_param_.weight_terminal_heading_error);
    node->get_parameter("mpc_zero_ff_steer_deg", mpc_param_.zero_ff_steer_deg);
    node->get_parameter("delay_compensation_time", mpc_param_.delay_compensation_time);

    node->get_parameter("steer_lim_deg", steer_lim_deg_);
    node->get_parameter("vehicle_model_wheelbase", wheelbase_);

    /* vehicle model setup */
    // node->get_parameter("vehicle_model_type", vehicle_model_type_);
    vehicle_model_ptr_ = std::make_shared<KinematicsBicycleModelNoDelay>(wheelbase_, amathutils::deg2rad(steer_lim_deg_));


    steer_cmd_prev_     = 0.0;
    lateral_error_prev_ = 0.0;
    yaw_error_prev_     = 0.0;

    /* delay compensation */
    const int          delay_step = std::round(mpc_param_.delay_compensation_time / ctrl_period_);
    std::deque<double> tmp_deque(delay_step, 0.0);
    input_buffer_ = tmp_deque;

    /* initialize lowpass filter */
    double steering_lpf_cutoff_hz, error_deriv_lpf_curoff_hz;
    node->get_parameter("steering_lpf_cutoff_hz", steering_lpf_cutoff_hz);
    node->get_parameter("error_deriv_lpf_curoff_hz", error_deriv_lpf_curoff_hz);
    lpf_steering_cmd_.initialize(ctrl_period_, steering_lpf_cutoff_hz);
    lpf_lateral_error_.initialize(ctrl_period_, error_deriv_lpf_curoff_hz);
    lpf_yaw_error_.initialize(ctrl_period_, error_deriv_lpf_curoff_hz);

    /* set up ros system */
    std::string out_twist, out_vehicle_cmd, in_vehicle_status, in_waypoints, in_selfpose;
    node->get_parameter("out_twist_name", out_twist);
    node->get_parameter("out_vehicle_cmd_name", out_vehicle_cmd);
    node->get_parameter("in_waypoints_name", in_vehicle_status);
    node->get_parameter("in_selfpose_name", in_waypoints);
    node->get_parameter("in_vehicle_status_name", in_selfpose);
    twist_cmd_publisher_          = create_publisher<geometry_msgs::msg::TwistStamped>(out_twist, 1);
    steer_vel_ctrl_cmd_publisher_ = create_publisher<mpc_msgs::msg::ControlCommandStamped>(out_vehicle_cmd, 1);
    ref_path_subscriber_          = create_subscription<mpc_msgs::msg::Lane>(
        in_waypoints, rclcpp::SystemDefaultsQoS(), std::bind(MPCFollower::callbackRefPath, this, std::placeholders::_1));
    current_pose_subscriber_ = create_subscription<geometry_msgs::msg::Pose>(
        in_waypoints, rclcpp::SystemDefaultsQoS(), std::bind(MPCFollower::callbackPose, this, std::placeholders::_1));
    vehicle_status_subscriber_ = create_subscription<mpc_msgs::msg::VehicleStatus>(
        in_waypoints, rclcpp::SystemDefaultsQoS(), std::bind(MPCFollower::callbackVehicleStatus, this, std::placeholders::_1));

    control_timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(ctrl_period_ * 1000)),
                                       std::bind(&MPCFollower::timerCallback, this));


    /* for debug */
    debug_filtered_traj_publisher_  = create_publisher<visualization_msgs::msg::Marker>("debug/filtered_traj", 1);
    debug_predicted_traj_publisher_ = create_publisher<visualization_msgs::msg::Marker>("debug/predicted_traj", 1);
    debug_values_publisher_         = create_publisher<std_msgs::msg::Float64MultiArray>("debug/debug_values", 1);
    debug_mpc_calc_time_publisher_  = create_publisher<std_msgs::msg::Float32>("debug/mpc_calc_time", 1);
    estimate_twist_subscriber_      = create_subscription<mpc_msgs::msg::VehicleStatus>(
        "estimate_twist",
        rclcpp::SystemDefaultsQoS(),
        std::bind(MPCFollower::callbackEstimateTwist, this, std::placeholders::_1));
}

nav2_util::CallbackReturn MPCFollower::on_activate(const rclcpp_lifecycle::State& state)
{
    RCLCPP_INFO(get_logger(), "Activating");
}

nav2_util::CallbackReturn MPCFollower::on_deactivate(const rclcpp_lifecycle::State& state) {}

nav2_util::CallbackReturn MPCFollower::on_cleanup(const rclcpp_lifecycle::State& state) {}

nav2_util::CallbackReturn MPCFollower::on_shutdown(const rclcpp_lifecycle::State& state) {}

void MPCFollower::timerCallback() {}

void MPCFollower::callbackRefPath(const mpc_msgs::msg::Lane::ConstPtr&) {}

void MPCFollower::callbackPose(const geometry_msgs::msg::PoseStamped::ConstPtr&) {}

void MPCFollower::callbackVehicleStatus(const mpc_msgs::msg::VehicleStatus& msg) {}

void MPCFollower::publishControlCommands(const double& vel_cmd,
                                         const double& acc_cmd,
                                         const double& steer_cmd,
                                         const double& steer_vel_cmd)
{}

void MPCFollower::publishTwist(const double& vel_cmd, const double& omega_cmd) {}

void MPCFollower::publishCtrlCmd(const double& vel_cmd, const double& acc_cmd, const double& steer_cmd) {}

bool MPCFollower::calculateMPC(double& vel_cmd, double& acc_cmd, double& steer_cmd, double& steer_vel_cmd) {}

void MPCFollower::convertTrajToMarker(
    const MPCTrajectory& traj, visualization_msgs::msg::Marker& markers, std::string ns, double r, double g, double b, double z)
{}