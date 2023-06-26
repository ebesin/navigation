/*
 * @Author       : dwayne
 * @Date         : 2023-06-24
 * @LastEditTime : 2023-06-26
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */
#include "mpc_follower/mpc_controller_node.hpp"

MpcController::MpcController(const std::string& name, const rclcpp::NodeOptions& options)
    : nav2_util::LifecycleNode(name, "", options)
{
    RCLCPP_INFO(get_logger(), "Creating MpcController...");

    /*-- system --*/
    declare_parameter("traj_resample_dist", rclcpp::ParameterValue(0.1));
    declare_parameter("use_steer_prediction", rclcpp::ParameterValue(false));
    declare_parameter("admissible_position_error", rclcpp::ParameterValue(5.0));
    declare_parameter("admissible_yaw_error_rad", rclcpp::ParameterValue(1.57));

    /*-- path smmothing --*/
    declare_parameter("enable_path_smoothing", rclcpp::ParameterValue(false));
    declare_parameter("path_filter_moving_ave_num", rclcpp::ParameterValue(25));
    declare_parameter("curvature_smoothing_num_traj", rclcpp::ParameterValue(15));
    declare_parameter("curvature_smoothing_num_ref_steer", rclcpp::ParameterValue(15));

    /*-- trajectory extending --*/
    declare_parameter("extend_trajectory_for_end_yaw_control", rclcpp::ParameterValue(true));

    /*-- mpc --*/
    declare_parameter("qp_solver_type", rclcpp::ParameterValue("osqp"));
    declare_parameter("mpc_prediction_horizon", rclcpp::ParameterValue(50));
    declare_parameter("mpc_prediction_dt", rclcpp::ParameterValue(0.1));
    declare_parameter("mpc_weight_lat_error", rclcpp::ParameterValue(1.0));
    declare_parameter("mpc_weight_heading_error", rclcpp::ParameterValue(0.0));
    declare_parameter("mpc_weight_heading_error_squared_vel", rclcpp::ParameterValue(0.3));
    declare_parameter("mpc_weight_steering_input", rclcpp::ParameterValue(1.0));
    declare_parameter("mpc_weight_steering_input_squared_vel", rclcpp::ParameterValue(0.25));
    declare_parameter("mpc_weight_lat_jerk", rclcpp::ParameterValue(0.1));
    declare_parameter("mpc_weight_steer_rate", rclcpp::ParameterValue(0.0));
    declare_parameter("mpc_weight_steer_acc", rclcpp::ParameterValue(0.000001));
    declare_parameter("mpc_low_curvature_weight_lat_error", rclcpp::ParameterValue(0.1));
    declare_parameter("mpc_low_curvature_weight_heading_error", rclcpp::ParameterValue(0.0));
    declare_parameter("mpc_low_curvature_weight_heading_error_squared_vel", rclcpp::ParameterValue(0.3));
    declare_parameter("mpc_low_curvature_weight_steering_input", rclcpp::ParameterValue(1.0));
    declare_parameter("mpc_low_curvature_weight_steering_input_squared_vel", rclcpp::ParameterValue(0.25));
    declare_parameter("mpc_low_curvature_weight_lat_jerk", rclcpp::ParameterValue(0.0));
    declare_parameter("mpc_low_curvature_weight_steer_rate", rclcpp::ParameterValue(0.0));
    declare_parameter("mpc_low_curvature_weight_steer_acc", rclcpp::ParameterValue(0.000001));
    declare_parameter("mpc_low_curvature_thresh_curvature", rclcpp::ParameterValue(0.0));
    declare_parameter("mpc_weight_terminal_lat_error", rclcpp::ParameterValue(1.0));
    declare_parameter("mpc_weight_terminal_heading_error", rclcpp::ParameterValue(0.1));
    declare_parameter("mpc_zero_ff_steer_deg", rclcpp::ParameterValue(0.5));
    declare_parameter("mpc_acceleration_limit", rclcpp::ParameterValue(2.0));
    declare_parameter("mpc_velocity_time_constant", rclcpp::ParameterValue(0.3));
    declare_parameter("mpc_min_prediction_length", rclcpp::ParameterValue(5.0));

    /*-- vehicle model --*/
    declare_parameter("vehicle_model_type", rclcpp::ParameterValue("kinematics"));
    declare_parameter("input_delay", rclcpp::ParameterValue(0.24));
    declare_parameter("vehicle_model_steer_tau", rclcpp::ParameterValue(0.3));
    declare_parameter("steer_rate_lim_dps_list_by_curvature", rclcpp::ParameterValue(std::vector<double>{10.0, 20.0, 30.0}));
    declare_parameter("curvature_list_for_steer_rate_lim", rclcpp::ParameterValue(std::vector<double>{0.001, 0.002, 0.01}));
    declare_parameter("steer_rate_lim_dps_list_by_velocity", rclcpp::ParameterValue(std::vector<double>{40.0, 30.0, 20.0}));
    declare_parameter("velocity_list_for_steer_rate_lim", rclcpp::ParameterValue(std::vector<double>{10.0, 15.0, 20.0}));
    declare_parameter("acceleration_limit", rclcpp::ParameterValue(2.0));
    declare_parameter("velocity_time_constant", rclcpp::ParameterValue(0.3));

    /*-- lowpass filter for noise reduction --*/
    declare_parameter("steering_lpf_cutoff_hz", rclcpp::ParameterValue(3.0));
    declare_parameter("error_deriv_lpf_cutoff_hz", rclcpp::ParameterValue(5.0));

    /*-- stop state: steering command is kept in the previous value in the stop state. --*/
    declare_parameter("stop_state_entry_ego_speed", rclcpp::ParameterValue(0.001));
    declare_parameter("stop_state_entry_target_speed", rclcpp::ParameterValue(0.001));
    declare_parameter("converged_steer_rad", rclcpp::ParameterValue(0.1));
    declare_parameter("keep_steer_control_until_converged", rclcpp::ParameterValue(true));
    declare_parameter("new_traj_duration_time", rclcpp::ParameterValue(1.0));
    declare_parameter("new_traj_end_dist", rclcpp::ParameterValue(0.3));


    /*-- steer offset --*/
    declare_parameter("enable_auto_steering_offset_removal", rclcpp::ParameterValue(true));
    declare_parameter("update_vel_threshold", rclcpp::ParameterValue(5.56));
    declare_parameter("update_steer_threshold", rclcpp::ParameterValue(0.035));
    declare_parameter("average_num", rclcpp::ParameterValue(1000));
    declare_parameter("steering_offset_limit", rclcpp::ParameterValue(0.02));
}

MpcController::~MpcController() {}


nav2_util::CallbackReturn MpcController::on_configure(const rclcpp_lifecycle::State& state)
{
    auto node = shared_from_this();

    node->get_parameter("enable_path_smoothing", trajectory_filtering_param_.enable_path_smoothing);
    node->get_parameter("path_filter_moving_ave_num", trajectory_filtering_param_.path_filter_moving_ave_num);
    node->get_parameter("curvature_smoothing_num_traj", trajectory_filtering_param_.curvature_smoothing_num_traj);
    node->get_parameter("curvature_smoothing_num_ref_steer", trajectory_filtering_param_.curvature_smoothing_num_ref_steer);
    node->get_parameter("traj_resample_dist", trajectory_filtering_param_.traj_resample_dist);
    node->get_parameter("extend_trajectory_for_end_yaw_control",
                        trajectory_filtering_param_.extend_trajectory_for_end_yaw_control);

    node->get_parameter("admissible_position_error", mpc_.m_admissible_position_error);
    node->get_parameter("admissible_position_error", mpc_.m_admissible_position_error);
    node->get_parameter("use_steer_prediction", mpc_.m_use_steer_prediction);
    node->get_parameter("vehicle_model_steer_tau", mpc_.m_param.steer_tau);

    node->get_parameter("stop_state_entry_ego_speed", stop_state_entry_ego_speed_);
    node->get_parameter("stop_state_entry_target_speed", stop_state_entry_target_speed_);
    node->get_parameter("converged_steer_rad", converged_steer_rad_);
    node->get_parameter("keep_steer_control_until_converged", keep_steer_control_until_converged_);
    node->get_parameter("new_traj_duration_time", new_traj_duration_time_);
    node->get_parameter("new_traj_end_dist", new_traj_end_dist_);
    node->get_parameter("mpc_converged_threshold_rps", mpc_converged_threshold_rps_);


    const auto vehicle_info =
        vehicle_info_util::VehicleInfoUtil(node->get_node_parameters_interface(), std::string("vehicle_info")).getVehicleInfo();

    const double     wheelbase = vehicle_info.wheel_base_m;
    constexpr double deg2rad   = static_cast<double>(M_PI) / 180.0;
    mpc_.m_steer_lim           = vehicle_info.max_steer_angle_rad;

    // steer rate limit depending on curvature
    std::vector<double> steer_rate_lim_dps_list_by_curvature;
    std::vector<double> curvature_list_for_steer_rate_lim;
    node->get_parameter("steer_rate_lim_dps_list_by_curvature", steer_rate_lim_dps_list_by_curvature);
    node->get_parameter("steer_rate_lim_dps_list_by_curvature", curvature_list_for_steer_rate_lim);
    for (size_t i = 0; i < steer_rate_lim_dps_list_by_curvature.size(); ++i) {
        mpc_.m_steer_rate_lim_map_by_curvature.emplace_back(curvature_list_for_steer_rate_lim.at(i),
                                                            steer_rate_lim_dps_list_by_curvature.at(i) * deg2rad);
    }

    // steer rate limit depending on velocity
    std::vector<double> steer_rate_lim_dps_list_by_velocity;
    std::vector<double> velocity_list_for_steer_rate_lim;
    node->get_parameter("steer_rate_lim_dps_list_by_curvature", steer_rate_lim_dps_list_by_velocity);
    node->get_parameter("steer_rate_lim_dps_list_by_curvature", velocity_list_for_steer_rate_lim);
    for (size_t i = 0; i < steer_rate_lim_dps_list_by_velocity.size(); ++i) {
        mpc_.m_steer_rate_lim_map_by_velocity.emplace_back(velocity_list_for_steer_rate_lim.at(i),
                                                           steer_rate_lim_dps_list_by_velocity.at(i) * deg2rad);
    }

    /*vehicle model setup*/
    auto vehicle_model_ptr = createVehicleModel(wheelbase, mpc_.m_steer_lim, mpc_.m_param.steer_tau);
    mpc_.setVehicleModel(vehicle_model_ptr);

    auto qpsolver_ptr = createQPSolverInterface();
    mpc_.setQPSolver(qpsolver_ptr);

    /* delay compensation */
    {
        double delay_tmp;
        node->get_parameter("input_delay", delay_tmp);
        const double delay_step  = std::round(delay_tmp / mpc_.m_ctrl_period);
        mpc_.m_param.input_delay = delay_step * mpc_.m_ctrl_period;
        mpc_.m_input_buffer      = std::deque<double>(static_cast<size_t>(delay_step), 0.0);
    }

    /* steering offset compensation */
    node->get_parameter("steering_offset.enable_auto_steering_offset_removal", enable_auto_steering_offset_removal_);
    steering_offset_ = createSteerOffsetEstimator(wheelbase);


    /* initialize low-pass filter */
    {
        double steering_lpf_cutoff_hz;
        double error_deriv_lpf_cutoff_hz;
        node->get_parameter("steering_lpf_cutoff_hz", steering_lpf_cutoff_hz);
        node->get_parameter("error_deriv_lpf_cutoff_hz", error_deriv_lpf_cutoff_hz);
        mpc_.initializeLowPassFilters(steering_lpf_cutoff_hz, error_deriv_lpf_cutoff_hz);
    }


    // ego nearest index search
    node->get_parameter("ego_nearest_dist_threshold", ego_nearest_dist_threshold_);
    node->get_parameter("ego_nearest_yaw_threshold", ego_nearest_yaw_threshold_);
    mpc_.ego_nearest_dist_threshold = ego_nearest_dist_threshold_;
    mpc_.ego_nearest_yaw_threshold  = ego_nearest_yaw_threshold_;

    predicted_traj_publisher_ = node->create_publisher<Trajectory>("~/output/predicted_trajectory", 1);
    debug_values_publisher_   = node->create_publisher<Float32MultiArrayStamped>("~/output/lateral_diagnostic", 1);
    steer_offset_publisher_   = node->create_publisher<Float32Stamped>("~/output/estimated_steer_offset", 1);

    ref_path_subscriber_ = create_subscription<Trajectory>(
        "~/input/reference_trajectory", rclcpp::QoS{1}, std::bind(&MpcController::onTrajectory, this, std::placeholders::_1));
    steering_subscriber_ = create_subscription<mpc_msgs::msg::SteeringReport>(
        "~/input/current_steering", rclcpp::QoS{1}, std::bind(&MpcController::onSteering, this, std::placeholders::_1));
    odometry_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
        "~/input/current_odometry", rclcpp::QoS{1}, std::bind(&MpcController::onOdometry, this, std::placeholders::_1));
    accel_subscriber_ = create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
        "~/input/current_accel", rclcpp::QoS{1}, std::bind(&MpcController::onAccel, this, std::placeholders::_1));
    operation_mode_subscriber_ = create_subscription<mpc_msgs::msg::OperationModeState>(
        "~/input/current_operation_mode", rclcpp::QoS{1}, [this](const mpc_msgs::msg::OperationModeState::SharedPtr msg) {
            current_operation_mode_ptr_ = msg;
        });

    // control_cmd_publisher_ =
    //     create_publisher<mpc_msgs::msg::AckermannControlCommand>("~/output/control_cmd", rclcpp::QoS{1}.transient_local());
    debug_marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/output/debug_marker", rclcpp::QoS{1});

    declareMPCparameters();

    /* get parameter updates */
    // using std::placeholders::_1;
    // m_set_param_res = node_->add_on_set_parameters_callback(std::bind(&MpcLateralController::paramCallback, this, _1));

    mpc_.initializeSteeringPredictor();

    mpc_.setLogger(node->get_logger());
    mpc_.setClock(node->get_clock());

    /*Timer*/
    {
        double ctrl_period;
        node->get_parameter("ctrl_period", ctrl_period);
        const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(ctrl_period));
        timer_control_ =
            rclcpp::create_timer(this, get_clock(), period_ns, std::bind(&MpcController::callbackTimerControl, this));
    }
}


nav2_util::CallbackReturn MpcController::on_activate(const rclcpp_lifecycle::State& state) {}

nav2_util::CallbackReturn MpcController::on_deactivate(const rclcpp_lifecycle::State& state) {}

nav2_util::CallbackReturn MpcController::on_cleanup(const rclcpp_lifecycle::State& state) {}

nav2_util::CallbackReturn MpcController::on_shutdown(const rclcpp_lifecycle::State& state) {}

void MpcController::callbackTimerControl()
{
    // 1. create input data
    const auto input_data = createInputData(*get_clock());
    if (!input_data) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Control is skipped since input data is not ready.");
        return;
    }
    MpcController::LateralOutput lat_out = run(*input_data);
    // mpc_msgs::msg::AckermannLateralCommand out;
    // out.stamp               = this->now();
    // out.steering_tire_angle = lat_out.control_cmd.steering_tire_angle;
    control_cmd_publisher_->publish(lat_out.control_cmd);

    // 6. publish debug marker
    // publishDebugMarker(*input_data, lat_out);
}

MpcController::LateralOutput MpcController::run(MpcController::InputData const& input_data)
{
    mpc_.setReferenceTrajectory(input_data.current_trajectory, trajectory_filtering_param_);
    current_trajectory_      = input_data.current_trajectory;
    current_kinematic_state_ = input_data.current_odometry;
    current_steering_        = input_data.current_steering;

    AckermannLateralCommand  ctrl_cmd;
    Trajectory               predicted_traj;
    Float32MultiArrayStamped debug_values;
    if (!is_ctrl_cmd_prev_initialized_) {
        ctrl_cmd_prev_                = getInitialControlCommand();
        is_ctrl_cmd_prev_initialized_ = true;
    }

    const bool is_mpc_solved =
        mpc_.calculateMPC(current_steering_, current_kinematic_state_, ctrl_cmd, predicted_traj, debug_values);

    if (!is_mpc_solved) {
        mpc_.resetPrevResult(current_steering_);
    }
    else {
        setSteeringToHistory(ctrl_cmd);
    }
    publishPredictedTraj(predicted_traj);

    const auto createLateralOutput = [this](const auto& cmd, const bool is_mpc_solved) {
        trajectory_follower::LateralOutput output;
        output.control_cmd = createCtrlCmdMsg(cmd);
        // To be sure current steering of the vehicle is desired steering angle, we need to check
        // following conditions.
        // 1. At the last loop, mpc should be solved because command should be optimized output.
        // 2. The mpc should be converged.
        // 3. The steer angle should be converged.
        output.sync_data.is_steer_converged = is_mpc_solved && isMpcConverged() && isSteerConverged(cmd);

        return output;
    };
}


boost::optional<MpcController::InputData> MpcController::createInputData(rclcpp::Clock& clock)
{
    if (!current_trajectory_ptr_) {
        RCLCPP_INFO_THROTTLE(get_logger(), clock, 5000, "Waiting for trajectory.");
        return {};
    }

    if (!current_odometry_ptr_) {
        RCLCPP_INFO_THROTTLE(get_logger(), clock, 5000, "Waiting for current odometry.");
        return {};
    }

    if (!current_steering_ptr_) {
        RCLCPP_INFO_THROTTLE(get_logger(), clock, 5000, "Waiting for current steering.");
        return {};
    }

    if (!current_accel_ptr_) {
        RCLCPP_INFO_THROTTLE(get_logger(), clock, 5000, "Waiting for current accel.");
        return {};
    }

    if (!current_operation_mode_ptr_) {
        RCLCPP_INFO_THROTTLE(get_logger(), clock, 5000, "Waiting for current operation mode.");
        return {};
    }

    MpcController::InputData input_data;
    input_data.current_trajectory     = *current_trajectory_ptr_;
    input_data.current_odometry       = *current_odometry_ptr_;
    input_data.current_steering       = *current_steering_ptr_;
    input_data.current_accel          = *current_accel_ptr_;
    input_data.current_operation_mode = *current_operation_mode_ptr_;

    return input_data;
}

bool MpcController::isMpcConverged() {}

void MpcController::callbackTimerControl() {}

void MpcController::onTrajectory(const mpc_msgs::msg::Trajectory::SharedPtr msg)
{
    current_trajectory_ptr_ = msg;
}

void MpcController::onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_odometry_ptr_ = msg;
}

void MpcController::onSteering(const mpc_msgs::msg::SteeringReport::SharedPtr msg)
{
    current_steering_ptr_ = msg;
}

void MpcController::onAccel(const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg)
{
    current_accel_ptr_ = msg;
}

void MpcController::declareMPCparameters()
{
    auto node = shared_from_this();
    node->get_parameter("mpc_prediction_horizon", mpc_.m_param.prediction_horizon);
    node->get_parameter("mpc_prediction_dt", mpc_.m_param.prediction_dt);

    node->get_parameter("mpc_weight_lat_error", mpc_.m_param.nominal_weight.lat_error);
    node->get_parameter("mpc_weight_heading_error", mpc_.m_param.nominal_weight.heading_error);
    node->get_parameter("mpc_weight_heading_error_squared_vel", mpc_.m_param.nominal_weight.heading_error_squared_vel);
    node->get_parameter("mpc_weight_steering_input", mpc_.m_param.nominal_weight.steering_input);
    node->get_parameter("mpc_weight_steering_input_squared_vel", mpc_.m_param.nominal_weight.steering_input_squared_vel);
    node->get_parameter("mpc_weight_lat_jerk", mpc_.m_param.nominal_weight.lat_jerk);
    node->get_parameter("mpc_weight_steer_rate", mpc_.m_param.nominal_weight.steer_rate);
    node->get_parameter("mpc_weight_steer_acc", mpc_.m_param.nominal_weight.steer_acc);
    node->get_parameter("mpc_weight_terminal_lat_error", mpc_.m_param.nominal_weight.terminal_lat_error);
    node->get_parameter("mpc_weight_terminal_heading_error", mpc_.m_param.nominal_weight.terminal_heading_error);

    node->get_parameter("mpc_low_curvature_lat_error", mpc_.m_param.low_curvature_weight.lat_error);
    node->get_parameter("mpc_low_curvature_heading_error", mpc_.m_param.low_curvature_weight.heading_error);
    node->get_parameter("mpc_low_curvature_heading_error_squared_vel",
                        mpc_.m_param.low_curvature_weight.heading_error_squared_vel);
    node->get_parameter("mpc_low_curvature_steering_input", mpc_.m_param.low_curvature_weight.steering_input);
    node->get_parameter("mpc_low_curvature_steering_input_squared_vel",
                        mpc_.m_param.low_curvature_weight.steering_input_squared_vel);
    node->get_parameter("mpc_low_curvature_lat_jerk", mpc_.m_param.low_curvature_weight.lat_jerk);
    node->get_parameter("mpc_low_curvature_steer_rate", mpc_.m_param.low_curvature_weight.steer_rate);
    node->get_parameter("mpc_low_curvature_steer_acc", mpc_.m_param.low_curvature_weight.steer_acc);
    node->get_parameter("mpc_low_curvature_thresh_curvature", mpc_.m_param.low_curvature_thresh_curvature);

    node->get_parameter("mpc_zero_ff_steer_deg", mpc_.m_param.zero_ff_steer_deg);
    node->get_parameter("mpc_acceleration_limit", mpc_.m_param.acceleration_limit);
    node->get_parameter("mpc_velocity_time_constant", mpc_.m_param.velocity_time_constant);
    node->get_parameter("mpc_min_prediction_length", mpc_.m_param.min_prediction_length);
}

std::shared_ptr<VehicleModelInterface> MpcController::createVehicleModel(const double wheelbase,
                                                                         const double steer_lim,
                                                                         const double steer_tau)
{
    std::shared_ptr<VehicleModelInterface> vehicle_model_ptr;
    auto                                   node = shared_from_this();
    std::string                            vehicle_model_type;
    node->get_parameter("vehicle_model_type", vehicle_model_type);
    if (vehicle_model_type == "kinematics_no_delay") {
        vehicle_model_ptr = std::make_shared<KinematicsBicycleModelNoDelay>(wheelbase, steer_lim);
        return vehicle_model_ptr;
    }
    return vehicle_model_ptr;
}


AckermannLateralCommand MpcController::getInitialControlCommand() const
{
    AckermannLateralCommand cmd;
    cmd.steering_tire_angle         = current_steering_.steering_tire_angle;
    cmd.steering_tire_rotation_rate = 0.0;
    return cmd;
}
