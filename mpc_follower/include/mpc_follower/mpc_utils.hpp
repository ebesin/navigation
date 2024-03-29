/*
 * @Author       : dwayne
 * @Date         : 2023-06-19
 * @LastEditTime : 2023-06-23
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef MPC_LATERAL_CONTROLLER__MPC_UTILS_HPP_
#define MPC_LATERAL_CONTROLLER__MPC_UTILS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"

#include <Eigen/Core>

#ifdef ROS_DISTRO_GALACTIC
#    include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#else
#    include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#endif

#include "mpc_trajectory.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mpc_msgs/msg/trajectory.hpp"
#include "mpc_msgs/msg/trajectory_point.hpp"

#include <cmath>
#include <string>
#include <utility>
#include <vector>


namespace MPCUtils {

using mpc_msgs::msg::Trajectory;
using mpc_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;

/**
 * @brief calculate 2d distance from trajectory[idx1] to trajectory[idx2]
 */
double calcDistance2d(const MPCTrajectory& trajectory, const size_t idx1, const size_t idx2);

/**
 * @brief calculate 3d distance from trajectory[idx1] to trajectory[idx2]
 */
double calcDistance3d(const MPCTrajectory& trajectory, const size_t idx1, const size_t idx2);

/**
 * @brief convert Euler angle vector including +-2pi to 0 jump to continuous series data
 * @param [inout] angle_vector input angle vector
 */
void convertEulerAngleToMonotonic(std::vector<double>& angle_vector);

/**
 * @brief calculate the lateral error of the given pose relative to the given reference pose
 * @param [in] ego_pose pose to check for error
 * @param [in] ref_pose reference pose
 * @return lateral distance between the two poses
 */
double calcLateralError(const Pose& ego_pose, const Pose& ref_pose);

/**
 * @brief convert the given Trajectory msg to a MPCTrajectory object
 * @param [in] input trajectory to convert
 * @return resulting MPCTrajectory
 */
MPCTrajectory convertToMPCTrajectory(const Trajectory& input);

/**
 * @brief convert the given MPCTrajectory to a Trajectory msg
 * @param [in] input MPCTrajectory to be converted
 * @return output converted Trajectory msg
 */
Trajectory convertToAutowareTrajectory(const MPCTrajectory& input);

/**
 * @brief calculate the arc length at each point of the given trajectory
 * @param [in] trajectory trajectory for which to calculate the arc length
 * @param [out] arc_length the cumulative arc length at each point of the trajectory
 */
void calcMPCTrajectoryArcLength(const MPCTrajectory& trajectory, std::vector<double>& arc_length);

/**
 * @brief resample the given trajectory with the given fixed interval
 * @param [in] input trajectory to resample
 * @param [in] resample_interval_dist the desired distance between two successive trajectory points
 * @return The pair contains the successful flag and the resultant resampled trajectory
 */
std::pair<bool, MPCTrajectory> resampleMPCTrajectoryByDistance(const MPCTrajectory& input, const double resample_interval_dist);

/**
 * @brief linearly interpolate the given trajectory assuming a base indexing and a new desired
 * indexing
 * @param [in] in_index indexes for each trajectory point
 * @param [in] in_traj MPCTrajectory to interpolate
 * @param [in] out_index desired interpolated indexes
 * @param [out] out_traj resulting interpolated MPCTrajectory
 */
bool linearInterpMPCTrajectory(const std::vector<double>& in_index,
                               const MPCTrajectory&       in_traj,
                               const std::vector<double>& out_index,
                               MPCTrajectory&             out_traj);

/**
 * @brief fill the relative_time field of the given MPCTrajectory
 * @param [in] traj MPCTrajectory for which to fill in the relative_time
 * @return true if the calculation was successful
 */
bool calcMPCTrajectoryTime(MPCTrajectory& traj);

/**
 * @brief recalculate the velocity field (vx) of the MPCTrajectory with dynamic smoothing
 * @param [in] start_idx index of the trajectory point from which to start smoothing
 * @param [in] start_vel initial velocity to set at the start_idx
 * @param [in] acc_lim limit on the acceleration
 * @param [in] tau constant to control the smoothing (high-value = very smooth)
 * @param [inout] traj MPCTrajectory for which to calculate the smoothed velocity
 */
void dynamicSmoothingVelocity(
    const size_t start_idx, const double start_vel, const double acc_lim, const double tau, MPCTrajectory& traj);

/**
 * @brief calculate yaw angle in MPCTrajectory from xy vector
 * @param [inout] traj object trajectory
 * @param [in] shift is forward or not
 */
void calcTrajectoryYawFromXY(MPCTrajectory& traj, const bool is_forward_shift);

/**
 * @brief Calculate path curvature by 3-points circle fitting with smoothing num (use nearest 3
 * points when num = 1)
 * @param [in] curvature_smoothing_num_traj index distance for 3 points for curvature calculation
 * @param [in] curvature_smoothing_num_ref_steer index distance for 3 points for smoothed curvature
 * calculation
 * @param [inout] traj object trajectory
 */
void calcTrajectoryCurvature(const int      curvature_smoothing_num_traj,
                             const int      curvature_smoothing_num_ref_steer,
                             MPCTrajectory& traj);

/**
 * @brief Calculate path curvature by 3-points circle fitting with smoothing num (use nearest 3
 * points when num = 1)
 * @param [in] curvature_smoothing_num index distance for 3 points for curvature calculation
 * @param [in] traj input trajectory
 * @return vector of curvatures at each point of the given trajectory
 */
std::vector<double> calcTrajectoryCurvature(const int curvature_smoothing_num, const MPCTrajectory& traj);

/**
 * @brief calculate nearest pose on MPCTrajectory with linear interpolation
 * @param [in] traj reference trajectory
 * @param [in] self_pose object pose
 * @param [out] nearest_pose nearest pose on path
 * @param [out] nearest_index path index of nearest pose
 * @param [out] nearest_time time of nearest pose on trajectory
 * @return false when nearest pose couldn't find for some reasons
 */
bool calcNearestPoseInterp(const MPCTrajectory& traj,
                           const Pose&          self_pose,
                           Pose*                nearest_pose,
                           size_t*              nearest_index,
                           double*              nearest_time,
                           double*              ref_vx,
                           const double         max_dist,
                           const double         max_yaw);

/**
 * @brief calculate distance to stopped point
 */
double calcStopDistance(const Trajectory& current_trajectory, const int origin);

/**
 * @brief extend terminal points
 * Note: The current MPC does not properly take into account the attitude angle at the end of the
 * path. By extending the end of the path in the attitude direction, the MPC can consider the
 * attitude angle well, resulting in improved control performance. If the trajectory is
 * well-defined considering the end point attitude angle, this feature is not necessary.
 * @param [in] terminal yaw
 * @param [in] extend interval
 * @param [in] flag of forward shift
 * @param [out] extended trajectory
 */
void extendTrajectoryInYawDirection(const double yaw, const double interval, const bool is_forward_shift, MPCTrajectory& traj);

/**
 * @brief Updates the value of a parameter with the given name.
 * @tparam T The type of the parameter value.
 * @param parameters A vector of rclcpp::Parameter objects.
 * @param name The name of the parameter to update.
 * @param value A reference variable to store the updated parameter value.
 */
template<typename T> void update_param(const std::vector<rclcpp::Parameter>& parameters, const std::string& name, T& value)
{
    auto it = std::find_if(parameters.cbegin(), parameters.cend(), [&name](const rclcpp::Parameter& parameter) {
        return parameter.get_name() == name;
    });
    if (it != parameters.cend()) {
        value = static_cast<T>(it->template get_value<T>());
    }
}

}   // namespace MPCUtils
#endif   // MPC_LATERAL_CONTROLLER__MPC_UTILS_HPP_