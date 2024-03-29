/*
 * @Author       : dwayne
 * @Date         : 2023-06-23
 * @LastEditTime : 2023-06-24
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef MOTION_UTILS__TRAJECTORY__PATH_WITH_LANE_ID_HPP_
#define MOTION_UTILS__TRAJECTORY__PATH_WITH_LANE_ID_HPP_

#include "motion_utils/trajectory/trajectory.hpp"
#include "mpc_utils/geometry/path_with_lane_id_geometry.hpp"

#include <boost/optional.hpp>

#include <algorithm>
#include <utility>
#include <vector>

namespace motion_utils {
inline boost::optional<std::pair<size_t, size_t>> getPathIndexRangeWithLaneId(const mpc_msgs::msg::PathWithLaneId& path,
                                                                              const int64_t                        target_lane_id)
{
    size_t start_idx = 0;   // NOTE: to prevent from maybe-uninitialized error
    size_t end_idx   = 0;   // NOTE: to prevent from maybe-uninitialized error

    bool found_first_idx = false;
    for (size_t i = 0; i < path.points.size(); ++i) {
        const auto& p = path.points.at(i);
        for (const auto& id : p.lane_ids) {
            if (id == target_lane_id) {
                if (!found_first_idx) {
                    start_idx       = i;
                    found_first_idx = true;
                }
                end_idx = i;
            }
        }
    }

    if (found_first_idx) {
        // NOTE: In order to fully cover lanes with target_lane_id, start_idx and end_idx are expanded.
        start_idx = static_cast<size_t>(std::max(0, static_cast<int>(start_idx) - 1));
        end_idx   = std::min(path.points.size() - 1, end_idx + 1);

        return std::make_pair(start_idx, end_idx);
    }

    return {};
}

inline size_t findNearestIndexFromLaneId(const mpc_msgs::msg::PathWithLaneId& path,
                                         const geometry_msgs::msg::Point&     pos,
                                         const int64_t                        lane_id)
{
    const auto opt_range = getPathIndexRangeWithLaneId(path, lane_id);
    if (opt_range) {
        const size_t start_idx = opt_range->first;
        const size_t end_idx   = opt_range->second;

        validateNonEmpty(path.points);

        const auto sub_points =
            std::vector<mpc_msgs::msg::PathPointWithLaneId>{path.points.begin() + start_idx, path.points.begin() + end_idx + 1};
        validateNonEmpty(sub_points);

        return start_idx + findNearestIndex(sub_points, pos);
    }

    return findNearestIndex(path.points, pos);
}

inline size_t findNearestSegmentIndexFromLaneId(const mpc_msgs::msg::PathWithLaneId& path,
                                                const geometry_msgs::msg::Point&     pos,
                                                const int64_t                        lane_id)
{
    const size_t nearest_idx = findNearestIndexFromLaneId(path, pos, lane_id);

    if (nearest_idx == 0) {
        return 0;
    }
    if (nearest_idx == path.points.size() - 1) {
        return path.points.size() - 2;
    }

    const double signed_length = calcLongitudinalOffsetToSegment(path.points, nearest_idx, pos);

    if (signed_length <= 0) {
        return nearest_idx - 1;
    }

    return nearest_idx;
}

// @brief Calculates the path to be followed by the rear wheel center in order to make the vehicle
// center follow the input path
// @param [in] path with position to be followed by the center of the vehicle
// @param [out] PathWithLaneId to be followed by the rear wheel center follow to make the vehicle
// center follow the input path NOTE: rear_to_cog is supposed to be positive
mpc_msgs::msg::PathWithLaneId convertToRearWheelCenter(const mpc_msgs::msg::PathWithLaneId& path,
                                                       const double                         rear_to_cog,
                                                       const bool enable_last_point_compensation = true);
}   // namespace motion_utils

#endif   // MOTION_UTILS__TRAJECTORY__PATH_WITH_LANE_ID_HPP_
