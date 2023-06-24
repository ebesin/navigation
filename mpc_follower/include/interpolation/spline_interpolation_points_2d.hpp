/*
 * @Author       : dwayne
 * @Date         : 2023-06-23
 * @LastEditTime : 2023-06-24
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef INTERPOLATION__SPLINE_INTERPOLATION_POINTS_2D_HPP_
#define INTERPOLATION__SPLINE_INTERPOLATION_POINTS_2D_HPP_

#include "interpolation/spline_interpolation.hpp"

#include <vector>

namespace interpolation {
std::array<std::vector<double>, 3> slerp2dFromXY(const std::vector<double>& base_keys,
                                                 const std::vector<double>& base_x_values,
                                                 const std::vector<double>& base_y_values,
                                                 const std::vector<double>& query_keys);

template<typename T> std::vector<double> splineYawFromPoints(const std::vector<T>& points);
}   // namespace interpolation

// non-static points spline interpolation
// NOTE: We can calculate yaw from the x and y by interpolation derivatives.
//
// Usage:
// ```
// SplineInterpolationPoints2d spline;
// // memorize pre-interpolation result internally
// spline.calcSplineCoefficients(base_keys, base_values);
// const auto interpolation_result1 = spline.getSplineInterpolatedPoint(
//   base_keys, query_keys1);
// const auto interpolation_result2 = spline.getSplineInterpolatedPoint(
//   base_keys, query_keys2);
// const auto yaw_interpolation_result = spline.getSplineInterpolatedYaw(
//   base_keys, query_keys1);
// ```
class SplineInterpolationPoints2d
{
public:
    SplineInterpolationPoints2d() = default;
    template<typename T> explicit SplineInterpolationPoints2d(const std::vector<T>& points)
    {
        std::vector<geometry_msgs::msg::Point> points_inner;
        for (const auto& p : points) {
            points_inner.push_back(mpc_utils::getPoint(p));
        }
        calcSplineCoefficientsInner(points_inner);
    }

    // TODO(murooka) implement these functions
    // std::vector<geometry_msgs::msg::Point> getSplineInterpolatedPoints(const double width);
    // std::vector<geometry_msgs::msg::Pose> getSplineInterpolatedPoses(const double width);

    // pose (= getSplineInterpolatedPoint + getSplineInterpolatedYaw)
    geometry_msgs::msg::Pose getSplineInterpolatedPose(const size_t idx, const double s) const;

    // point
    geometry_msgs::msg::Point getSplineInterpolatedPoint(const size_t idx, const double s) const;

    // yaw
    double              getSplineInterpolatedYaw(const size_t idx, const double s) const;
    std::vector<double> getSplineInterpolatedYaws() const;

    // curvature
    double              getSplineInterpolatedCurvature(const size_t idx, const double s) const;
    std::vector<double> getSplineInterpolatedCurvatures() const;

    size_t getSize() const { return base_s_vec_.size(); }
    size_t getOffsetIndex(const size_t idx, const double offset) const;
    double getAccumulatedLength(const size_t idx) const;

private:
    void                calcSplineCoefficientsInner(const std::vector<geometry_msgs::msg::Point>& points);
    SplineInterpolation spline_x_;
    SplineInterpolation spline_y_;
    SplineInterpolation spline_z_;

    std::vector<double> base_s_vec_;
};

#endif   // INTERPOLATION__SPLINE_INTERPOLATION_POINTS_2D_HPP_
