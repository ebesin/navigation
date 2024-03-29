/*
 * @Author       : dwayne
 * @Date         : 2023-06-21
 * @LastEditTime : 2023-06-28
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#pragma once

#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/register/point.hpp>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/point.hpp>
namespace mpc_utils {
// 2D
struct Point2d;
using Segment2d         = boost::geometry::model::segment<Point2d>;
using Box2d             = boost::geometry::model::box<Point2d>;
using LineString2d      = boost::geometry::model::linestring<Point2d>;
using LinearRing2d      = boost::geometry::model::ring<Point2d>;
using Polygon2d         = boost::geometry::model::polygon<Point2d>;
using MultiPoint2d      = boost::geometry::model::multi_point<Point2d>;
using MultiLineString2d = boost::geometry::model::multi_linestring<LineString2d>;
using MultiPolygon2d    = boost::geometry::model::multi_polygon<Polygon2d>;

// 3D
struct Point3d;
using Segment3d         = boost::geometry::model::segment<Point3d>;
using Box3d             = boost::geometry::model::box<Point3d>;
using LineString3d      = boost::geometry::model::linestring<Point3d>;
using LinearRing3d      = boost::geometry::model::ring<Point3d>;
using Polygon3d         = boost::geometry::model::polygon<Point3d>;
using MultiPoint3d      = boost::geometry::model::multi_point<Point3d>;
using MultiLineString3d = boost::geometry::model::multi_linestring<LineString3d>;
using MultiPolygon3d    = boost::geometry::model::multi_polygon<Polygon3d>;

struct Point2d : public Eigen::Vector2d
{
    Point2d() = default;
    Point2d(const double x, const double y)
        : Eigen::Vector2d(x, y)
    {}

    [[nodiscard]] Point3d to_3d(const double z = 0.0) const;
};

struct Point3d : public Eigen::Vector3d
{
    Point3d() = default;
    Point3d(const double x, const double y, const double z)
        : Eigen::Vector3d(x, y, z)
    {}

    [[nodiscard]] Point2d to_2d() const;
};

inline Point3d Point2d::to_3d(const double z) const
{
    return Point3d{x(), y(), z};
}

inline Point2d Point3d::to_2d() const
{
    return Point2d{x(), y()};
}

inline geometry_msgs::msg::Point toMsg(const Point3d& point)
{
    geometry_msgs::msg::Point msg;
    msg.x = point.x();
    msg.y = point.y();
    msg.z = point.z();
    return msg;
}

inline Point3d fromMsg(const geometry_msgs::msg::Point& msg)
{
    Point3d point;
    point.x() = msg.x;
    point.y() = msg.y;
    point.z() = msg.z;
    return point;
}
}   // namespace mpc_utils
BOOST_GEOMETRY_REGISTER_POINT_2D(   // NOLINT
    mpc_utils::Point2d,
    double,
    cs::cartesian,
    x(),
    y())                            // NOLINT
BOOST_GEOMETRY_REGISTER_POINT_3D(   // NOLINT
    mpc_utils::Point3d,
    double,
    cs::cartesian,
    x(),
    y(),
    z())   // NOLINT