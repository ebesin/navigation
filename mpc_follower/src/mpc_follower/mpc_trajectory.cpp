/*
 * @Author       : dwayne
 * @Date         : 2023-06-23
 * @LastEditTime : 2023-06-23
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#include "mpc_follower/mpc_trajectory.hpp"

void MPCTrajectory::push_back(const double& xp,
                              const double& yp,
                              const double& zp,
                              const double& yawp,
                              const double& vxp,
                              const double& kp,
                              const double& smooth_kp,
                              const double& tp)
{
    x.push_back(xp);
    y.push_back(yp);
    z.push_back(zp);
    yaw.push_back(yawp);
    vx.push_back(vxp);
    k.push_back(kp);
    smooth_k.push_back(smooth_kp);
    relative_time.push_back(tp);
}

void MPCTrajectory::push_back(const MPCTrajectoryPoint& p)
{
    x.push_back(p.x);
    y.push_back(p.y);
    z.push_back(p.z);
    yaw.push_back(p.yaw);
    vx.push_back(p.vx);
    k.push_back(p.k);
    smooth_k.push_back(p.smooth_k);
    relative_time.push_back(p.relative_time);
}

MPCTrajectoryPoint MPCTrajectory::back()
{
    MPCTrajectoryPoint p;

    p.x             = x.back();
    p.y             = y.back();
    p.z             = z.back();
    p.yaw           = yaw.back();
    p.vx            = vx.back();
    p.k             = k.back();
    p.smooth_k      = smooth_k.back();
    p.relative_time = relative_time.back();

    return p;
}

void MPCTrajectory::clear()
{
    x.clear();
    y.clear();
    z.clear();
    yaw.clear();
    vx.clear();
    k.clear();
    smooth_k.clear();
    relative_time.clear();
}

size_t MPCTrajectory::size() const
{
    if (x.size() == y.size() && x.size() == z.size() && x.size() == yaw.size() && x.size() == vx.size() && x.size() == k.size() &&
        x.size() == smooth_k.size() && x.size() == relative_time.size()) {
        return x.size();
    }
    else {
        std::cerr << "[MPC trajectory] trajectory size is inappropriate" << std::endl;
        return 0;
    }
}
