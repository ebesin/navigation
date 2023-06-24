/*
 * @Author       : dwayne
 * @Date         : 2023-06-23
 * @LastEditTime : 2023-06-23
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#include "mpc_follower/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.hpp"

#include <cmath>

KinematicsBicycleModelNoDelay::KinematicsBicycleModelNoDelay(const double wheelbase, const double steer_lim)
    : VehicleModelInterface(/* dim_x */ 2, /* dim_u */ 1, /* dim_y */ 2, wheelbase)
{
    m_steer_lim = steer_lim;
}

void KinematicsBicycleModelNoDelay::calculateDiscreteMatrix(
    Eigen::MatrixXd& a_d, Eigen::MatrixXd& b_d, Eigen::MatrixXd& c_d, Eigen::MatrixXd& w_d, const double dt)
{
    auto sign = [](double x) { return (x > 0.0) - (x < 0.0); };

    /* Linearize delta around delta_r (reference delta) */
    double delta_r = atan(m_wheelbase * m_curvature);
    if (std::abs(delta_r) >= m_steer_lim) {
        delta_r = m_steer_lim * static_cast<double>(sign(delta_r));
    }
    double cos_delta_r_squared_inv = 1 / (cos(delta_r) * cos(delta_r));

    a_d << 0.0, m_velocity, 0.0, 0.0;

    b_d << 0.0, m_velocity / m_wheelbase * cos_delta_r_squared_inv;

    c_d << 1.0, 0.0, 0.0, 1.0;

    w_d << 0.0, -m_velocity / m_wheelbase * delta_r * cos_delta_r_squared_inv;

    // bilinear discretization for ZOH system
    // no discretization is needed for Cd
    Eigen::MatrixXd       I          = Eigen::MatrixXd::Identity(m_dim_x, m_dim_x);
    const Eigen::MatrixXd i_dt2a_inv = (I - dt * 0.5 * a_d).inverse();
    a_d                              = i_dt2a_inv * (I + dt * 0.5 * a_d);
    b_d                              = i_dt2a_inv * b_d * dt;
    w_d                              = i_dt2a_inv * w_d * dt;
}

void KinematicsBicycleModelNoDelay::calculateReferenceInput(Eigen::MatrixXd& u_ref)
{
    u_ref(0, 0) = std::atan(m_wheelbase * m_curvature);
}