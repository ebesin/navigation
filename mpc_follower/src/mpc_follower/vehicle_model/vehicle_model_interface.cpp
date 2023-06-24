/*
 * @Author       : dwayne
 * @Date         : 2023-06-23
 * @LastEditTime : 2023-06-23
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#include "mpc_follower/vehicle_model/vehicle_model_interface.hpp"


VehicleModelInterface::VehicleModelInterface(int dim_x, int dim_u, int dim_y, double wheelbase)
    : m_dim_x(dim_x)
    , m_dim_u(dim_u)
    , m_dim_y(dim_y)
    , m_wheelbase(wheelbase)
{}
int VehicleModelInterface::getDimX()
{
    return m_dim_x;
}
int VehicleModelInterface::getDimU()
{
    return m_dim_u;
}
int VehicleModelInterface::getDimY()
{
    return m_dim_y;
}
double VehicleModelInterface::getWheelbase()
{
    return m_wheelbase;
}
void VehicleModelInterface::setVelocity(const double velocity)
{
    m_velocity = velocity;
}
void VehicleModelInterface::setCurvature(const double curvature)
{
    m_curvature = curvature;
}