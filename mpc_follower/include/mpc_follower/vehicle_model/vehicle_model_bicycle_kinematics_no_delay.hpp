/*
 * @Author       : dwayne
 * @Date         : 2023-06-17
 * @LastEditTime : 2023-06-21
 * @Description  : vehicle model class of bicycle kinematics without steering delay
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */


/*
 *    Representation
 * e      : lateral error
 * th     : heading angle error
 * steer  : steering angle (input)
 * v      : velocity
 * W      : wheelbase length
 * tau    : time constant for steering dynamics
 *
 *    State & Input
 * x = [e, th]^T
 * u = steer
 *
 *    Nonlinear model
 * dx1/dt = v * sin(x2)
 * dx2/dt = v * tan(u) / W
 *
 *    Linearized model around reference point (v = v_r, th = th_r, steer = steer_r)
 *  dx/dt = [0, vr] * x + [                  0] * u + [                           0]
 *          [0,  0]       [vr/W/cos(steer_r)^2]       [-vr*steer_r/W/cos(steer_r)^2]
 *
 */

#pragma once
#include "mpc_follower/vehicle_model/vehicle_model_interface.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

/**
 * Vehicle model class of bicycle kinematics without steering delay
 * @brief calculate model-related values
 */
class KinematicsBicycleModelNoDelay : public VehicleModelInterface
{
public:
    /**
   * @brief constructor with parameter initialization
   * @param [in] wheelbase wheelbase length [m]
   * @param [in] steer_lim steering angle limit [rad]
   */
    KinematicsBicycleModelNoDelay(const double wheelbase, const double steer_lim);

    /**
   * @brief destructor
   */
    ~KinematicsBicycleModelNoDelay() = default;

    /**
   * @brief calculate discrete model matrix of x_k+1 = a_d * xk + b_d * uk + w_d, yk = c_d * xk
   * @param [out] a_d coefficient matrix
   * @param [out] b_d coefficient matrix
   * @param [out] c_d coefficient matrix
   * @param [out] w_d coefficient matrix
   * @param [in] dt Discretization time [s]
   */
    void calculateDiscreteMatrix(
        Eigen::MatrixXd& a_d, Eigen::MatrixXd& b_d, Eigen::MatrixXd& c_d, Eigen::MatrixXd& w_d, const double dt) override;

    /**
   * @brief calculate reference input
   * @param [out] u_ref input
   */
    void calculateReferenceInput(Eigen::MatrixXd& u_ref) override;

    std::string modelName() override { return "kinematics_no_delay"; };

private:
    double m_steer_lim;   //!< @brief steering angle limit [rad]
};
