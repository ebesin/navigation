/*
 * @Author       : dwayne
 * @Date         : 2023-06-21
 * @LastEditTime : 2023-06-21
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef MPC_LATERAL_CONTROLLER__QP_SOLVER__QP_SOLVER_OSQP_HPP_
#define MPC_LATERAL_CONTROLLER__QP_SOLVER__QP_SOLVER_OSQP_HPP_

#include "osqp_interface/osqp_interface.hpp"
#include "qp_solver/qp_solver_interface.hpp"
#include "rclcpp/rclcpp.hpp"


/// Solver for QP problems using the OSQP library
class QPSolverOSQP : public QPSolverInterface
{
public:
    /**
   * @brief constructor
   */
    explicit QPSolverOSQP(const rclcpp::Logger& logger);

    /**
   * @brief destructor
   */
    virtual ~QPSolverOSQP() = default;

    /**
   * @brief solve QP problem : minimize j = u' * h_mat * u + f_vec' * u without constraint
   * @param [in] h_mat parameter matrix in object function
   * @param [in] f_vec parameter matrix in object function
   * @param [in] a parameter matrix for constraint lb_a < a*u < ub_a (not used here)
   * @param [in] lb parameter matrix for constraint lb < U < ub (not used here)
   * @param [in] ub parameter matrix for constraint lb < U < ub (not used here)
   * @param [in] lb_a parameter matrix for constraint lb_a < a*u < ub_a (not used here)
   * @param [in] ub_a parameter matrix for constraint lb_a < a*u < ub_a (not used here)
   * @param [out] u optimal variable vector
   * @return true if the problem was solved
   */
    bool solve(const Eigen::MatrixXd& h_mat,
               const Eigen::MatrixXd& f_vec,
               const Eigen::MatrixXd& a,
               const Eigen::VectorXd& lb,
               const Eigen::VectorXd& ub,
               const Eigen::VectorXd& lb_a,
               const Eigen::VectorXd& ub_a,
               Eigen::VectorXd&       u) override;

private:
    osqp::OSQPInterface osqpsolver_;
    rclcpp::Logger      logger_;
};
#endif   // MPC_LATERAL_CONTROLLER__QP_SOLVER__QP_SOLVER_OSQP_HPP_
