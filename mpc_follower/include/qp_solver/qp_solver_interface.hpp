/*
 * @Author       : dwayne
 * @Date         : 2023-06-21
 * @LastEditTime : 2023-06-21
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef MPC_LATERAL_CONTROLLER__QP_SOLVER__QP_SOLVER_INTERFACE_HPP_
#define MPC_LATERAL_CONTROLLER__QP_SOLVER__QP_SOLVER_INTERFACE_HPP_

#include <Eigen/Core>

/// Interface for solvers of Quadratic Programming (QP) problems
class QPSolverInterface
{
public:
    /**
   * @brief destructor
   */
    virtual ~QPSolverInterface() = default;

    /**
   * @brief solve QP problem : minimize J = u' * h_mat * u + f_vec' * u without constraint
   * @param [in] h_mat parameter matrix in object function
   * @param [in] f_vec parameter matrix in object function
   * @param [in] a parameter matrix for constraint lb_a < a*u < ub_a
   * @param [in] lb parameter matrix for constraint lb < u < ub
   * @param [in] ub parameter matrix for constraint lb < u < ub
   * @param [in] lb_a parameter matrix for constraint lb_a < a*u < ub_a
   * @param [in] ub_a parameter matrix for constraint lb_a < a*u < ub_a
   * @param [out] u optimal variable vector
   * @return true if the problem was solved
   */
    virtual bool solve(const Eigen::MatrixXd& h_mat,
                       const Eigen::MatrixXd& f_vec,
                       const Eigen::MatrixXd& a,
                       const Eigen::VectorXd& lb,
                       const Eigen::VectorXd& ub,
                       const Eigen::VectorXd& lb_a,
                       const Eigen::VectorXd& ub_a,
                       Eigen::VectorXd&       u) = 0;
};
#endif   // MPC_LATERAL_CONTROLLER__QP_SOLVER__QP_SOLVER_INTERFACE_HPP_
