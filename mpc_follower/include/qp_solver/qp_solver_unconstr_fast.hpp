/*
 * @Author       : dwayne
 * @Date         : 2023-06-21
 * @LastEditTime : 2023-06-21
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef MPC_LATERAL_CONTROLLER__QP_SOLVER__QP_SOLVER_UNCONSTR_FAST_HPP_
#define MPC_LATERAL_CONTROLLER__QP_SOLVER__QP_SOLVER_UNCONSTR_FAST_HPP_

#include "qp_solver/qp_solver_interface.hpp"


/**
 * Solver for QP problems using least square decomposition
 * implement with Eigen's standard Cholesky decomposition (LLT)
 */
class QPSolverEigenLeastSquareLLT : public QPSolverInterface
{
public:
    /**
   * @brief constructor
   */
    QPSolverEigenLeastSquareLLT();

    /**
   * @brief destructor
   */
    ~QPSolverEigenLeastSquareLLT() = default;

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
};
#endif   // MPC_LATERAL_CONTROLLER__QP_SOLVER__QP_SOLVER_UNCONSTR_FAST_HPP_
