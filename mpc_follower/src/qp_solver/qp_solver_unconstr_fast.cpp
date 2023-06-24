/*
 * @Author       : dwayne
 * @Date         : 2023-06-23
 * @LastEditTime : 2023-06-23
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#include "qp_solver/qp_solver_unconstr_fast.hpp"

#include <Eigen/Dense>


QPSolverEigenLeastSquareLLT::QPSolverEigenLeastSquareLLT() {}
bool QPSolverEigenLeastSquareLLT::solve(const Eigen::MatrixXd& h_mat,
                                        const Eigen::MatrixXd& f_vec,
                                        const Eigen::MatrixXd& /*a*/,
                                        const Eigen::VectorXd& /*lb*/,
                                        const Eigen::VectorXd& /*ub*/,
                                        const Eigen::VectorXd& /*lb_a*/,
                                        const Eigen::VectorXd& /*ub_a*/,
                                        Eigen::VectorXd& u)
{
    if (std::fabs(h_mat.determinant()) < 1.0E-9) {
        return false;
    }

    u = -h_mat.llt().solve(f_vec);

    return true;
}
