/*
 * @Author       : dwayne
 * @Date         : 2023-06-23
 * @LastEditTime : 2023-06-23
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#include "qp_solver/qp_solver_osqp.hpp"

#include <string>
#include <vector>


QPSolverOSQP::QPSolverOSQP(const rclcpp::Logger& logger)
    : logger_{logger}
{}
bool QPSolverOSQP::solve(const Eigen::MatrixXd& h_mat,
                         const Eigen::MatrixXd& f_vec,
                         const Eigen::MatrixXd& a,
                         const Eigen::VectorXd& lb,
                         const Eigen::VectorXd& ub,
                         const Eigen::VectorXd& lb_a,
                         const Eigen::VectorXd& ub_a,
                         Eigen::VectorXd&       u)
{
    const Eigen::Index raw_a    = a.rows();
    const Eigen::Index col_a    = a.cols();
    const Eigen::Index dim_u    = ub.size();
    Eigen::MatrixXd    Identity = Eigen::MatrixXd::Identity(dim_u, dim_u);

    // convert matrix to vector for osqpsolver
    std::vector<double> f(&f_vec(0), f_vec.data() + f_vec.cols() * f_vec.rows());

    std::vector<double> lower_bound;
    std::vector<double> upper_bound;

    for (int i = 0; i < dim_u; ++i) {
        lower_bound.push_back(lb(i));
        upper_bound.push_back(ub(i));
    }

    for (int i = 0; i < col_a; ++i) {
        lower_bound.push_back(lb_a(i));
        upper_bound.push_back(ub_a(i));
    }

    Eigen::MatrixXd osqpA = Eigen::MatrixXd(dim_u + col_a, raw_a);
    osqpA << Identity, a;

    /* execute optimization */
    auto result = osqpsolver_.optimize(h_mat, osqpA, f, lower_bound, upper_bound);

    std::vector<double> U_osqp = std::get<0>(result);
    u = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>>(&U_osqp[0], static_cast<Eigen::Index>(U_osqp.size()), 1);

    const int status_val = std::get<3>(result);
    if (status_val != 1) {
        RCLCPP_WARN(logger_, "optimization failed : %s", osqpsolver_.getStatusMessage().c_str());
        return false;
    }
    const auto has_nan = std::any_of(U_osqp.begin(), U_osqp.end(), [](const auto v) { return std::isnan(v); });
    if (has_nan) {
        RCLCPP_WARN(logger_, "optimization failed: result contains NaN values");
        return false;
    }

    // polish status: successful (1), unperformed (0), (-1) unsuccessful
    int status_polish = std::get<2>(result);
    if (status_polish == -1) {
        RCLCPP_WARN(logger_, "osqp status_polish = %d (unsuccessful)", status_polish);
        return false;
    }
    if (status_polish == 0) {
        RCLCPP_WARN(logger_, "osqp status_polish = %d (unperformed)", status_polish);
        return true;
    }
    return true;
}
