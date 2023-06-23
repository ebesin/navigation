/*
 * @Author       : dwayne
 * @Date         : 2023-06-21
 * @LastEditTime : 2023-06-21
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef OSQP_INTERFACE__CSC_MATRIX_CONV_HPP_
#define OSQP_INTERFACE__CSC_MATRIX_CONV_HPP_

#include "osqp/glob_opts.h"   // for 'c_int' type ('long' or 'long long')
#include "osqp_interface/visibility_control.hpp"

#include <Eigen/Core>

#include <vector>


namespace osqp {
/// \brief Compressed-Column-Sparse Matrix
struct OSQP_INTERFACE_PUBLIC CSC_Matrix
{
    /// Vector of non-zero values. Ex: [4,1,1,2]
    std::vector<c_float> m_vals;
    /// Vector of row index corresponding to values. Ex: [0, 1, 0, 1] (Eigen: 'inner')
    std::vector<c_int> m_row_idxs;
    /// Vector of 'val' indices where each column starts. Ex: [0, 2, 4] (Eigen: 'outer')
    std::vector<c_int> m_col_idxs;
};

/// \brief Calculate CSC matrix from Eigen matrix
OSQP_INTERFACE_PUBLIC CSC_Matrix calCSCMatrix(const Eigen::MatrixXd& mat);
/// \brief Calculate upper trapezoidal CSC matrix from square Eigen matrix
OSQP_INTERFACE_PUBLIC CSC_Matrix calCSCMatrixTrapezoidal(const Eigen::MatrixXd& mat);
/// \brief Print the given CSC matrix to the standard output
OSQP_INTERFACE_PUBLIC void printCSCMatrix(const CSC_Matrix& csc_mat);

}   // namespace osqp


#endif   // OSQP_INTERFACE__CSC_MATRIX_CONV_HPP_
