/*
 * @Author       : dwayne
 * @Date         : 2023-06-21
 * @LastEditTime : 2023-06-21
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#include "osqp_interface/csc_matrix_conv.hpp"

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include <exception>
#include <iostream>
#include <vector>


namespace osqp {
CSC_Matrix calCSCMatrix(const Eigen::MatrixXd& mat)
{
    const size_t       elem = static_cast<size_t>(mat.nonZeros());
    const Eigen::Index rows = mat.rows();
    const Eigen::Index cols = mat.cols();

    std::vector<c_float> vals;
    vals.reserve(elem);
    std::vector<c_int> row_idxs;
    row_idxs.reserve(elem);
    std::vector<c_int> col_idxs;
    col_idxs.reserve(elem);

    // Construct CSC matrix arrays
    c_float val;
    c_int   elem_count = 0;

    col_idxs.push_back(0);

    for (Eigen::Index j = 0; j < cols; j++) {       // col iteration
        for (Eigen::Index i = 0; i < rows; i++) {   // row iteration
            // Get values of nonzero elements
            val = mat(i, j);
            if (std::fabs(val) < 1e-9) {
                continue;
            }
            elem_count += 1;

            // Store values
            vals.push_back(val);
            row_idxs.push_back(i);
        }

        col_idxs.push_back(elem_count);
    }

    CSC_Matrix csc_matrix = {vals, row_idxs, col_idxs};

    return csc_matrix;
}

CSC_Matrix calCSCMatrixTrapezoidal(const Eigen::MatrixXd& mat)
{
    const size_t       elem = static_cast<size_t>(mat.nonZeros());
    const Eigen::Index rows = mat.rows();
    const Eigen::Index cols = mat.cols();

    if (rows != cols) {
        throw std::invalid_argument("Matrix must be square (n, n)");
    }

    std::vector<c_float> vals;
    vals.reserve(elem);
    std::vector<c_int> row_idxs;
    row_idxs.reserve(elem);
    std::vector<c_int> col_idxs;
    col_idxs.reserve(elem);

    // Construct CSC matrix arrays
    c_float      val;
    Eigen::Index trap_last_idx = 0;
    c_int        elem_count    = 0;

    col_idxs.push_back(0);

    for (Eigen::Index j = 0; j < cols; j++) {                 // col iteration
        for (Eigen::Index i = 0; i <= trap_last_idx; i++) {   // row iteration
            // Get values of nonzero elements
            val = mat(i, j);
            if (std::fabs(val) < 1e-9) {
                continue;
            }
            elem_count += 1;

            // Store values
            vals.push_back(val);
            row_idxs.push_back(i);
        }

        col_idxs.push_back(elem_count);
        trap_last_idx += 1;
    }

    CSC_Matrix csc_matrix = {vals, row_idxs, col_idxs};

    return csc_matrix;
}

void printCSCMatrix(const CSC_Matrix& csc_mat)
{
    std::cout << "[";
    for (const c_float& val : csc_mat.m_vals) {
        std::cout << val << ", ";
    }
    std::cout << "]\n";

    std::cout << "[";
    for (const c_int& row : csc_mat.m_row_idxs) {
        std::cout << row << ", ";
    }
    std::cout << "]\n";

    std::cout << "[";
    for (const c_int& col : csc_mat.m_col_idxs) {
        std::cout << col << ", ";
    }
    std::cout << "]\n";
}

}   // namespace osqp