/*
 * @Author       : dwayne
 * @Date         : 2023-06-19
 * @LastEditTime : 2023-06-19
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef AMATHUTILS_LIB_TIME_DELAY_KALMAN_FILTER_HPP
#define AMATHUTILS_LIB_TIME_DELAY_KALMAN_FILTER_HPP

#include "amathutils_lib/kalman_filter.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <iostream>

/**
 * @file time_delay_kalman_filter.h
 * @brief kalman filter with delayed measurement class
 * @author Takamasa Horibe
 * @date 2019.05.01
 */

class TimeDelayKalmanFilter : public KalmanFilter
{
public:
    /**
   * @brief No initialization constructor.
   */
    TimeDelayKalmanFilter();

    /**
   * @brief initialization of kalman filter
   * @param x initial state
   * @param P0 initial covariance of estimated state
   * @param max_delay_step Maximum number of delay steps, which determines the dimension of the extended kalman filter
   */
    void init(const Eigen::MatrixXd& x, const Eigen::MatrixXd& P, const int max_delay_step);

    /**
   * @brief get latest time estimated state
   * @param x latest time estimated state
   */
    void getLatestX(Eigen::MatrixXd& x);

    /**
   * @brief get latest time estimation covariance
   * @param P latest time estimation covariance
   */
    void getLatestP(Eigen::MatrixXd& P);

    /**
   * @brief calculate kalman filter covariance by predicion model with time delay. This is mainly for EKF of nonlinear process model.
   * @param x_next predicted state by prediction model
   * @param A coefficient matrix of x for process model
   * @param Q covariace matrix for process model
   */
    bool predictWithDelay(const Eigen::MatrixXd& x_next, const Eigen::MatrixXd& A, const Eigen::MatrixXd& Q);

    /**
   * @brief calculate kalman filter covariance by measurement model with time delay. This is mainly for EKF of nonlinear process model.
   * @param y measured values
   * @param C coefficient matrix of x for measurement model
   * @param R covariance matrix for measurement model
   * @param delay_step measurement delay
   */
    bool updateWithDelay(const Eigen::MatrixXd& y, const Eigen::MatrixXd& C, const Eigen::MatrixXd& R, const int delay_step);

private:
    int max_delay_step_;   //!< @brief maximum number of delay steps
    int dim_x_;            //!< @brief dimension of latest state
    int dim_x_ex_;         //!< @brief dimension of extended state with dime delay
};

#endif   // AMATHUTILS_LIB_TIME_DELAY_KALMAN_FILTER_HPP
