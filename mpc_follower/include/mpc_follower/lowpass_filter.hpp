/*
 * @Author       : dwayne
 * @Date         : 2023-06-19
 * @LastEditTime : 2023-06-21
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef MPC_LATERAL_CONTROLLER__LOWPASS_FILTER_HPP_
#define MPC_LATERAL_CONTROLLER__LOWPASS_FILTER_HPP_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

/**
 * @brief 2nd-order Butterworth Filter
 * reference : S. Butterworth, "On the Theory of Filter Amplifier", Experimental wireless, 1930.
 */
class Butterworth2dFilter
{
private:
    double m_y1;   //!< @brief filter coefficient calculated with cutoff frequency and sampling time
    double m_y2;   //!< @brief filter coefficient calculated with cutoff frequency and sampling time
    double m_u1;   //!< @brief filter coefficient calculated with cutoff frequency and sampling time
    double m_u2;   //!< @brief filter coefficient calculated with cutoff frequency and sampling time
    double m_a1;   //!< @brief filter coefficient calculated with cutoff frequency and sampling time
    double m_a2;   //!< @brief filter coefficient calculated with cutoff frequency and sampling time
    double m_b0;   //!< @brief filter coefficient calculated with cutoff frequency and sampling time
    double m_b1;   //!< @brief filter coefficient calculated with cutoff frequency and sampling time
    double m_b2;   //!< @brief filter coefficient calculated with cutoff frequency and sampling time

public:
    /**
   * @brief constructor with initialization
   * @param [in] dt sampling time
   * @param [in] f_cutoff_hz cutoff frequency [Hz]
   */
    explicit Butterworth2dFilter(double dt = 0.01, double f_cutoff_hz = 5.0);

    /**
   * @brief destructor
   */
    ~Butterworth2dFilter();

    /**
   * @brief constructor
   * @param [in] dt sampling time
   * @param [in] f_cutoff_hz cutoff frequency [Hz]
   */
    void initialize(const double& dt, const double& f_cutoff_hz);

    /**
   * @brief filtering (call this function at each sampling time with input)
   * @param [in] u scalar input for filter
   * @return filtered scalar value
   */
    double filter(const double& u);

    /**
   * @brief filtering for time-series data
   * @param [in] t time-series data for input vector
   * @param [out] u object vector
   */
    void filt_vector(const std::vector<double>& t, std::vector<double>& u) const;

    /**
   * @brief filtering for time-series data from both forward-backward direction for zero phase delay
   * @param [in] t time-series data for input vector
   * @param [out] u object vector
   */
    void filtfilt_vector(const std::vector<double>& t,
                         std::vector<double>&       u) const;   // filtering forward and backward direction

    /**
   * @brief get filter coefficients
   * @param [out] coeffs coefficients of filter [a0, a1, a2, b0, b1, b2].
   */
    void getCoefficients(std::vector<double>& coeffs) const;
};

/**
 * @brief Move Average Filter
 */
namespace MoveAverageFilter {
/**
 * @brief filtering vector
 * @param [in] num index distance for moving average filter
 * @param [out] u object vector
 */
bool filt_vector(const int num, std::vector<double>& u);
}   // namespace MoveAverageFilter
#endif   // MPC_LATERAL_CONTROLLER__LOWPASS_FILTER_HPP_
