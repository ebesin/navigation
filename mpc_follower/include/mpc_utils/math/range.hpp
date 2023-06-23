/*
 * @Author       : dwayne
 * @Date         : 2023-06-21
 * @LastEditTime : 2023-06-21
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef MPC_UTILS__MATH__RANGE_HPP_
#define MPC_UTILS__MATH__RANGE_HPP_

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace mpc_utils {
template<class T> std::vector<T> arange(const T start, const T stop, const T step = 1)
{
    if (step == 0) {
        throw std::invalid_argument("step must be non-zero value.");
    }

    if (step > 0 && stop < start) {
        throw std::invalid_argument("must be stop >= start for positive step.");
    }

    if (step < 0 && stop > start) {
        throw std::invalid_argument("must be stop <= start for negative step.");
    }

    const double max_i_double = std::ceil(static_cast<double>(stop - start) / step);
    const auto   max_i        = static_cast<size_t>(max_i_double);

    std::vector<T> out;
    out.reserve(max_i);
    for (size_t i = 0; i < max_i; ++i) {
        out.push_back(start + i * step);
    }

    return out;
}

template<class T> std::vector<double> linspace(const T start, const T stop, const size_t num)
{
    const auto start_double = static_cast<double>(start);
    const auto stop_double  = static_cast<double>(stop);

    if (num == 0) {
        return {};
    }

    if (num == 1) {
        return {start_double};
    }

    std::vector<double> out;
    out.reserve(num);
    const double step = (stop_double - start_double) / static_cast<double>(num - 1);
    for (size_t i = 0; i < num; i++) {
        out.push_back(start_double + static_cast<double>(i) * step);
    }

    return out;
}

}   // namespace mpc_utils

#endif   // MPC_UTILS__MATH__RANGE_HPP_
