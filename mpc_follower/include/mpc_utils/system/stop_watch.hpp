/*
 * @Author       : dwayne
 * @Date         : 2023-06-21
 * @LastEditTime : 2023-06-23
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef TIER4_AUTOWARE_UTILS__SYSTEM__STOP_WATCH_HPP_
#define TIER4_AUTOWARE_UTILS__SYSTEM__STOP_WATCH_HPP_

#include <chrono>
#include <string>
#include <unordered_map>

namespace mpc_utils {
template<class OutputUnit   = std::chrono::seconds,
         class InternalUnit = std::chrono::microseconds,
         class Clock        = std::chrono::steady_clock>
class StopWatch
{
public:
    StopWatch() { tic(default_name); }

    void tic(const std::string& name = default_name) { t_start_[name] = Clock::now(); }

    void tic(const char* name) { tic(std::string(name)); }

    double toc(const std::string& name, const bool reset = false)
    {
        const auto t_start  = t_start_.at(name);
        const auto t_end    = Clock::now();
        const auto duration = std::chrono::duration_cast<InternalUnit>(t_end - t_start).count();

        if (reset) {
            t_start_[name] = Clock::now();
        }

        const auto one_sec = std::chrono::duration_cast<InternalUnit>(OutputUnit(1)).count();

        return static_cast<double>(duration) / one_sec;
    }

    double toc(const char* name, const bool reset = false) { return toc(std::string(name), reset); }

    double toc(const bool reset = false) { return toc(default_name, reset); }

private:
    using Time = std::chrono::time_point<Clock>;
    static constexpr const char* default_name{"__auto__"};

    std::unordered_map<std::string, Time> t_start_;
};
}   // namespace mpc_utils

#endif   // TIER4_AUTOWARE_UTILS__SYSTEM__STOP_WATCH_HPP_
