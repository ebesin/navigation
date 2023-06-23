/*
 * @Author       : dwayne
 * @Date         : 2023-06-21
 * @LastEditTime : 2023-06-21
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef OSQP_INTERFACE__VISIBILITY_CONTROL_HPP_
#define OSQP_INTERFACE__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
#    if defined(OSQP_INTERFACE_BUILDING_DLL) || defined(OSQP_INTERFACE_EXPORTS)
#        define OSQP_INTERFACE_PUBLIC __declspec(dllexport)
#        define OSQP_INTERFACE_LOCAL
#    else   // defined(OSQP_INTERFACE_BUILDING_DLL) || defined(OSQP_INTERFACE_EXPORTS)
#        define OSQP_INTERFACE_PUBLIC __declspec(dllimport)
#        define OSQP_INTERFACE_LOCAL
#    endif   // defined(OSQP_INTERFACE_BUILDING_DLL) || defined(OSQP_INTERFACE_EXPORTS)
#elif defined(__linux__)
#    define OSQP_INTERFACE_PUBLIC __attribute__((visibility("default")))
#    define OSQP_INTERFACE_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
#    define OSQP_INTERFACE_PUBLIC __attribute__((visibility("default")))
#    define OSQP_INTERFACE_LOCAL __attribute__((visibility("hidden")))
#else
#    error "Unsupported Build Configuration"
#endif

#endif   // OSQP_INTERFACE__VISIBILITY_CONTROL_HPP_
