cmake_minimum_required(VERSION 3.8)
project(mpc_follower)
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(mpc_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(osqp_vendor REQUIRED)
find_package(tier4_debug_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_common REQUIRED)
get_target_property(OSQP_INCLUDE_SUB_DIR osqp::osqp INTERFACE_INCLUDE_DIRECTORIES)
get_filename_component(OSQP_INCLUDE_DIR ${OSQP_INCLUDE_SUB_DIR} PATH)

include_directories(
  include
)
include_directories(SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
set(DEPENDENCIES
  rclcpp
  Eigen3
  mpc_msgs
  geometry_msgs
  nav_msgs
  tier4_debug_msgs
  visualization_msgs
  diagnostic_msgs
  unique_identifier_msgs
  PCL
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ###############################################################################
# ##############
# OSQP_INTERFACE
# ##############
# 编译静态库
set(osqp_library_name osqp_interface CACHE INTERNAL "library")
set(OSQP_INTERFACE_LIB_SRC
  src/osqp_interface/csc_matrix_conv.cpp
  src/osqp_interface/osqp_interface.cpp
)
add_library(${osqp_library_name} SHARED
  ${OSQP_INTERFACE_LIB_SRC}
)

# 设置头文件路径
target_include_directories(${osqp_library_name}
  SYSTEM PUBLIC
  "${OSQP_INCLUDE_DIR}"
  "${EIGEN3_INCLUDE_DIR}"
)
set(OSQP_DEPENDENCIES
  rclcpp
  Eigen3
  osqp_vendor
)

# 链接库和依赖
ament_target_dependencies(${osqp_library_name}
  ${OSQP_DEPENDENCIES}
)

# ###############################################################################
# ##############
# VEHICLE_MODEL
# ##############
set(vehicle_model_library_name vehicle_model_lib CACHE INTERNAL "library")
set(VEHICLE_MODEL_LIB_SRC
  src/mpc_follower/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.cpp
  src/mpc_follower/vehicle_model/vehicle_model_interface.cpp
)
add_library(${vehicle_model_library_name} SHARED
  ${VEHICLE_MODEL_LIB_SRC}
)
target_include_directories(${vehicle_model_library_name}
  SYSTEM PUBLIC
  "${EIGEN3_INCLUDE_DIR}"
)
set(VEHICLE_MODEL_DEPENDENCIES
  Eigen3
)
ament_target_dependencies(${vehicle_model_library_name}
  ${VEHICLE_MODEL_DEPENDENCIES}
)
ament_target_dependencies(${vehicle_model_library_name}
  ${DEPENDENCIES}
)
# ###############################################################################
# ##############
# OSQP_SOLVER
# ##############
set(osqp_solver_library_name osqp_solver_lib CACHE INTERNAL "library")
set(OSQP_SOLVER_LIB_SRC
  src/qp_solver/qp_solver_osqp.cpp
  src/qp_solver/qp_solver_unconstr_fast.cpp
)
add_library(${osqp_solver_library_name} SHARED
  ${OSQP_SOLVER_LIB_SRC}
)
set(OSQP_SOLVER_DEPENDENCIES
  rclcpp
  Eigen3
)
ament_target_dependencies(${vehicle_model_library_name}
  ${OSQP_SOLVER_DEPENDENCIES}
)
target_link_libraries(${osqp_solver_library_name} ${osqp_library_name})

# ###############################################################################
# ##############
# MPC_UTILS
# ##############
set(mpc_utils_library_name mpc_utils_lib CACHE INTERNAL "library")
set(MPC_UTILS_LIB_SRC
  src/mpc_utils/mpc_utils.cpp
  src/mpc_utils/math/sin_table.cpp
  src/mpc_utils/math/trigonometry.cpp
  src/mpc_utils/ros/msg_operation.cpp
)
add_library(${mpc_utils_library_name} SHARED
  ${MPC_UTILS_LIB_SRC}
)
target_include_directories(${mpc_utils_library_name}
  SYSTEM PUBLIC
  "${EIGEN3_INCLUDE_DIR}"
)
set(MPC_UTILS_DEPENDENCIES
  rclcpp
  Eigen3
  mpc_msgs
  geometry_msgs
  nav_msgs
  tier4_debug_msgs
  visualization_msgs
  diagnostic_msgs
  unique_identifier_msgs
  PCL
  tf2
  tf2_ros
  tf2_geometry_msgs
)
ament_target_dependencies(${mpc_utils_library_name}
  ${MPC_UTILS_DEPENDENCIES}
)

# ###############################################################################
# ##############
# INTERPOLATION
# ##############
set(interpolation_library_name interpolation_lib CACHE INTERNAL "library")
set(INTERPOLATION_LIB_SRC
  src/interpolation/linear_interpolation.cpp
  src/interpolation/spline_interpolation.cpp
  src/interpolation/spline_interpolation_points_2d.cpp
  src/interpolation/spherical_linear_interpolation.cpp
)
add_library(${interpolation_library_name} SHARED
  ${INTERPOLATION_LIB_SRC}
)
set(INTERPOLATION_DEPENDENCIES
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
ament_target_dependencies(${interpolation_library_name}
  ${INTERPOLATION_DEPENDENCIES}
)
ament_target_dependencies(${interpolation_library_name}
  ${DEPENDENCIES}
)

target_link_libraries(${interpolation_library_name} ${mpc_utils_library_name})

# ###############################################################################
# ##############
# MOTION_UTILS
# ##############
find_package(Boost REQUIRED)
set(motion_utils_library_name motion_utils_lib CACHE INTERNAL "library")
set(MOTION_UTILS_LIB_SRC
  src/motion_utils/motion_utils.cpp
  src/motion_utils/distance/distance.cpp
  src/motion_utils/marker/marker_helper.cpp
  src/motion_utils/marker/virtual_wall_marker_creator.cpp
  src/motion_utils/resample/resample.cpp
  src/motion_utils/trajectory/interpolation.cpp
  src/motion_utils/trajectory/path_with_lane_id.cpp
  src/motion_utils/vehicle/vehicle_state_checker.cpp
)
add_library(${motion_utils_library_name} SHARED
  ${MOTION_UTILS_LIB_SRC}
)
set(MOTION_UTILS_DEPENDENCIES
  geometry_msgs
  visualization_msgs
  mpc_msgs
  rclcpp
  rclcpp
  nav_msgs
)
ament_target_dependencies(${motion_utils_library_name}
  ${MOTION_UTILS_DEPENDENCIES}
)
ament_target_dependencies(${motion_utils_library_name}
  ${DEPENDENCIES}
)
target_link_libraries(${motion_utils_library_name}
    ${interpolation_library_name}
    ${mpc_utils_library_name}
)

# ###############################################################################


# ##############
# VEHICLE_INFO_UTIL
# ##############
set(vehicle_info_util_library_name vehicle_info_util_lib CACHE INTERNAL "library")
set(VEHICLE_INFO_UTIL_SRC
  src/vehicle_info_util/vehicle_info_util.cpp
)
add_library(${vehicle_info_util_library_name} SHARED
  ${VEHICLE_INFO_UTIL_SRC}
)
ament_target_dependencies(${vehicle_info_util_library_name}
  ${DEPENDENCIES}
)
target_link_libraries(${vehicle_info_util_library_name}
  ${mpc_utils_library_name}
)

# ##############
# STEERING_OFFSET
# ##############
set(steering_offset_library_name steering_offset_lib CACHE INTERNAL "library")
set(STEERING_OFFSET_SRC
  src/steering_offset/steering_offset.cpp
)
add_library(${steering_offset_library_name} SHARED
  ${STEERING_OFFSET_SRC}
)
set(STEERING_OFFSET_DEPENDENCIES
  geometry_msgs
)
ament_target_dependencies(${steering_offset_library_name}
  ${STEERING_OFFSET_DEPENDENCIES}
)
ament_target_dependencies(${steering_offset_library_name}
  ${DEPENDENCIES}
)
# ##############
# MPC
# ##############
set(mpc_library_name mpc_lib CACHE INTERNAL "library")
set(MPC_LIB_SRC
  src/mpc_follower/lowpass_filter.cpp
  src/mpc_follower/mpc_trajectory.cpp
  src/mpc_follower/mpc_utils.cpp
  src/mpc_follower/steering_predictor.cpp
  src/mpc_follower/mpc.cpp
  src/mpc_follower/mpc_controller.cpp
)
add_library(${mpc_library_name} SHARED
  ${MPC_LIB_SRC}
)
set(MPC_DEPENDENCIES
  nav2_util
)
ament_target_dependencies(${mpc_library_name}
  ${MPC_DEPENDENCIES}
)
ament_target_dependencies(${mpc_library_name}
  ${DEPENDENCIES}
)
target_link_libraries(${mpc_library_name}
  ${mpc_utils_library_name}
  ${interpolation_library_name}
  ${motion_utils_library_name}
  ${osqp_solver_library_name}
  ${osqp_library_name}
  ${vehicle_model_library_name}
  ${vehicle_info_util_library_name}
  ${steering_offset_library_name}
)


set(mpc_controller_exec_name mpc_controller_node)
set(MPC_CONTROLLER_EXEC__SRC
    src/mpc_follower/mpc_controller_node.cpp
)
add_executable(${mpc_controller_exec_name} ${MPC_CONTROLLER_EXEC__SRC})
set(MPC_CONTROLLER_DEPENDENCIES
  rclcpp
  Eigen3
  mpc_msgs
  geometry_msgs
  nav_msgs
  tier4_debug_msgs
  visualization_msgs
  diagnostic_msgs
  unique_identifier_msgs
  PCL
  tf2
  tf2_ros
  tf2_geometry_msgs
)
ament_target_dependencies(${mpc_controller_exec_name}
  ${MPC_CONTROLLER_DEPENDENCIES}
)
target_link_libraries(${mpc_controller_exec_name}
    ${mpc_library_name}
)

install(TARGETS ${mpc_controller_exec_name}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
