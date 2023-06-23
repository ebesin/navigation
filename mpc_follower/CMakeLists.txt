cmake_minimum_required(VERSION 3.8)
project(mpc_follower)

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
get_target_property(OSQP_INCLUDE_SUB_DIR osqp::osqp INTERFACE_INCLUDE_DIRECTORIES)
get_filename_component(OSQP_INCLUDE_DIR ${OSQP_INCLUDE_SUB_DIR} PATH)

include_directories(
  include
)

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
