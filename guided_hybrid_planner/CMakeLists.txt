cmake_minimum_required(VERSION 3.5)

project(guided_hybrid_planner)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(navigation2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# 设置头文件相对路径
include_directories(
  include
)


# 编译可执行文件
set(executable_name guided_hybrid_planner_flow)
add_executable(${executable_name}
  src/main.cpp
)

# 编译静态库
set(library_name guided_hybrid_planner CACHE INTERNAL "library")
add_library(${library_name} SHARED 
  src/guided_hybrid_astar.cpp
  src/hybrid_state_node.cpp
  src/guided_hybrid_planner.cpp
  src/rs_path.cpp
  src/collision_checker.cpp
  src/costmap_downsampler.cpp
  src/goal_pose_subscriber.cpp
  src/init_pose_subscriber.cpp
  src/guided_hybrid_astar_flow.cpp
  src/visualization_tools.cpp
)

set(dependencies
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  lifecycle_msgs
  rclcpp_lifecycle
  nav2_util
  nav2_core
  tf2_geometry_msgs
  Eigen3
)

# 链接库和依赖
ament_target_dependencies(${library_name}
  ${dependencies}
)

# 链接库和依赖
ament_target_dependencies(${executable_name}
  ${dependencies}
)


# 链接
target_link_libraries(${executable_name} ${library_name})

# target_include_directories(dwayne_nav PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(nav2_core guided_hybrid_planner.xml)


install(DIRECTORY include/  DESTINATION include/)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY map DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_subdirectory(tests)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()