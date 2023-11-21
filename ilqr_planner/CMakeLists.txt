cmake_minimum_required(VERSION 3.8)
project(ilqr_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vehicle_model REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mpc_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTest)


# 设置头文件相对路径
include_directories(
    include
)
include_directories(${mpc_msgs_INCLUDE_DIRS})

set(DEPENDENCIES
    rclcpp
    vehicle_model
    nav_msgs
    mpc_msgs
    tf2
    tf2_geometry_msgs
    Eigen3
)

set(OPTIMIZER_SRC
    src/ilqr_optimizer.cpp
    src/optimizer_utils.cpp
    src/ilqr_path_optimizer.cpp
    src/ilqr_planner.cpp
)
add_library(optimizer_lib SHARED
    ${OPTIMIZER_SRC}
)
ament_target_dependencies(optimizer_lib
    ${DEPENDENCIES}
)

# release
add_executable(${PROJECT_NAME} src/optimizer_node.cpp)
target_link_libraries(${PROJECT_NAME}
    optimizer_lib
)
ament_target_dependencies(${PROJECT_NAME}
    ${DEPENDENCIES}
)

# tests
add_executable(${PROJECT_NAME}_test tests/ros_test_optimizer.cpp)
target_link_libraries(${PROJECT_NAME}_test
    optimizer_lib
    ${GTEST_LIBRARIES}
    ${GTEST_MAIN_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}_test 
    ${DEPENDENCIES}
)


install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_test
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# find_package(ament_cmake_gtest REQUIRED)
# ament_add_gtest(${PROJECT_NAME}_test tests/ros_test_optimizer.cpp)
# ament_target_dependencies(${PROJECT_NAME}_test rclcpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
