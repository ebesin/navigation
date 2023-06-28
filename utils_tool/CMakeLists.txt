cmake_minimum_required(VERSION 3.5)

project(utils_tool)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mpc_msgs REQUIRED)

# 设置头文件相对路径
include_directories(
    include
)

set(visualization_tool_library_name visualization_tool_lib)
set(VISUALIZATION_TOOL_SRC
    src/visualization_tool.cpp
)
add_library(${visualization_tool_library_name} SHARED
    ${VISUALIZATION_TOOL_SRC}
)
set(VISUALIZATION_TOOL_DEPENDENCIES
    geometry_msgs
    rclcpp
    nav_msgs
    visualization_msgs
    tf2_geometry_msgs
)
ament_target_dependencies(${visualization_tool_library_name}
    ${VISUALIZATION_TOOL_DEPENDENCIES}
)

set(visualization_tool_exec_name visualization_tool_node)
set(VISUALIZATION_TOOL_EXEC_SRC
    src/visualization_node.cpp
)
add_executable(${visualization_tool_exec_name} ${VISUALIZATION_TOOL_EXEC_SRC})
set(VISUALIZATION_TOOL_EXEC_DEPENDENCIES
    mpc_msgs
)
ament_target_dependencies(${visualization_tool_exec_name}
    ${VISUALIZATION_TOOL_EXEC_DEPENDENCIES}
)
target_link_libraries(${visualization_tool_exec_name}
    ${visualization_tool_library_name}
)

install(TARGETS ${visualization_tool_exec_name}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${visualization_tool_library_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

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