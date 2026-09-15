cmake_minimum_required(VERSION 3.5)
project(uav_utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)

# C++ utilities remain header-only; Python nodes share project-path resolution.
ament_python_install_package(${PROJECT_NAME})
add_library(uav_utils INTERFACE)

# 用于代替传统的target_link_libraries
ament_target_dependencies(uav_utils
        INTERFACE
        rclcpp
        )

# 添加头文件地址
target_include_directories(uav_utils INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
        )


# 为了使下游文件可以访问
ament_export_targets(uav_utils HAS_LIBRARY_TARGET)
ament_export_dependencies(
        rclcpp
)


# 注册 导出头文件
install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
)

# 注册 导出库文件
install(
        TARGETS uav_utils
        EXPORT uav_utils
        INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(project_paths_python test/test_project_paths.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  add_executable(project_paths_test test/project_paths_test.cpp)
  target_compile_features(project_paths_test PRIVATE cxx_std_17)
  target_include_directories(project_paths_test PRIVATE include)
  add_test(NAME project_paths_cpp COMMAND project_paths_test)
endif()

ament_package()
