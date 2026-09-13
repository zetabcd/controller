cmake_minimum_required(VERSION 3.5)
project(fms_utils)

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
find_package(rclcpp REQUIRED)

# 添加源文件，生成库
add_library(openfsm SHARED
        include/${PROJECT_NAME}/openfsm.h
        src/openfsm.cpp
        )

# 用于代替传统的target_link_libraries
ament_target_dependencies(openfsm
        PUBLIC
        rclcpp
        )

# 添加头文件地址
target_include_directories(openfsm PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
        )


# 为了使下游文件可以访问
ament_export_targets(openfsm HAS_LIBRARY_TARGET)
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
        TARGETS openfsm
        EXPORT openfsm
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

ament_package()
