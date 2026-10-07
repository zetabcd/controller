cmake_minimum_required(VERSION 3.5)
project(px4ctrl)
# 仿真/实机选择在 include/px4ctrl/input.h；不再读取 CMake 缓存中的同名选项。

# sun: px4ctrl_node 运行外环与状态机，px4ctrlrate_node 运行高频角速度环和电机分配。
# sun: 两个可执行程序通过 /rates_thrust_setpoint 解耦，频率可分别配置。

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
message(STATUS "CMAKE_MODULE_PATH = ${CMAKE_MODULE_PATH}")
message(STATUS "CMAKE_ROOT = ${CMAKE_ROOT}")

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(fms_utils REQUIRED)
find_package(uav_utils REQUIRED)
find_package(px4debug_msgs REQUIRED)
find_package(joy_msgs REQUIRED)
find_package(ratectrl_msgs REQUIRED)
find_package(gap_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# All third-party libraries are built natively and installed into a normal
# prefix (the provided script uses /usr/local).  3rdpart/src contains source
# checkouts only and is deliberately not part of CMake's dependency search.
find_package(Eigen3 3.3 CONFIG REQUIRED)
find_package(osqp 1.0 CONFIG REQUIRED)
find_package(acados CONFIG REQUIRED)

set(PX4CTRL_ACADOS_GENERATED
  "${CMAKE_CURRENT_SOURCE_DIR}/generated/acados/px4ctrl_nmpc")

# px4ctrlrate_node uses the system SuiteSparse/UMFPACK development package.
find_path(UMFPACK_INCLUDE_DIR NAMES suitesparse/umfpack.h REQUIRED)
find_library(UMFPACK_LIBRARY NAMES umfpack REQUIRED)

message(STATUS "px4ctrl Eigen: Eigen3::Eigen")
message(STATUS "px4ctrl OSQP: osqp::osqp")
message(STATUS "px4ctrl acados: acados")
message(STATUS "px4ctrl UMFPACK: ${UMFPACK_LIBRARY}")

# The generated solver belongs to px4ctrl, so build it on the target machine
# instead of committing or locating a prebuilt libacados_ocp_solver*.so.
add_library(px4ctrl_acados_ocp_solver STATIC
  "${PX4CTRL_ACADOS_GENERATED}/acados_solver_px4ctrl_nmpc.c"
  "${PX4CTRL_ACADOS_GENERATED}/px4ctrl_nmpc_model/px4ctrl_nmpc_expl_ode_fun.c"
  "${PX4CTRL_ACADOS_GENERATED}/px4ctrl_nmpc_model/px4ctrl_nmpc_expl_vde_forw.c"
  "${PX4CTRL_ACADOS_GENERATED}/px4ctrl_nmpc_model/px4ctrl_nmpc_expl_vde_adj.c"
  "${PX4CTRL_ACADOS_GENERATED}/px4ctrl_nmpc_cost/px4ctrl_nmpc_cost_y_0_fun.c"
  "${PX4CTRL_ACADOS_GENERATED}/px4ctrl_nmpc_cost/px4ctrl_nmpc_cost_y_0_fun_jac_ut_xt.c"
  "${PX4CTRL_ACADOS_GENERATED}/px4ctrl_nmpc_cost/px4ctrl_nmpc_cost_y_0_hess.c"
  "${PX4CTRL_ACADOS_GENERATED}/px4ctrl_nmpc_cost/px4ctrl_nmpc_cost_y_fun.c"
  "${PX4CTRL_ACADOS_GENERATED}/px4ctrl_nmpc_cost/px4ctrl_nmpc_cost_y_fun_jac_ut_xt.c"
  "${PX4CTRL_ACADOS_GENERATED}/px4ctrl_nmpc_cost/px4ctrl_nmpc_cost_y_hess.c"
  "${PX4CTRL_ACADOS_GENERATED}/px4ctrl_nmpc_cost/px4ctrl_nmpc_cost_y_e_fun.c"
  "${PX4CTRL_ACADOS_GENERATED}/px4ctrl_nmpc_cost/px4ctrl_nmpc_cost_y_e_fun_jac_ut_xt.c"
  "${PX4CTRL_ACADOS_GENERATED}/px4ctrl_nmpc_cost/px4ctrl_nmpc_cost_y_e_hess.c")
target_include_directories(px4ctrl_acados_ocp_solver PUBLIC
  "${PX4CTRL_ACADOS_GENERATED}")
target_link_libraries(px4ctrl_acados_ocp_solver PUBLIC acados hpipm blasfeo m)

# ROS-independent acados solver, also exercised directly by regression tests.
add_library(px4ctrl_reference STATIC src/control_reference.cpp src/trajectory.cpp src/sampled_trajectory.cpp src/external_trajectory.cpp)
target_include_directories(px4ctrl_reference PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS px4ctrl_reference EXPORT px4ctrl_referenceTargets ARCHIVE DESTINATION lib)
ament_export_targets(px4ctrl_referenceTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
target_link_libraries(px4ctrl_reference PUBLIC Eigen3::Eigen)

add_library(px4ctrl_omtraj STATIC src/omtraj.cpp src/omtraj_dynamics.cpp src/omtraj_io.cpp src/omtraj_reference.cpp)
target_include_directories(px4ctrl_omtraj PUBLIC include)
target_link_libraries(px4ctrl_omtraj PUBLIC px4ctrl_reference Eigen3::Eigen osqp::osqp)

add_executable(trajectory_inspect src/trajectory_inspect.cpp)
target_link_libraries(trajectory_inspect px4ctrl_reference)
install(TARGETS trajectory_inspect DESTINATION lib/${PROJECT_NAME})

add_library(px4ctrl_acados_nmpc STATIC src/acados_nmpc_solver.cpp)
target_include_directories(px4ctrl_acados_nmpc PUBLIC include)
target_link_libraries(px4ctrl_acados_nmpc PUBLIC Eigen3::Eigen px4ctrl_acados_ocp_solver)

add_library(px4ctrl_ommpc STATIC src/ommpc_solver.cpp)
target_include_directories(px4ctrl_ommpc PUBLIC include)
target_link_libraries(px4ctrl_ommpc PUBLIC px4ctrl_reference osqp::osqp)

# sun: 顶层目标包含输入转换、状态机、外环控制器、参数和坐标变换实现。
add_executable(px4ctrl_node 
  src/px4ctrl_node.cpp
  src/fcu_reboot.cpp
  src/external_execution.cpp
	src/input.cpp
  src/px4ctrlfsm.cpp
  src/controller.cpp
  src/px4ctrlparam.cpp
  src/frame_transforms.cpp
  src/ommpc.cpp
  src/acados_nmpc.cpp
)
ament_target_dependencies(px4ctrl_node 
  rclcpp 
  ament_index_cpp
  geometry_msgs
  sensor_msgs
  px4_msgs 
  fms_utils 
  uav_utils 
  px4debug_msgs 
  joy_msgs 
  ratectrl_msgs
  gap_msgs
  std_srvs
)
target_include_directories(px4ctrl_node PRIVATE include)
target_link_libraries(px4ctrl_node
  Eigen3::Eigen
  osqp::osqp
  px4ctrl_acados_nmpc
  px4ctrl_ommpc
  px4ctrl_reference
  px4ctrl_omtraj
  m)

add_executable(omtraj_visualizer_node
  src/omtraj_visualizer_node.cpp
)
ament_target_dependencies(omtraj_visualizer_node
  rclcpp
  ament_index_cpp
  uav_utils
  geometry_msgs
  visualization_msgs
)
target_include_directories(omtraj_visualizer_node SYSTEM PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_include_directories(omtraj_visualizer_node PRIVATE include)
target_link_libraries(omtraj_visualizer_node px4ctrl_omtraj)

# 实机轨迹窗口复用 quadsim 的红/绿线数据来源：PX4 实际位置与控制器参考位置。
# GUI 可在地面站电脑运行，机载电脑只需要继续发布原有 ROS 话题。
add_executable(realflight_trajectory_visualizer_node
  src/realflight_trajectory_visualizer_node.cpp)
ament_target_dependencies(realflight_trajectory_visualizer_node
  rclcpp
  geometry_msgs
  visualization_msgs
  px4_msgs
  px4debug_msgs
)

# sun: 底层目标依赖 SuiteSparse/UMFPACK，为滑动窗口 TV 正则微分求解稀疏线性系统。
add_executable(px4ctrlrate_node 
  src/px4ctrlrate_node.cpp
  src/px4ctrlparam.cpp
  src/frame_transforms.cpp
)
ament_target_dependencies(px4ctrlrate_node 
  rclcpp px4_msgs 
  geometry_msgs
  sensor_msgs
  fms_utils 
  uav_utils 
  px4debug_msgs 
  joy_msgs 
  ratectrl_msgs
  gap_msgs
  std_srvs
)
target_include_directories(px4ctrlrate_node SYSTEM PRIVATE
  "${UMFPACK_INCLUDE_DIR}")
target_include_directories(px4ctrlrate_node PRIVATE include)
target_link_libraries(px4ctrlrate_node Eigen3::Eigen ${UMFPACK_LIBRARY})

# 独立实验节点：只使用 PX4 原生位置控制器，不链接现有 FSM/MPC/电机分配链。
add_executable(px4_native_position_node
  src/px4_native_position_node.cpp
  src/px4_native_position_control.cpp)
ament_target_dependencies(px4_native_position_node
  rclcpp geometry_msgs px4_msgs joy_msgs)
target_include_directories(px4_native_position_node PRIVATE include)
target_link_libraries(px4_native_position_node Eigen3::Eigen)

install(TARGETS px4ctrl_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS px4ctrlrate_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS omtraj_visualizer_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS realflight_trajectory_visualizer_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS px4_native_position_node DESTINATION lib/${PROJECT_NAME})
# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})
# Install launch and config
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(external_execution_test test/external_execution_test.cpp src/external_execution.cpp)
  target_link_libraries(external_execution_test px4ctrl_reference)
  ament_target_dependencies(external_execution_test rclcpp gap_msgs std_srvs)
  set_tests_properties(external_execution_test PROPERTIES ENVIRONMENT
    "ROS_DOMAIN_ID=188;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=/tmp/px4ctrl_external_execution_test")
  ament_add_gtest(external_trajectory_test test/external_trajectory_test.cpp)
  target_link_libraries(external_trajectory_test px4ctrl_reference)
  ament_add_gtest(takeoff_origin_test test/takeoff_origin_test.cpp)
  target_include_directories(takeoff_origin_test PRIVATE include)
  target_link_libraries(takeoff_origin_test Eigen3::Eigen)
  ament_add_gtest(landing_reference_test test/landing_reference_test.cpp)
  target_include_directories(landing_reference_test PRIVATE include)
  target_link_libraries(landing_reference_test Eigen3::Eigen)
  ament_add_gtest(motor_speed_control_test test/motor_speed_control_test.cpp)
  target_include_directories(motor_speed_control_test PRIVATE include)
  target_link_libraries(motor_speed_control_test Eigen3::Eigen)
  ament_target_dependencies(motor_speed_control_test rclcpp px4_msgs px4debug_msgs uav_utils)
  ament_add_gtest(omtraj_test test/omtraj_test.cpp)
  target_link_libraries(omtraj_test px4ctrl_omtraj)
  ament_add_gtest(ommpc_test test/ommpc_test.cpp)
  target_link_libraries(ommpc_test px4ctrl_ommpc)
  ament_add_gtest(acados_nmpc_test test/acados_nmpc_test.cpp)
  target_link_libraries(acados_nmpc_test px4ctrl_acados_nmpc)
  ament_add_gtest(trajectory_test test/trajectory_test.cpp)
  target_link_libraries(trajectory_test px4ctrl_reference px4ctrl_omtraj)
  ament_add_gtest(control_reference_test test/control_reference_test.cpp)
  target_link_libraries(control_reference_test px4ctrl_reference)
  ament_add_gtest(controller_adapter_test
    test/controller_adapter_test.cpp src/acados_nmpc.cpp src/frame_transforms.cpp)
  target_link_libraries(controller_adapter_test px4ctrl_reference px4ctrl_acados_nmpc osqp::osqp)
  ament_target_dependencies(controller_adapter_test
    rclcpp geometry_msgs sensor_msgs px4_msgs fms_utils uav_utils px4debug_msgs joy_msgs)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
