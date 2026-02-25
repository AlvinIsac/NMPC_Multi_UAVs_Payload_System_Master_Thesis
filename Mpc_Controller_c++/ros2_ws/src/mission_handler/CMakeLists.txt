cmake_minimum_required(VERSION 3.5)
project(mission_handler)

# ------------------------------------------------------------------------------
# Language standards
# ------------------------------------------------------------------------------
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ------------------------------------------------------------------------------
# Compiler warnings
# ------------------------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # Enable all basic warnings, but silence common noisy ones
  add_compile_options(-Wall -Wno-unused-variable -Wno-deprecated -Wno-sign-compare)
  add_compile_options(-Wall -Wno-class-memaccess -Wno-unused-variable -Wno-deprecated)
endif()

# ------------------------------------------------------------------------------
# Find ROS 2 and external dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ros2_muavp_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)

# ------------------------------------------------------------------------------
# Include directories
# ------------------------------------------------------------------------------
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  $ENV{HOME}/casadi/install/include
)

# Link directory for CasADi
link_directories($ENV{HOME}/casadi/install/lib)

# ------------------------------------------------------------------------------
# Add Executables
# ------------------------------------------------------------------------------

add_executable(mpc_failure_handler2_node
  src/states_handler_node/mpc_failure_handler2.cpp
)

target_include_directories(mpc_failure_handler2_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $ENV{HOME}/casadi/install/include
)

target_link_libraries(mpc_failure_handler2_node
  casadi
)

ament_target_dependencies(mpc_failure_handler2_node
  rclcpp
  ament_index_cpp
  ros2_muavp_interface
  geometry_msgs
  Eigen3
)

add_executable(mpc_failure_handler3_node
  src/states_handler_node/mpc_failure_handler3.cpp
)

target_include_directories(mpc_failure_handler3_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $ENV{HOME}/casadi/install/include
)

target_link_libraries(mpc_failure_handler3_node
  casadi
)

ament_target_dependencies(mpc_failure_handler3_node
  rclcpp
  ament_index_cpp
  ros2_muavp_interface
  geometry_msgs
  Eigen3
)

# New MPC main executable
add_executable(mpc_main_node
  src/states_handler_node/mpc_main.cpp
)

target_include_directories(mpc_main_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $ENV{HOME}/casadi/install/include
)

target_link_libraries(mpc_main_node
  casadi
)

ament_target_dependencies(mpc_main_node
  rclcpp
  ament_index_cpp
  ros2_muavp_interface
  geometry_msgs
  Eigen3
)

# Input setpoint booster node
add_executable(input_setpoint_booster_node
  src/states_handler_node/input_setpoint_booster.cpp
)

ament_target_dependencies(input_setpoint_booster_node
  rclcpp
  ros2_muavp_interface
)

add_executable(ref_generator_node src/states_handler_node/ref_generator.cpp)
target_include_directories(ref_generator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ref_generator_node
  rclcpp
  ros2_muavp_interface
)

# Add the new MPC UAV executable
add_executable(mpc_uav_node
  src/states_handler_node/mpc_uav.cpp
)

target_include_directories(mpc_uav_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $ENV{HOME}/casadi/install/include
)

target_link_libraries(mpc_uav_node
  casadi
)

ament_target_dependencies(mpc_uav_node
  rclcpp
  ament_index_cpp
  ros2_muavp_interface
  geometry_msgs
  Eigen3
)

# Add the new MPC 30 states executable
add_executable(mpc_main_30_states_node
  src/states_handler_node/mpc_main_30_states.cpp
)

target_include_directories(mpc_main_30_states_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $ENV{HOME}/casadi/install/include
)

target_link_libraries(mpc_main_30_states_node
  casadi
)

ament_target_dependencies(mpc_main_30_states_node
  rclcpp
  ament_index_cpp
  ros2_muavp_interface
  geometry_msgs
  Eigen3
)

# Add the new MPC 30 states with tension executable
add_executable(mpc_main_30_states_with_tension_node
  src/states_handler_node/mpc_main_30_states_with_tension.cpp
)

target_include_directories(mpc_main_30_states_with_tension_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $ENV{HOME}/casadi/install/include
)

target_link_libraries(mpc_main_30_states_with_tension_node
  casadi
)

ament_target_dependencies(mpc_main_30_states_with_tension_node
  rclcpp
  ament_index_cpp
  ros2_muavp_interface
  geometry_msgs
  Eigen3
)

# Add the new UAV1 hover controller executable
add_executable(mpc_uav1_hover_node
  src/states_handler_node/mpc_uav1_hover.cpp
)

target_include_directories(mpc_uav1_hover_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $ENV{HOME}/casadi/install/include
)

target_link_libraries(mpc_uav1_hover_node
  casadi
)

ament_target_dependencies(mpc_uav1_hover_node
  rclcpp
  ament_index_cpp
  ros2_muavp_interface
  geometry_msgs
  Eigen3
)

# ------------------------------------------------------------------------------
# Install Targets
# ------------------------------------------------------------------------------
install(TARGETS
  mpc_failure_handler2_node
  mpc_failure_handler3_node
  mpc_main_node
  ref_generator_node
  input_setpoint_booster_node
  mpc_uav_node
  mpc_main_30_states_node
  mpc_main_30_states_with_tension_node
  mpc_uav1_hover_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
