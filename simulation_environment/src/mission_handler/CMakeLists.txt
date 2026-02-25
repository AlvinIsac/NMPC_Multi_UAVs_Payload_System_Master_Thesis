cmake_minimum_required(VERSION 3.5)
project(mission_handler)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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
find_package(ament_index_cpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ros2_muavp_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)

# uncomment the following section in order to fill in further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

# Edit
include_directories(include)
include_directories($ENV{HOME}/casadi/include)
link_directories($ENV{HOME}/casadi/build)

###############################

add_library(TransitionsHandler SHARED src/transitions_handler_node/transitions_handler.cpp src/transitions_handler_node/Parser.cpp)
target_link_libraries(TransitionsHandler ${rclcpp_LIBRARIES} config++)

add_library(StatesHandler SHARED src/states_handler_node/states_handler.cpp src/states_handler_node/preflight_checks.cpp src/states_handler_node/states_methods.cpp src/states_handler_node/failure_states_methods.cpp src/states_handler_node/control_algorithms.cpp src/states_handler_node/control_supports.cpp src/states_handler_node/utils.cpp src/states_handler_node/formation_optimizer.cpp src/states_handler_node/trajectory_objects.cpp)
target_link_libraries(StatesHandler ${rclcpp_LIBRARIES} config++)

add_library(FailuresHandler SHARED src/failures_handler_node/failures_handler.cpp)
target_link_libraries(FailuresHandler ${rclcpp_LIBRARIES} config++)

###############################

ament_target_dependencies(TransitionsHandler rclcpp rclcpp_components px4_msgs ros2_muavp_interface ament_index_cpp)

rclcpp_components_register_nodes(TransitionsHandler "TransitionsHandler")


ament_target_dependencies(StatesHandler rclcpp rclcpp_components px4_msgs ros2_muavp_interface ament_index_cpp)

rclcpp_components_register_nodes(StatesHandler "StatesHandler")


ament_target_dependencies(FailuresHandler rclcpp rclcpp_components px4_msgs ros2_muavp_interface ament_index_cpp)

rclcpp_components_register_nodes(FailuresHandler "FailuresHandler")

###############################

add_executable(transitions_handler_main src/transitions_handler_node/transitions_handler_main.cpp)
target_include_directories(transitions_handler_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(transitions_handler_main TransitionsHandler)

add_executable(states_handler_main src/states_handler_node/states_handler_main.cpp)
target_include_directories(states_handler_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(states_handler_main StatesHandler)

add_executable(failures_handler_main src/failures_handler_node/failures_handler_main.cpp)
target_include_directories(failures_handler_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(failures_handler_main FailuresHandler)

## Edit ##
add_executable(failure_monitor_node src/states_handler_node/failure_monitor.cpp)
ament_target_dependencies(failure_monitor_node rclcpp ros2_muavp_interface)

# === Updated Section ===
add_executable(mpc_failure_handler2_node src/states_handler_node/mpc_failure_handler2.cpp)
target_include_directories(mpc_failure_handler2_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mpc_failure_handler2_node StatesHandler casadi)
ament_target_dependencies(mpc_failure_handler2_node
  rclcpp
  Eigen3
  ros2_muavp_interface
  geometry_msgs
)

add_executable(mpc_failure_handler3_node src/states_handler_node/mpc_failure_handler3.cpp)
target_include_directories(mpc_failure_handler3_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mpc_failure_handler3_node StatesHandler casadi)
ament_target_dependencies(mpc_failure_handler3_node
  rclcpp
  Eigen3
  ros2_muavp_interface
  geometry_msgs
)

add_executable(ref_generator_node src/states_handler_node/ref_generator.cpp)
target_include_directories(ref_generator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ref_generator_node
  rclcpp
  ros2_muavp_interface
)

# New MPC main executable
add_executable(mpc_main_node src/states_handler_node/mpc_main.cpp)
target_include_directories(mpc_main_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mpc_main_node StatesHandler casadi)
ament_target_dependencies(mpc_main_node
  rclcpp
  Eigen3
  ros2_muavp_interface
  geometry_msgs
)

# Input setpoint booster nodes (separate for each UAV)
add_executable(input_setpoint_booster_uav1_node
  src/states_handler_node/input_setpoint_booster_uav1.cpp
)
ament_target_dependencies(input_setpoint_booster_uav1_node
  rclcpp
  ros2_muavp_interface
)

add_executable(input_setpoint_booster_uav2_node
  src/states_handler_node/input_setpoint_booster_uav2.cpp
)
ament_target_dependencies(input_setpoint_booster_uav2_node
  rclcpp
  ros2_muavp_interface
)

add_executable(input_setpoint_booster_uav3_node
  src/states_handler_node/input_setpoint_booster_uav3.cpp
)
ament_target_dependencies(input_setpoint_booster_uav3_node
  rclcpp
  ros2_muavp_interface
)

# MPC/PID switcher nodes
add_executable(mpc_pid_switcher_x500_1_node
  src/states_handler_node/mpc_pid_switcher_x500_1.cpp
)
ament_target_dependencies(mpc_pid_switcher_x500_1_node
  rclcpp
  ros2_muavp_interface
)

add_executable(mpc_pid_switcher_x500_2_node
  src/states_handler_node/mpc_pid_switcher_x500_2.cpp
)
ament_target_dependencies(mpc_pid_switcher_x500_2_node
  rclcpp
  ros2_muavp_interface
)

add_executable(mpc_pid_switcher_x500_3_node
  src/states_handler_node/mpc_pid_switcher_x500_3.cpp
)
ament_target_dependencies(mpc_pid_switcher_x500_3_node
  rclcpp
  ros2_muavp_interface
)

# MPC main 30 states executable
add_executable(mpc_main_30_states_node src/states_handler_node/mpc_main_30_states.cpp)
target_include_directories(mpc_main_30_states_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mpc_main_30_states_node StatesHandler casadi)
ament_target_dependencies(mpc_main_30_states_node
  rclcpp
  Eigen3
  ros2_muavp_interface
  geometry_msgs
)

# MPC main 30 states with tension executable
add_executable(mpc_main_30_states_with_tension_node src/states_handler_node/mpc_main_30_states_with_tension.cpp)
target_include_directories(mpc_main_30_states_with_tension_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mpc_main_30_states_with_tension_node StatesHandler casadi)
ament_target_dependencies(mpc_main_30_states_with_tension_node
  rclcpp
  Eigen3
  ros2_muavp_interface
  geometry_msgs
)

# MPC UAV1 hover executable
add_executable(mpc_uav1_hover_node src/states_handler_node/mpc_uav1_hover.cpp)
target_include_directories(mpc_uav1_hover_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mpc_uav1_hover_node StatesHandler casadi)
ament_target_dependencies(mpc_uav1_hover_node
  rclcpp
  Eigen3
  ros2_muavp_interface
  geometry_msgs
)


###############################

ament_export_dependencies(rclcpp)

install(TARGETS transitions_handler_main
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS states_handler_main
  DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS failures_handler_main
  DESTINATION lib/${PROJECT_NAME})

## Updated install section ##
install(TARGETS mpc_failure_handler2_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS mpc_failure_handler3_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  ref_generator_node
  DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS
  failure_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install new executables
install(TARGETS
  mpc_main_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install input setpoint booster nodes (separate for each UAV)
install(TARGETS
  input_setpoint_booster_uav1_node
  input_setpoint_booster_uav2_node
  input_setpoint_booster_uav3_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install MPC/PID switcher nodes
install(TARGETS
  mpc_pid_switcher_x500_1_node
  mpc_pid_switcher_x500_2_node
  mpc_pid_switcher_x500_3_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install new MPC executables
install(TARGETS
  mpc_main_30_states_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  mpc_main_30_states_with_tension_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  mpc_uav1_hover_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  conf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
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
