cmake_minimum_required(VERSION 3.5)
project(offboard_rtps)

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
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ros2_muavp_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

add_library(OffboardRTPS SHARED src/offboard_rtps_handler.cpp)
target_link_libraries(OffboardRTPS ${rclcpp_LIBRARIES})

ament_target_dependencies(OffboardRTPS rclcpp rclcpp_components px4_msgs ros2_muavp_interface std_msgs)

rclcpp_components_register_nodes(OffboardRTPS "OffboardRTPS")

add_executable(offboard_rtps_main src/offboard_rtps_main.cpp)
target_include_directories(offboard_rtps_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(offboard_rtps_main OffboardRTPS)
  
ament_export_dependencies(rclcpp)

install(TARGETS offboard_rtps_main
  DESTINATION lib/${PROJECT_NAME})

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
