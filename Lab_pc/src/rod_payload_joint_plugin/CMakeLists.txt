cmake_minimum_required(VERSION 3.5)
project(rod_payload_joint_plugin)

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

find_package(gazebo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ros2_muavp_interface REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")



add_library(RodPayloadJointPlugin SHARED src/plugin.cpp)
target_link_libraries(RodPayloadJointPlugin
        ${GAZEBO_LIBRARIES} ${rclcpp_LIBRARIES})
        
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
        

ament_target_dependencies(RodPayloadJointPlugin rclcpp rclcpp_components ros2_muavp_interface sensor_msgs)
rclcpp_components_register_nodes(RodPayloadJointPlugin "RodPayloadJointPlugin")

# add_executable(joint_plugin_node src/plugin.cpp)
# target_include_directories(joint_plugin_node PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
  
# target_link_libraries(joint_plugin_node JointPlugin)

ament_export_dependencies(rclcpp)

# install(TARGETS joint_plugin_node DESTINATION lib/${PROJECT_NAME})

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
  

