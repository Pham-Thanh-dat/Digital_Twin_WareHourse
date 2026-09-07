cmake_minimum_required(VERSION 3.8)
project(multibot_controller)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(multibot_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
set(DEPS
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  std_srvs
  multibot_interfaces
  tf2
  tf2_ros
  sensor_msgs
)
# ── Library (component) ───────────────────────────────────────────────────────
add_library(multimode_controller_component SHARED
  src/multimode_controller_node.cpp
  src/goal_controllet.cpp
)
target_include_directories(multimode_controller_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(multimode_controller_component ${DEPS})
rclcpp_components_register_node(
  multimode_controller_component
  PLUGIN "multibot_controller::MultimodeControllerNode"
  EXECUTABLE multimode_controller_node
)

# ── Library (component) - Congestion Zone Manager ─────────────────────────────
add_library(congestion_zone_manager_component SHARED
  src/congestion_zone_manager.cpp
)
target_include_directories(congestion_zone_manager_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(congestion_zone_manager_component ${DEPS})
rclcpp_components_register_node(
  congestion_zone_manager_component
  PLUGIN "multibot_controller::CongestionZoneManagerNode"
  EXECUTABLE congestion_zone_manager_node
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS multimode_controller_component congestion_zone_manager_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  src/twist_to_wheels.py
  src/wheel_odom.py
  src/fix_scan_frame.py
  src/congestion_costmap.py
  src/assign_nav2_safe_routes.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
