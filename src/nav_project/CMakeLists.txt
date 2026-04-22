cmake_minimum_required(VERSION 3.8)
project(nav_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find all required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav_interfaces REQUIRED)
find_package(tf2 REQUIRED)

# --- 1. Navigation Server Component ---
# Compile as a shared library
add_library(navigation_server_component SHARED src/navigation_server_component.cpp)
ament_target_dependencies(navigation_server_component
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  nav_msgs
  nav_interfaces
  tf2
)
# Register the component plugin
rclcpp_components_register_node(navigation_server_component
  PLUGIN "nav_project::NavigationServer"
  EXECUTABLE navigation_server_exe
)

# --- 2. UI Client Component ---
# Compile as a shared library
add_library(ui_client_component SHARED src/ui_client_component.cpp)
ament_target_dependencies(ui_client_component
  rclcpp
  rclcpp_action
  rclcpp_components
  nav_interfaces
)
# Register the component plugin
rclcpp_components_register_node(ui_client_component
  PLUGIN "nav_project::UIClient"
  EXECUTABLE ui_client_exe
)

# Install the shared libraries and executables so ROS 2 can find them
install(TARGETS
  navigation_server_component
  ui_client_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
