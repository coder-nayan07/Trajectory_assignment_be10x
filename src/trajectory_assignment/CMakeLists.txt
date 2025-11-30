cmake_minimum_required(VERSION 3.8)
project(trajectory_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 2. Define Planner Node
add_executable(planner_node src/planner_node.cpp)
ament_target_dependencies(planner_node rclcpp geometry_msgs nav_msgs)

# 3. Define Controller Node
add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node 
  rclcpp 
  nav_msgs 
  geometry_msgs 
  tf2 
  tf2_geometry_msgs
)

# 4. Install BOTH executables
install(TARGETS
  planner_node
  controller_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
