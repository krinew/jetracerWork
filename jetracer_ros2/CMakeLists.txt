cmake_minimum_required(VERSION 3.12)
project(jetracer_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# We will add executables here in later steps

# Disable linting tests - ament_lint_auto not installed on this system
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   set(ament_cmake_copyright_FOUND TRUE)
#   set(ament_cmake_uncrustify_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

add_executable(jetracer src/jetracer.cpp)
target_include_directories(jetracer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(jetracer
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS jetracer
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}/)

# Install Python scripts
install(PROGRAMS
  scripts/odom_ekf.py
  scripts/calibrate_linear.py
  DESTINATION lib/${PROJECT_NAME}
)





ament_package()
