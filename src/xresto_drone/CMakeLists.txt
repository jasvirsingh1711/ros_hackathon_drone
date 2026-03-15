cmake_minimum_required(VERSION 3.8)
project(xresto_drone)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(xacro REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  urdf
  meshes
  worlds
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  src/hover_node.py
  src/trajectory_planner.py
  src/gate_navigator.py
  src/altitude_diagnostics.py
  src/landing_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

# C++ executable for gate detector
add_executable(gate_detector src/gate_detector.cpp)
ament_target_dependencies(gate_detector 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  cv_bridge 
  OpenCV
  std_msgs
)

install(TARGETS
  gate_detector
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
