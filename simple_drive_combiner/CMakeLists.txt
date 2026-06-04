cmake_minimum_required(VERSION 3.8)
project(simple_drive_combiner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)       # <--- ADD THIS
find_package(geometry_msgs REQUIRED)  # <--- ADD THIS

# --- ADD THESE LINES ---
# Define the executable binary and its source file
add_executable(simple_drive_combiner_node src/simple_drive_combiner.cpp)

# Link the ROS 2 dependencies to the executable
ament_target_dependencies(simple_drive_combiner_node rclcpp ackermann_msgs nav_msgs std_msgs geometry_msgs)

# Install the executable to the lib directory so 'ros2 run' can find it
install(TARGETS
  simple_drive_combiner_node
  DESTINATION lib/${PROJECT_NAME}
)
# ----------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()