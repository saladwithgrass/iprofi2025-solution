cmake_minimum_required(VERSION 3.5)
project(solution_bachelor)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # For C++ nodes
find_package(std_msgs REQUIRED)

find_package(rclpy REQUIRED)   # For Python nodes

# Include directories
include_directories(include)

# Declare a C++ executable
add_executable(solution_node src/solution.cpp)

# Link libraries
ament_target_dependencies(solution_node rclcpp std_msgs)

# Install executables
install(TARGETS solution_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/solution.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Enable testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ament package export
ament_package()
