cmake_minimum_required(VERSION 3.5)
project(solution_bachelor)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # For C++ nodes
find_package(rclpy REQUIRED)   # For Python nodes
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# for my msgs
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED) # for custom msgs

# add my message
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TwistStampedArray.msg"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)


# Include directories
include_directories(include/${PROJECT_NAME})
include_directories(${OpenCV_INCLUDE_DIRS})

# Declare a C++ executable
add_executable(solution_node src/solution.cpp)

# Link libraries
ament_target_dependencies(solution_node rclcpp std_msgs sensor_msgs nav_msgs OpenCV cv_bridge)

# Install executables
install(TARGETS solution_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/lidar_v2.py
  scripts/mapper.py
  scripts/director.py
  scripts/pid_controller.py
  scripts/control_summator.py
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
ament_export_dependencies(rosidl_default_runtime)
ament_package()
