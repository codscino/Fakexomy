cmake_minimum_required(VERSION 3.5)
project(raw_data_publisher)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Find rosidl default generators
find_package(rosidl_default_generators REQUIRED)

# Add the message file
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MyFloatArray.msg"
)

# Install Python packages
ament_python_install_package()

# Install directories
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
