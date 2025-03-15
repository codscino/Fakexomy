# Use a ROS 2 base image for aarch64
FROM ros:humble-ros-base

# Set the working directory
WORKDIR /ros2_ws

# Ensure dependencies are installed
RUN apt update && apt install -y \
    python3-pip \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-ros2launch \
    ros-humble-rclpy \
    ros-humble-rclcpp \
    ros-humble-sensor-msgs \
    ros-humble-image-transport \
    && rm -rf /var/lib/apt/lists/*

RUN pip install ArducamDepthCamera
RUN pip install opencv-python-headless

# Set the shell to bash for the subsequent commands
SHELL ["/bin/bash", "-c"]

# Set up the ROS 2 workspace
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# Create workspace and Arducam folder inside the container
RUN mkdir -p /ros2_ws/src

WORKDIR /ExoMy_Sensors
# Set environment variables
ENV ROS_DOMAIN_ID=7
ENV COLCON_WS=/ros2_ws
RUN colcon build

# Default command: Start a bash shell
CMD ["/bin/bash"]
