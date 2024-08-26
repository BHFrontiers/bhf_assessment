FROM osrf/ros:humble-core

# Install packages
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-rclcpp \
    ros-humble-std-msgs \
    libopencv-dev

# Setup a ROS2 workspace
WORKDIR /home/ros2_ws

# Copy the ROS package into the workspace
COPY ./src /home/ros2_ws/src/

# Install dependencies
RUN . /opt/ros/humble/setup.sh && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y
WORKDIR /home/ros2_ws

#Build Packages
RUN . /opt/ros/humble/setup.bash && \
    colcon build

#Source Workspace
RUN bash -c "source /opt/ros/humble/setup.bash && source /home/ros2_ws/install/setup.bash"

#Entrypoint File
WORKDIR /home/ros2_ws/src/bhf_assessment/docker
RUN chmod +x entrypoint.sh

ENTRYPOINT [ "/home/ros2_ws/src/bhf_assessment/docker/entrypoint.sh" ]