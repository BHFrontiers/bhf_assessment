FROM osrf/ros:humble-core

# Install packages
RUN apt-get update && apt-get install -y \
    ros-humble-desktop-full \
    python3-colcon-common-extensions \
    build-essential \
    cmake \
    git \
    python3-vcstool \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-compressed_image_transport \
    ros-humble-rqt-image-view \
    ros-humble-rclcpp \
    ros-humble-std-msgs \
    libopencv-dev \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*


# Setup a ROS2 workspace
WORKDIR /home/ros2_ws/src
RUN git clone https://github.com/Arif-Anjum/bhf_assessment.git

WORKDIR /home/ros2_ws

# Install dependencies
RUN . /opt/ros/humble/setup.sh && \
    apt-get update && \
    rosdep init && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y
WORKDIR /home/ros2_ws

#Build Packages
RUN . /opt/ros/humble/setup.bash && \
    colcon build

# Source the ROS 2 and workspace setup scripts
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc

#Entrypoint File
WORKDIR /home/ros2_ws/src/bhf_assessment/docker
RUN chmod +x entrypoint.sh

ENTRYPOINT [ "/home/ros2_ws/src/bhf_assessment/docker/entrypoint.sh" ]

# Launch the ROS node
CMD [ "bash" ]