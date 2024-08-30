FROM osrf/ros:humble-desktop

# Install packages
RUN apt-get update && apt-get install -y \
    ros-humble-desktop-full \
    python3-colcon-common-extensions \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstool \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-compressed-image-transport \
    ros-humble-rqt-image-view \
    ros-humble-rclcpp \
    ros-humble-std-msgs \
    libopencv-dev \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*
    
# Initialize rosdep if not already initialized
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
    rosdep init; \
    fi
    
# Initialize rosdep
RUN rosdep update

# Setup a ROS2 workspace
WORKDIR /home/ros2_ws/src
RUN git clone https://github.com/Arif-Anjum/bhf_assessment.git

# Install dependencies for the workspace
WORKDIR /home/ros2_ws
RUN rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN . /opt/ros/humble/setup.sh && colcon build

# Source the ROS 2 and workspace setup scripts
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc

#Entrypoint File
WORKDIR /home/ros2_ws/src/bhf_assessment/docker
RUN chmod +x entrypoint.sh

ENTRYPOINT [ "/home/ros2_ws/src/bhf_assessment/docker/entrypoint.sh" ]

# Launch the ROS node
CMD [ "bash" ]