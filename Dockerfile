# Base ROS 2 Jazzy environment with GUI and OpenGL acceleration
FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Install MoveIt 2, Gazebo Sim, and Control dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-servo \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-joint-trajectory-controller \
    ros-${ROS_DISTRO}-tf2-ros \
    python3-colcon-common-extensions \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Set up the workspace
WORKDIR /root/manipulator_ws

# Copy ROS 2 source packages
COPY src/ /root/manipulator_ws/src/

# Source ROS 2 and build packages
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install

# Automatically source ROS 2 and the workspace on container startup
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /root/manipulator_ws/install/setup.bash" >> /root/.bashrc

# Default entrypoint starts the Gazebo Sim + MoveIt 2 digital twin
CMD ["bash", "-c", "source /root/manipulator_ws/install/setup.bash && ros2 launch manipulator_gazebo gazebo_moveit.launch.py"]
