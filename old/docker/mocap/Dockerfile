# Use ROS Noetic as the base image
FROM ros:noetic

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    sudo \
    vim \
    nano \
    net-tools

# Set up mocap user
RUN useradd -m mocap && echo "mocap:mocap" | chpasswd && adduser mocap sudo

# Configure sudoers to not require a password for sudo
RUN echo "mocap ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Switch to mocap user
USER mocap

# Set up ROS workspace
WORKDIR /home/mocap/catkin_ws/src
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_init_workspace"

# Clone QTM ROS package
RUN git clone https://github.com/KTH-SML/motion_capture_system.git

# Install ROS package dependencies
WORKDIR /home/mocap/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && rosdep update" && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && rosdep install --from-paths src --ignore-src -r -y"

# Build the ROS workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Add ROS Environment Setup to .bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/mocap/.bashrc && \
    echo "source /home/mocap/catkin_ws/devel/setup.bash" >> /home/mocap/.bashrc && \
    echo "export ROS_IP=$(hostname -I | awk '{print $1}')" >> /home/mocap/.bashrc

# Set WORKDIR to src directory and clean package list
WORKDIR /home/mocap/catkin_ws/src/motion_capture_system

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/*  && \
    sudo apt-get clean

# Set up entrypoint
ENTRYPOINT ["/bin/bash"]
