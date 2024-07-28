# Use ROS Noetic as the base image
FROM ros:noetic

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt install -y \
    git \
    ca-certificates \
    gnupg \
    lsb-core \
    sudo \
    wget \
    curl \
    net-tools

# Set up dev user
RUN useradd -m dev && echo "dev:dev" | chpasswd && adduser dev sudo

# Configure sudoers to not require a password for sudo
RUN echo "dev ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Install Gazebo ROS Packages
RUN sudo apt install -y ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control

# Install MAVROS
RUN sudo apt install -y ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs && \
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    sudo bash ./install_geographiclib_datasets.sh

# Switch to dev user
USER dev

# Set up PX4 environment
WORKDIR /home/dev
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive
RUN bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

# Configure Gazebo Classic Simulator
WORKDIR /home/dev/PX4-Autopilot
RUN DONT_RUN=1 make px4_sitl_default gazebo-classic

# Set up ROS workspace
WORKDIR /home/dev/catkin_ws/src
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_init_workspace"

# Clone MAVROSPY and QTM ROS packages
RUN git clone https://github.com/KTH-SML/motion_capture_system.git
RUN git clone https://github.com/bandofpv/mavrospy.git

# Install ROS package dependencies
WORKDIR /home/dev/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && rosdep update" && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && rosdep install --from-paths src --ignore-src -r -y"

# Build the ROS workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Add ROS Environment Setup to .bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/dev/.bashrc && \
    echo "source /home/dev/catkin_ws/devel/setup.bash" >> /home/dev/.bashrc && \
    echo "source /home/dev/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash /home/dev/PX4-Autopilot /home/dev/PX4-Autopilot/build/px4_sitl_default" >> /home/dev/.bashrc && \
    echo "export ROS_PACKAGE_PATH=/home/dev/catkin_ws/src:/opt/ros/noetic/share:/home/dev/PX4-Autopilot:/home/dev/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic" >> /home/dev/.bashrc

# Set up entrypoint
ENTRYPOINT ["/bin/bash"]
