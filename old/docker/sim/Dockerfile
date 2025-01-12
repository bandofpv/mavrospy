# Use ROS Noetic as the base image
FROM ros:noetic

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    ca-certificates \
    gnupg \
    lsb-core \
    sudo \
    wget \
    curl \
    vim \
    nano \
    net-tools

# Set up sim user
RUN useradd -m sim && echo "sim:sim" | chpasswd && adduser sim sudo

# Configure sudoers to not require a password for sudo
RUN echo "sim ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Install Gazebo ROS Packages
RUN sudo apt-get install -y \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control

# Install RViz
RUN sudo apt-get install -y ros-noetic-rviz

# Install MAVROS
RUN sudo apt-get install -y \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
    ros-noetic-mavros-msgs && \
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    sudo bash ./install_geographiclib_datasets.sh

# Switch to sim user
USER sim

# Set up PX4 environment
WORKDIR /home/sim
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive
RUN bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

# Configure Gazebo Classic Simulator
WORKDIR /home/sim/PX4-Autopilot
RUN DONT_RUN=1 make px4_sitl_default gazebo-classic

# Set up ROS workspace
WORKDIR /home/sim/catkin_ws/src
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_init_workspace"

# Clone MAVROSPY package
RUN git clone https://github.com/bandofpv/mavrospy.git

# Install ROS package dependencies
WORKDIR /home/sim/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && rosdep update" && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && rosdep install --from-paths src --ignore-src -r -y"

# Build the ROS workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Add ROS Environment Setup to .bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/sim/.bashrc && \
    echo "source /home/sim/catkin_ws/devel/setup.bash" >> /home/sim/.bashrc && \
    echo "source /home/sim/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash /home/sim/PX4-Autopilot /home/sim/PX4-Autopilot/build/px4_sitl_default" >> /home/sim/.bashrc && \
    echo "export ROS_PACKAGE_PATH=/home/sim/catkin_ws/src:/opt/ros/noetic/share:/home/sim/PX4-Autopilot:/home/sim/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic" >> /home/sim/.bashrc

# Set WORKDIR to src directory and clean package list
WORKDIR /home/sim/catkin_ws/src/mavrospy

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/*  && \
    sudo apt-get clean

# Set up entrypoint
ENTRYPOINT ["/bin/bash"]
