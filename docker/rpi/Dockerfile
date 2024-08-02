# Use ROS Noetic as the base image
FROM ros:noetic

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt install -y \
    git \
    sudo \
    wget \
    net-tools \
    openssh-client

# Set up dev user
RUN useradd -m dev && echo "dev:dev" | chpasswd && adduser dev sudo

# Configure sudoers to not require a password for sudo
RUN echo "dev ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Install MAVROS
RUN sudo apt install -y ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs && \
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    sudo bash ./install_geographiclib_datasets.sh

# Switch to dev user
USER dev

# Set up SSH for cloning the repository
RUN mkdir -p /home/dev/.ssh && \
    ssh-keyscan github.com >> /home/dev/.ssh/known_hosts

# Set up ROS workspace
WORKDIR /home/dev/catkin_ws/src
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_init_workspace"

# Clone MAVROSPY and packages
RUN git clone https://github.com/bandofpv/mavrospy.git

# Install ROS package dependencies
WORKDIR /home/dev/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && rosdep update" && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && rosdep install --from-paths src --ignore-src -r -y"

# Build the ROS workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Add ROS Environment Setup to .bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/dev/.bashrc && \
    echo "source /home/dev/catkin_ws/devel/setup.bash" >> /home/dev/.bashrc

# Set up entrypoint
ENTRYPOINT ["/bin/bash"]
