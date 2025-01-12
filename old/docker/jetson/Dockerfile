# Use ROS Noetic as the base image
FROM ros:noetic

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    sudo \
    wget \
    net-tools \
    python3-pip \
    vim \
    nano

# Install MAVROS
RUN apt-get install -y ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs && \
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    bash ./install_geographiclib_datasets.sh

# Clean up
RUN rm -rf /var/lib/apt/lists/* && \
    apt-get clean

# Set up jetson user
RUN useradd -m jetson

# Grant jetson permission to /dev/ttyUSB0
RUN usermod -aG dialout jetson

# Set the working directory for the jetson user
WORKDIR /home/jetson/catkin_ws/src

# Ensure the jetson user has the necessary permissions
RUN mkdir -p /home/jetson/catkin_ws/src && \
    chown -R jetson:jetson /home/jetson/catkin_ws

# Switch to jetson user
USER jetson

# Install pymavlink
RUN pip3 install pymavlink

# Set up ROS workspace
WORKDIR /home/jetson/catkin_ws/src
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_init_workspace"

# Clone MAVROSPY repo
RUN git clone https://github.com/bandofpv/mavrospy.git

# Install ROS package dependencies
WORKDIR /home/jetson/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && rosdep update" && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && rosdep install --from-paths src --ignore-src -r -y"

# Build the ROS workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Add start up commands to .bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/jetson/.bashrc && \
    echo "source /home/jetson/catkin_ws/devel/setup.bash" >> /home/jetson/.bashrc && \
    echo "export ROS_IP=$(hostname -I | awk '{print $1}')" >> /home/jetson/.bashrc && \
    echo "export ROS_MASTER_URI=http://$(hostname -I | awk '{print $1}'):11311" >> /home/jetson/.bashrc && \
    echo "export FCU_URL=/dev/ttyUSB0:921600" >> /home/jetson/.bashrc

# Start in MAVROSPY direcotry
WORKDIR /home/jetson/catkin_ws/src/mavrospy

# Set up entrypoint
ENTRYPOINT ["/bin/bash"]
