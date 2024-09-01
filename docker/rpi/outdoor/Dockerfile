# Use mavrospy-rpi as the base image
FROM mavrospy-rpi:latest

# Source the ROS and MAVROSPY setup.bash files and launch the outdoor.launch file
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && source /home/rpi/catkin_ws/devel/setup.bash && sudo chown rpi /dev/serial0 && roslaunch mavrospy outdoor.launch"]
