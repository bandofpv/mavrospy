# Use mavrospy-jetson as the base image
FROM mavrospy-jetson:latest

# Source the ROS and MAVROSPY setup.bash files and launch the outdoor.launch file
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && source /home/rpi/catkin_ws/devel/setup.bash && roslaunch mavrospy outdoor.launch fcu_url:=/dev/ttyUSB0:921600 pattern:=square_head"]
