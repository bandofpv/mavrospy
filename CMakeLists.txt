cmake_minimum_required(VERSION 3.0.2)
project(mavrospy)

find_package(catkin REQUIRED COMPONENTS
  tf
  rospy
  std_msgs
  mavros_msgs
  geometry_msgs
)
