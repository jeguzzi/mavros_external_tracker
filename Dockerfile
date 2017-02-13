FROM jeguzzi/ros:kinetic-ros-dev
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y \
    python-pip \
    ros-kinetic-mavros \
    ros-kinetic-tf \
    ros-kinetic-message-to-tf \
    wget

COPY . src/mavros_external_tracker
RUN catkin build

# RUN mkdir -p /home/root/catkin_ws/src/mavros_external_tracker/urdf
# RUN wget -O /home/root/catkin_ws/src/mavros_external_tracker/urdf/rover_scaled.urdf https://raw.githubusercontent.com/erlerobot/ardupilot_sitl_gazebo_plugin/master/ardupilot_sitl_gazebo_plugin/urdf/rover_scaled.urdf
# RUN apt-get install -y  ros-kinetic-xacro
