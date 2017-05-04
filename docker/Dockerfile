FROM jeguzzi/ros:kinetic-ros-dev
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y \
    ros-kinetic-mavros \
    ros-kinetic-mavros-extras \
    ros-kinetic-tf \
    ros-kinetic-message-to-tf \
    ros-kinetic-tf2-geometry-msgs \
    ros-kinetic-dynamic-reconfigure \
    && rm -rf /var/lib/apt/lists/*

COPY . src/mavros_external_tracker
RUN catkin build

# RUN mkdir -p /home/root/catkin_ws/src/mavros_external_tracker/urdf
# RUN wget -O /home/root/catkin_ws/src/mavros_external_tracker/urdf/rover_scaled.urdf https://raw.githubusercontent.com/erlerobot/ardupilot_sitl_gazebo_plugin/master/ardupilot_sitl_gazebo_plugin/urdf/rover_scaled.urdf
# RUN apt-get install -y  ros-kinetic-xacro
