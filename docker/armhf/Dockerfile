FROM jeguzzi/ros:armhf-kinetic-ros-dev
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
