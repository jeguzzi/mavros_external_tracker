FROM jeguzzi/ros:armhf-kinetic-ros-dev
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y \
    python-pip \
    ros-kinetic-mavros \
    ros-kinetic-tf \
    ros-kinetic-message-to-tf \
    ros-kinetic-tf2-geometry-msgs \
    wget

COPY . src/mavros_external_tracker
RUN catkin build
