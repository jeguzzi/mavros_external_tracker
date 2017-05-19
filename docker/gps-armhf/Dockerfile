FROM jeguzzi/ros:armhf-kinetic-ros-dev
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y \
    ros-kinetic-nmea-navsat-driver \
    ros-kinetic-diagnostic-updater \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/KumarRobotics/ublox.git src/ublox

RUN catkin build
