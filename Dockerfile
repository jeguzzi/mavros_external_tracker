FROM ros:latest
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y \
    python-pip \
    ros-kinetic-mavros \
    ros-kinetic-tf \
    ros-kinetic-message-to-tf \
    wget
# RUN pip install --upgrade pip

RUN mkdir -p /home/root/catkin_ws/src
COPY . /home/root/catkin_ws/src/mavros_external_tracker
# RUN mkdir -p /home/root/catkin_ws/src/mavros_external_tracker/urdf
# RUN wget -O /home/root/catkin_ws/src/mavros_external_tracker/urdf/rover_scaled.urdf https://raw.githubusercontent.com/erlerobot/ardupilot_sitl_gazebo_plugin/master/ardupilot_sitl_gazebo_plugin/urdf/rover_scaled.urdf

RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; catkin_init_workspace /home/root/catkin_ws/src; catkin_make -C /home/root/catkin_ws;'
RUN /bin/sed -i \
    '/source "\/opt\/ros\/$ROS_DISTRO\/setup.bash"/a source "\/home\/root\/catkin_ws\/devel\/setup.bash"' \
    /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# RUN apt-get install -y  ros-kinetic-xacro
