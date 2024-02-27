FROM osrf/ros:noetic-desktop-full
LABEL authors="Artyom"
ADD src autoware_mini_practice/src
ADD run.sh autoware_mini_practice/
WORKDIR autoware_mini_practice

#RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
#        > /etc/apt/sources.list.d/ros-latest.list'
#
#RUN apt-get install -y wget
#
#RUN wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

RUN apt-get update && apt-get install -y \
      python3-catkin-tools \
      git \
      python3 \
      ros-${ROS_DISTRO}-novatel-oem7-driver \
      ros-${ROS_DISTRO}-autoware-msgs \
      ros-${ROS_DISTRO}-jsk-visualization \
      pip && \
      rm -rf /var/lib/apt/lists/*

RUN pip install pyproj

SHELL ["/bin/bash", "-c"]

RUN chmod +x run.sh

ENTRYPOINT ./run.sh