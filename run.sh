export DISPLAY=host.docker.internal:0

source /opt/ros/noetic/setup.bash

catkin build

source devel/setup.bash

roslaunch practice_2 practice_2.launch

bash