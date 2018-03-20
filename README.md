# AMR-group-7
https://github.com/LCAS/teaching/wiki/CMP3103M


#simulator
roslaunch uol_turtlebot_simulator labc.launch
roslaunch uol_turtlebot_simulator object-search-training.launch

#real robot
ssh computing@10.82.0.67
roslaunch uol_turtlebot_common turtlebot.launch
wget https://gist.githubusercontent.com/marc-hanheide/1ac6d9dfd6e89d4f6126/raw/45028eb2a212c432e470b17a4c3998af6f13b09b/ros-network.sh -P ~/
source ~/ros-network.sh 10.82.0.67
rosservice call /reset_bumper_stop
