# multirobot
Project of multirobot

#First run the multiturtlebot world
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world_multi.launch

#run rvis and navigation
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_navigation turtlebot3_navigation_multi.launch

#move to the place you store leader_follower file.
cd catkin_ws/src/turtlebot3/turtlebot3_slam/launch

#change its mod
 chmod +x leader_follower.py
 
 #run the python file.
 rosrun turtlebot3_slam leader_follower.py
