# ufo_detector
robot wars will never be the same
Contains a detection node that can detect other robots with no prerequisite knowledge about them using only a LIDAR.
Contains a kill node. To kill the detected robots.
Warning: This does not differentiate between humans and robots. Use with caution at your own risk.
How to use:
Got windows? Get ubuntu 20.04.
Get ROS
go to workspace/src
git clone https://github.com/DavidRhysGemmell/ufo_detector
cd ..
cakin_make

Launch your lidar sensor
roslaunch ufo_detector UFO_detect_and_attack.launch
