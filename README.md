# graph_optimization_mg_cs169

This is a git repository for graph optimization package for CS 69 / 169 class at Dartmouth Robotics perception course. Works are coded by Mingi Jeong, 1st year Ph.D. Students in Robotics/Computer Science at Dartmouth. This program was conducted on ROS kinetic and Linux Ubuntu 16.04

I am pleased to take this class taught by Prof.Alberto Quattrini Li.

# How to download and install necessary packages

1. git clone to your catkin_ws folder
2. Make sure you download rosbag file recorded from Husarion Rosbot 2.0 from the following link https://drive.google.com/open?id=1onVc88q--nnUFm7Kv0Schqn1i-yoMVre
3. Edit launch files for saving path parameters and etc.


# How to execute the program
1. source ~/catkin_ws/devel/setup.bash (afterwards recommended : rospack profile)
2. run command to make your python scripts executable e.g (chmod +x ~/catkin_ws/src/graph_optimization_mg_cs169/scripts/graph_optimization.py and plotter.py)
3. roslaunch graph_optimization_mg_cs169 graph_optimization.launch
