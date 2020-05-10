# ROB_545_Proj

Installation:

1. sudo apt-get install ros-melodic-pr2*
2. Copy files from src to your workspace src
3. catkin_make
4. All the .tar files will be used to create environment 
5. Files from env folder can be pasted in usr/share/gazebo in their respective folders for ease of usage

Execution:

1. To launch PR2 in empty world: roslaunch pr2_gazebo pr2_empty_world.launch
2. To launch environment from gazebo: gazebo worlds/<env name>
